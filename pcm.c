/*
 * Linux driver for D.O.Tec EXBOX.UMA (PloyTec USB interface)
 *
 * Copyright 2015 (C) Google Inc.
 *
 * Authors:  Adrian Knoth <adi@drcomp.erfurt.thur.de>
 *           Adrian Knoth <aknoth@google.com>
 *
 * The driver is based on the hiface driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/slab.h>
#include <linux/kfifo.h>
#include <sound/pcm.h>

#include "pcm.h"
#include "chip.h"

#define PCM_PACKET_SIZE 512
#define PCM_BYTES_PLAYBACK 480
#define PCM_BUFFER_SIZE (2 * PCM_N_URBS * PCM_PACKET_SIZE)

static const unsigned int rates[] = { 44100, 48000, 88200, 96000, 176400, 192000,
				      352800, 384000 };
static const struct snd_pcm_hw_constraint_list constraints_extra_rates = {
	.count = ARRAY_SIZE(rates),
	.list = rates,
	.mask = 0,
};

static const struct snd_pcm_hardware pcm_hw = {
	.info = SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_PAUSE |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_BATCH,

	.formats = SNDRV_PCM_FMTBIT_S32_LE,

	.rates = SNDRV_PCM_RATE_44100 |
		SNDRV_PCM_RATE_48000 |
		SNDRV_PCM_RATE_88200 |
		SNDRV_PCM_RATE_96000 |
		SNDRV_PCM_RATE_176400 |
		SNDRV_PCM_RATE_192000,

	.rate_min = 44100,
	.rate_max = 192000,
	.channels_min = 32,
	.channels_max = 32,
	.buffer_bytes_max = PCM_BUFFER_SIZE,
	.period_bytes_min = PCM_PACKET_SIZE,
	.period_bytes_max = 32 * 4 * 4096, /* 32ch * 4bytes * 4096 samples */
	.periods_min = 2,
	.periods_max = 1024
};

static int exbox_pcm_set_rate(struct pcm_runtime *rt, unsigned int rate)
{
	struct exbox_chip *chip = rt->chip;

	return snd_exbox_set_samplerate(chip, rate);
}

static struct pcm_substream *exbox_pcm_get_substream(struct snd_pcm_substream
						      *alsa_sub)
{
	struct pcm_runtime *rt = snd_pcm_substream_chip(alsa_sub);
	struct device *device = &rt->chip->dev->dev;

	if (alsa_sub->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		return &rt->playback;
	} else {
		return &rt->capture;
	}

	dev_err(device, "Error getting pcm substream slot.\n");
	return NULL;
}

/* call with stream_mutex locked */
void exbox_pcm_stream_stop(struct pcm_runtime *rt)
{
	int i;
	int time;
	struct usb_device *device = rt->chip->dev;

	if (rt->stream_state != STREAM_DISABLED) {
		rt->stream_state = STREAM_STOPPING;

		for (i = 0; i < PCM_N_URBS; i++) {
			time = usb_wait_anchor_empty_timeout(
					&rt->out_urbs[i].submitted, 100);
			if (!time)
				usb_kill_anchored_urbs(
					&rt->out_urbs[i].submitted);
			time = usb_wait_anchor_empty_timeout(
					&rt->in_urbs[i].submitted, 100);
			if (!time)
				usb_kill_anchored_urbs(
					&rt->in_urbs[i].submitted);
			usb_kill_urb(&rt->out_urbs[i].instance);
			usb_kill_urb(&rt->in_urbs[i].instance);
		}

		rt->stream_state = STREAM_DISABLED;
		usb_clear_halt(device, usb_sndbulkpipe(device, EP_OUT));
		usb_clear_halt(device, usb_rcvbulkpipe(device, EP_IN));
		/* Clear any stale endpoints before starting the new stream */
		dev_err(&device->dev, "USB PCM endpoints cleared in %s\n",
				__func__);
	}
}

/* call with stream_mutex locked */
int exbox_pcm_stream_start(struct pcm_runtime *rt)
{
	int ret = 0;
	int i;

	if (rt->stream_state == STREAM_DISABLED) {

		/* reset panic state when starting a new stream */
		rt->panic = false;

		/* submit our out urbs zero init */
		rt->stream_state = STREAM_STARTING;
		for (i = 0; i < PCM_N_URBS; i++) {
			memset(rt->out_urbs[i].buffer, 0, PCM_PACKET_SIZE);
			memset(rt->in_urbs[i].buffer, 0, PCM_PACKET_SIZE);
			/* Indiciate no midi by sending 0xfd */
			rt->out_urbs[i].buffer[480] = 0xfd;
			rt->out_urbs[i].buffer[481] = 0xff;
			usb_anchor_urb(&rt->out_urbs[i].instance,
				       &rt->out_urbs[i].submitted);
			usb_anchor_urb(&rt->in_urbs[i].instance,
				       &rt->in_urbs[i].submitted);
			ret = usb_submit_urb(&rt->out_urbs[i].instance,
					     GFP_ATOMIC);
			if (ret) {
				exbox_pcm_stream_stop(rt);
				return ret;
			}

			ret = usb_submit_urb(&rt->in_urbs[i].instance,
					     GFP_ATOMIC);
			if (ret) {
				exbox_pcm_stream_stop(rt);
				return ret;
			}
		}

		/* wait for first out urb to return (sent in in urb handler) */
		wait_event_timeout(rt->stream_wait_queue, rt->stream_wait_cond,
				   HZ);
		if (rt->stream_wait_cond) {
			struct device *device = &rt->chip->dev->dev;
			dev_dbg(device, "%s: PCM stream is running wakeup event\n",
				 __func__);
			rt->stream_state = STREAM_RUNNING;
		} else {
			exbox_pcm_stream_stop(rt);
			return -EIO;
		}
	}
	return ret;
}

/* call with substream locked */
/* returns true if a period elapsed */
static bool exbox_pcm_playback(struct pcm_substream *sub, struct pcm_urb *urb)
{
	struct snd_pcm_runtime *alsa_rt = sub->instance->runtime;
	struct device *device = &urb->chip->dev->dev;
	u8 *source;
	unsigned int pcm_buffer_size;

	pcm_buffer_size = snd_pcm_lib_buffer_bytes(sub->instance);

	if (sub->dma_off + PCM_BYTES_PLAYBACK <= pcm_buffer_size) {
		dev_dbg(device, "%s: (1) buffer_size %#x dma_offset %#x\n", __func__,
			 (unsigned int) pcm_buffer_size,
			 (unsigned int) sub->dma_off);

		source = alsa_rt->dma_area + sub->dma_off;

		memcpy(urb->buffer, source, PCM_BYTES_PLAYBACK);
	} else {
		/* wrap around at end of ring buffer */
		unsigned int len;

		dev_dbg(device, "%s: (2) buffer_size %#x dma_offset %#x\n", __func__,
			 (unsigned int) pcm_buffer_size,
			 (unsigned int) sub->dma_off);

		len = pcm_buffer_size - sub->dma_off;

		source = alsa_rt->dma_area + sub->dma_off;
		memcpy(urb->buffer, source, len);

		source = alsa_rt->dma_area;
		memcpy(urb->buffer + len, source,
			       PCM_BYTES_PLAYBACK - len);
	}
	sub->dma_off += PCM_BYTES_PLAYBACK;
	if (sub->dma_off >= pcm_buffer_size)
		sub->dma_off -= pcm_buffer_size;

	sub->period_off += PCM_BYTES_PLAYBACK/(3*32); /* sample size times
							 channelcount */
	dev_dbg(device, "%s: (2) period_size %i period_off %i\n", __func__,
			(unsigned int) alsa_rt->period_size,
			(unsigned int) sub->period_off);
	if (sub->period_off >= alsa_rt->period_size) {
		sub->period_off %= alsa_rt->period_size;
		return true;
	}
	return false;
}

/* call with substream locked */
static bool exbox_pcm_capture(struct pcm_substream *sub, struct pcm_urb *urb)
{
	struct snd_pcm_runtime *alsa_rt = sub->instance->runtime;
	struct device *device = &urb->chip->dev->dev;
	u8 *dst;
	unsigned int pcm_buffer_size;

	pcm_buffer_size = snd_pcm_lib_buffer_bytes(sub->instance);

	if (sub->dma_off + PCM_PACKET_SIZE <= pcm_buffer_size) {
		dev_dbg(device, "%s: (1) buffer_size %#x dma_offset %#x\n", __func__,
			 (unsigned int) pcm_buffer_size,
			 (unsigned int) sub->dma_off);

		dst = alsa_rt->dma_area + sub->dma_off;

		memcpy(dst, urb->buffer, PCM_PACKET_SIZE);
	} else {
		/* wrap around at end of ring buffer */
		unsigned int len;

		dev_dbg(device, "%s: (2) buffer_size %#x dma_offset %#x\n", __func__,
			 (unsigned int) pcm_buffer_size,
			 (unsigned int) sub->dma_off);

		len = pcm_buffer_size - sub->dma_off;

		dst = alsa_rt->dma_area + sub->dma_off;
		memcpy(dst, urb->buffer, len);

		dst = alsa_rt->dma_area;
		memcpy(dst, urb->buffer + len,
			       PCM_PACKET_SIZE - len);
	}

	sub->dma_off += PCM_PACKET_SIZE;
	if (sub->dma_off >= pcm_buffer_size)
		sub->dma_off -= pcm_buffer_size;

	sub->period_off += PCM_PACKET_SIZE/(4*32); /* bytes per sample *
						      channels */
	return false;
}

static void exbox_pcm_out_urb_handler(struct urb *usb_urb)
{
	struct pcm_urb *out_urb = usb_urb->context;
	struct pcm_runtime *rt = out_urb->chip->pcm;
	struct pcm_substream *sub;
	bool do_period_elapsed = false;
	unsigned long flags;
	int ret;
	static unsigned int midi_counter = 0;
	u8 midi_byte = 0xfd; /* 0xfd == placeholder for NO MIDI */

	out_urb->chip->out_count++;

	if (rt->panic || rt->stream_state == STREAM_STOPPING)
		return;

	if (unlikely(usb_urb->status == -ENOENT ||	/* unlinked */
		     usb_urb->status == -ENODEV ||	/* device removed */
		     usb_urb->status == -ECONNRESET ||	/* unlinked */
		     usb_urb->status == -ESHUTDOWN)) {	/* device disabled */
		goto out_fail;
	}

	if (rt->stream_state == STREAM_STARTING) {
		rt->stream_wait_cond = true;
		wake_up(&rt->stream_wait_queue);
	}

	/* now send our playback data (if a free out urb was found) */
	sub = &rt->playback;
	spin_lock_irqsave(&sub->lock, flags);
	if (sub->active)
		do_period_elapsed = exbox_pcm_playback(sub, out_urb);
	else {
		memset(out_urb->buffer, 0, PCM_PACKET_SIZE);
	}

	spin_unlock_irqrestore(&sub->lock, flags);

	if (do_period_elapsed)
		snd_pcm_period_elapsed(sub->instance);

	/* Mux in the MIDI byte */
	/* FIXME: Implement rate limits for rates above 48kHz*/
	if (midi_counter % 4 == 0 && !kfifo_is_empty(&out_urb->chip->midi2pcm)) {
		if (!kfifo_get(&out_urb->chip->midi2pcm, &midi_byte)) {
			dev_err(&out_urb->chip->dev->dev, "%s: cannot read MIDI FIFO", __func__);
		}
		dev_err(&out_urb->chip->dev->dev, "%s: setting MIDI byte to %x (FIFO at %i, counter at %u)", __func__,
				midi_byte, kfifo_avail(&out_urb->chip->midi2pcm), midi_counter);
	}
	midi_counter++;
	out_urb->buffer[480] = midi_byte; /* Either real data or placeholder */
	out_urb->buffer[481] = 0xff; /* MIDI status */

	ret = usb_submit_urb(&out_urb->instance, GFP_ATOMIC);
	if (ret < 0)
		goto out_fail;

	return;

out_fail:
	rt->panic = true;
}

static void exbox_pcm_in_urb_handler(struct urb *usb_urb)
{
	struct pcm_urb *in_urb = usb_urb->context;
	struct pcm_runtime *rt = in_urb->chip->pcm;
	struct pcm_substream *sub;
	unsigned long flags;
	int ret;

	in_urb->chip->in_count++;


	if (rt->panic || rt->stream_state == STREAM_STOPPING)
		return;

	if (unlikely(usb_urb->status == -ENOENT ||	/* unlinked */
		     usb_urb->status == -ENODEV ||	/* device removed */
		     usb_urb->status == -ECONNRESET ||	/* unlinked */
		     usb_urb->status == -ESHUTDOWN)) {	/* device disabled */
		goto out_fail;
	}

	if (rt->stream_state == STREAM_STARTING) {
		rt->stream_wait_cond = true;
		wake_up(&rt->stream_wait_queue);
	}


	/* receive our capture data */
	sub = &rt->capture;
	spin_lock_irqsave(&sub->lock, flags);
	if (sub->active) {
		exbox_pcm_capture(sub, in_urb);
		if (sub->period_off >= sub->instance->runtime->period_size) {
			sub->period_off %= sub->instance->runtime->period_size;
			spin_unlock_irqrestore(&sub->lock, flags);
			snd_pcm_period_elapsed(sub->instance);
		} else
			spin_unlock_irqrestore(&sub->lock, flags);
	} else {
		spin_unlock_irqrestore(&sub->lock, flags);
	}



	ret = usb_submit_urb(&in_urb->instance, GFP_ATOMIC);
	if (ret < 0)
		goto out_fail;


	return;

out_fail:
	rt->panic = true;
}

static int exbox_pcm_open(struct snd_pcm_substream *alsa_sub)
{
	struct pcm_runtime *rt = snd_pcm_substream_chip(alsa_sub);
	struct pcm_substream *sub = NULL;
	struct snd_pcm_runtime *alsa_rt = alsa_sub->runtime;
	int ret;

	if (rt->panic)
		return -EPIPE;

	mutex_lock(&rt->stream_mutex);
	alsa_rt->hw = pcm_hw;

	if (alsa_sub->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		sub = &rt->playback;
		/* The device only does S24_3LE for playback. */
		alsa_rt->hw.formats = SNDRV_PCM_FMTBIT_S24_3LE;
	} else {
		sub = &rt->capture;
	}

	if (!sub) {
		struct device *device = &rt->chip->dev->dev;
		mutex_unlock(&rt->stream_mutex);
		dev_err(device, "Invalid stream type\n");
		return -EINVAL;
	}

	if (rt->extra_freq) {
		alsa_rt->hw.rates |= SNDRV_PCM_RATE_KNOT;
		alsa_rt->hw.rate_max = 384000;

		/* explicit constraints needed as we added SNDRV_PCM_RATE_KNOT */
		ret = snd_pcm_hw_constraint_list(alsa_sub->runtime, 0,
						 SNDRV_PCM_HW_PARAM_RATE,
						 &constraints_extra_rates);
		if (ret < 0) {
			mutex_unlock(&rt->stream_mutex);
			return ret;
		}
	}

	sub->instance = alsa_sub;
	sub->active = false;
	mutex_unlock(&rt->stream_mutex);
	return 0;
}

static int exbox_pcm_close(struct snd_pcm_substream *alsa_sub)
{
	struct pcm_runtime *rt = snd_pcm_substream_chip(alsa_sub);
	struct pcm_substream *sub = exbox_pcm_get_substream(alsa_sub);
	unsigned long flags;

	if (rt->panic)
		return 0;

	mutex_lock(&rt->stream_mutex);
	if (sub) {

		/* deactivate substream */
		spin_lock_irqsave(&sub->lock, flags);
		sub->instance = NULL;
		sub->active = false;
		spin_unlock_irqrestore(&sub->lock, flags);

		/* all substreams closed? if so, stop streaming */
		if (!rt->playback.instance && !rt->capture.instance) {
			/* FIXME: Keep running for now until we know what we
			 * really need.
			 */
#if 0
			exbox_pcm_stream_stop(rt);
#endif
		}

	}
	mutex_unlock(&rt->stream_mutex);
	return 0;
}

static int exbox_pcm_hw_params(struct snd_pcm_substream *alsa_sub,
				struct snd_pcm_hw_params *hw_params)
{
	return snd_pcm_lib_alloc_vmalloc_buffer(alsa_sub,
						params_buffer_bytes(hw_params));
}

static int exbox_pcm_hw_free(struct snd_pcm_substream *alsa_sub)
{
	return snd_pcm_lib_free_vmalloc_buffer(alsa_sub);
}

static int exbox_pcm_prepare(struct snd_pcm_substream *alsa_sub)
{
	struct pcm_runtime *rt = snd_pcm_substream_chip(alsa_sub);
	struct pcm_substream *sub = exbox_pcm_get_substream(alsa_sub);
	struct snd_pcm_runtime *alsa_rt = alsa_sub->runtime;
        struct usb_device *device = rt->chip->dev;
	int ret;

	if (rt->panic)
		return -EPIPE;
	if (!sub)
		return -ENODEV;

        if(!rt->chip->device_streaming) {
                return -EPIPE;
        }

	mutex_lock(&rt->stream_mutex);

	exbox_pcm_stream_stop(rt);

	sub->dma_off = 0;
	sub->period_off = 0;

	if (rt->stream_state == STREAM_DISABLED) {
		ret = exbox_pcm_set_rate(rt, alsa_rt->rate);
		if (ret) {
			mutex_unlock(&rt->stream_mutex);
			dev_err(&device->dev, "Error setting sample rate, but still continuing.\n");
			//return ret;
		}
		ret = exbox_pcm_stream_start(rt);
		if (ret) {
			mutex_unlock(&rt->stream_mutex);
			dev_err(&device->dev, "Error starting streams, aborting.\n");
			return ret;
		}
	}
	mutex_unlock(&rt->stream_mutex);
	return 0;
}

static int exbox_pcm_trigger(struct snd_pcm_substream *alsa_sub, int cmd)
{
	struct pcm_substream *sub = exbox_pcm_get_substream(alsa_sub);
	struct pcm_runtime *rt = snd_pcm_substream_chip(alsa_sub);


	if (rt->panic)
		return -EPIPE;
	if (!sub)
		return -ENODEV;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		spin_lock_irq(&sub->lock);
		sub->active = true;
		spin_unlock_irq(&sub->lock);
		return 0;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		spin_lock_irq(&sub->lock);
		sub->active = false;
		spin_unlock_irq(&sub->lock);
		return 0;

	default:
		return -EINVAL;
	}
}

static snd_pcm_uframes_t exbox_pcm_pointer(struct snd_pcm_substream *alsa_sub)
{
	struct pcm_substream *sub = exbox_pcm_get_substream(alsa_sub);
	struct pcm_runtime *rt = snd_pcm_substream_chip(alsa_sub);
	unsigned long flags;
	snd_pcm_uframes_t dma_offset;

	if (rt->panic || !sub)
		return SNDRV_PCM_POS_XRUN;

	spin_lock_irqsave(&sub->lock, flags);
	dma_offset = sub->dma_off;
	spin_unlock_irqrestore(&sub->lock, flags);
	return bytes_to_frames(alsa_sub->runtime, dma_offset);
}

static const struct snd_pcm_ops pcm_ops = {
	.open = exbox_pcm_open,
	.close = exbox_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = exbox_pcm_hw_params,
	.hw_free = exbox_pcm_hw_free,
	.prepare = exbox_pcm_prepare,
	.trigger = exbox_pcm_trigger,
	.pointer = exbox_pcm_pointer,
	.page = snd_pcm_lib_get_vmalloc_page,
};

static int exbox_pcm_init_urb(struct pcm_urb *urb,
			       struct exbox_chip *chip,
			       bool is_input,
			       unsigned int ep,
			       void (*handler)(struct urb *))
{
	urb->chip = chip;
	usb_init_urb(&urb->instance);

	urb->buffer = kzalloc(PCM_PACKET_SIZE, GFP_KERNEL);
	if (!urb->buffer)
		return -ENOMEM;

	usb_fill_bulk_urb(&urb->instance, chip->dev,
			  (is_input ? usb_rcvbulkpipe(chip->dev, ep) : usb_sndbulkpipe(chip->dev, ep)), (void *)urb->buffer,
			  PCM_PACKET_SIZE, handler, urb);
	if (usb_urb_ep_type_check(&urb->instance))
		return -EINVAL;
	init_usb_anchor(&urb->submitted);

	return 0;
}

void exbox_pcm_abort(struct exbox_chip *chip)
{
	struct pcm_runtime *rt = chip->pcm;
	unsigned long flags;

	if (rt) {
		rt->panic = true;
		if (rt->playback.instance) {
			snd_pcm_stream_lock_irqsave(rt->playback.instance, flags);
			snd_pcm_stop(rt->playback.instance,
					SNDRV_PCM_STATE_XRUN);
			snd_pcm_stream_unlock_irqrestore(rt->playback.instance, flags);
		}

		if (rt->capture.instance) {
			snd_pcm_stream_lock_irqsave(rt->capture.instance, flags);
			snd_pcm_stop(rt->capture.instance,
					SNDRV_PCM_STATE_XRUN);
			snd_pcm_stream_unlock_irqrestore(rt->capture.instance, flags);
		}

		mutex_lock(&rt->stream_mutex);
		exbox_pcm_stream_stop(rt);
		mutex_unlock(&rt->stream_mutex);
	}
}

void exbox_pcm_destroy(struct exbox_chip *chip)
{
	struct pcm_runtime *rt = chip->pcm;
	int i;

	if (!rt) {
		goto out;
	}

	for (i = 0; i < PCM_N_URBS; i++) {
		kfree(rt->out_urbs[i].buffer);
		kfree(rt->in_urbs[i].buffer);
	}

out:
	kfree(chip->pcm);
	chip->pcm = NULL;
}

static void exbox_pcm_free(struct snd_pcm *pcm)
{
	struct pcm_runtime *rt = pcm->private_data;

	if (rt)
		exbox_pcm_destroy(rt->chip);
}

int exbox_pcm_init(struct exbox_chip *chip, u8 extra_freq)
{
	int i;
	int ret;
	struct snd_pcm *pcm;
	struct pcm_runtime *rt;

	rt = kzalloc(sizeof(*rt), GFP_KERNEL);
	if (!rt)
		return -ENOMEM;

	rt->chip = chip;
	rt->stream_state = STREAM_DISABLED;
	if (extra_freq)
		rt->extra_freq = 1;

	init_waitqueue_head(&rt->stream_wait_queue);
	mutex_init(&rt->stream_mutex);
	spin_lock_init(&rt->playback.lock);
	spin_lock_init(&rt->capture.lock);
	rt->playback.active = false;
	rt->capture.active = false;

	for (i = 0; i < PCM_N_URBS; i++) {
		ret = exbox_pcm_init_urb(&rt->in_urbs[i], chip, true, EP_IN,
				    exbox_pcm_in_urb_handler);
		if (ret < 0) {
			kfree(rt);
			dev_err(&chip->dev->dev, "Cannot initialise input URBs\n");
			return ret;
}
		ret = exbox_pcm_init_urb(&rt->out_urbs[i], chip, false, EP_OUT,
				    exbox_pcm_out_urb_handler);
		if (ret < 0) {
			kfree(rt);
			dev_err(&chip->dev->dev, "Cannot initialise output URBs\n");
			return ret;
		}

		rt->in_urbs[i].peer = &rt->out_urbs[i];
		rt->out_urbs[i].peer = &rt->in_urbs[i];
	}

	ret = snd_pcm_new(chip->card, "D.O.tec UMA", 0, 1, 1, &pcm);
	if (ret < 0) {
		kfree(rt);
		dev_err(&chip->dev->dev, "Cannot create pcm instance\n");
		return ret;
	}

	pcm->private_data = rt;
	pcm->private_free = exbox_pcm_free;

	strlcpy(pcm->name, "D.O.tec UMA", sizeof(pcm->name));
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &pcm_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &pcm_ops);

	rt->instance = pcm;

	chip->pcm = rt;
	return 0;
}
