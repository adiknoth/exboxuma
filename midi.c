/*
 * Linux driver for D.O.Tec EXBOX.UMA (PloyTec USB interface)
 *
 * Rawmidi driver
 *
 * Copyright 2015 (C) Google Inc.
 *
 * Authors:  Adrian Knoth <adi@drcomp.erfurt.thur.de>
 *           Adrian Knoth <aknoth@google.com>
 *
 * The driver is based on the 6fire driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/slab.h>
#include <sound/rawmidi.h>

#include "midi.h"
#include "chip.h"

enum {
	MIDI_BUFSIZE = 512
};

static int exbox_midi_init_urb(struct midi_urb *urb,
			       struct exbox_chip *chip,
			       unsigned int ep,
			       void (*handler)(struct urb *))
{
	urb->chip = chip;
	usb_init_urb(&urb->instance);

	urb->buffer = kzalloc(MIDI_BUFSIZE, GFP_KERNEL);
	if (!urb->buffer)
		return -ENOMEM;

	usb_fill_bulk_urb(&urb->instance, chip->dev,
			  usb_rcvbulkpipe(chip->dev, ep),
			  (void *)urb->buffer,
			  MIDI_BUFSIZE, handler, urb);
	init_usb_anchor(&urb->submitted);

	return 0;
}

static void exbox_midi_in_handler(struct urb *usb_urb)
{
	struct midi_urb *in_urb = usb_urb->context;
	struct midi_runtime *rt = in_urb->chip->midi;
	struct exbox_chip *chip = in_urb->chip;

	int i;
	int packet_size = 4; /* 4xMIDI, 1xStatus (NOT MIDI-status) */
	unsigned long flags;
	int ret;

	chip->midi_in_count++;

	if (rt->panic || rt->stream_state == STREAM_STOPPING)
		return;

	if (unlikely(usb_urb->status == -ENOENT ||	/* unlinked */
		     usb_urb->status == -ENODEV ||	/* device removed */
		     usb_urb->status == -ECONNRESET ||	/* unlinked */
		     usb_urb->status == -ESHUTDOWN)) {	/* device disabled */
		goto out_fail;
	}


	spin_lock_irqsave(&rt->in_lock, flags);

	if (!rt->in) {
		goto out_ok;
	}

	if (chip->samplerate > 48000) {
		packet_size = 1; /* 1xMIDI, 1xStatus for rates above 48kHz */
	}

	for (i=0; i < packet_size; i++) {
		/* The device uses 0xfd to indicate "no data" */
		if (in_urb->buffer[i] != 0xfd && in_urb->buffer[i] != 0xff) {
			snd_rawmidi_receive(rt->in, &in_urb->buffer[i], 1);
			dev_dbg(&in_urb->chip->dev->dev, "%s: receiving MIDI byte %x", __func__, in_urb->buffer[i]);
		}

	}


out_ok:
	spin_unlock_irqrestore(&rt->in_lock, flags);
	ret = usb_submit_urb(&in_urb->instance, GFP_ATOMIC);
	if (ret < 0)
		goto out_fail;


	return;

out_fail:
	rt->panic = true;
	return;
}

static int exbox_midi_out_open(struct snd_rawmidi_substream *alsa_sub)
{
	return 0;
}

static int exbox_midi_out_close(struct snd_rawmidi_substream *alsa_sub)
{
	return 0;
}

static void exbox_midi_out_trigger(
		struct snd_rawmidi_substream *alsa_sub, int up)
{
	struct midi_runtime *rt = alsa_sub->rmidi->private_data;
	int ret;
	unsigned long flags;
	int size;

	spin_lock_irqsave(&rt->out_lock, flags);
	if (up) { /* start transfer */
		if (rt->out) { /* we are already transmitting so just return */
			goto out;
		}

		size = kfifo_avail(&rt->chip->midi2pcm);
		if (size <= 0) {
			dev_err(&rt->chip->dev->dev, "%s kfifo_avail negative/zero", __func__);
			goto out;
		}

		ret = snd_rawmidi_transmit(alsa_sub, rt->out_buffer, size);
		dev_err(&rt->chip->dev->dev, "%s rawmidi: %i/%i (buf)", __func__, ret, size);
		rt->out = alsa_sub;
		if (ret > 0) {
			/* Copy 'ret' elements from the out_buffer to the
			 * fifo.
			 */
			kfifo_in(&rt->chip->midi2pcm, rt->out_buffer, ret);
		}
	} else {
		rt->out = NULL;
	}

out:
	spin_unlock_irqrestore(&rt->out_lock, flags);
	return;
}

static void exbox_midi_out_drain(struct snd_rawmidi_substream *alsa_sub)
{
	struct midi_runtime *rt = alsa_sub->rmidi->private_data;
	int retry = 0;

	while (rt->out && retry++ < 100)
		msleep(10);
}

/* call with stream_mutex locked */
int exbox_midi_stream_start(struct midi_runtime *rt)
{
	int ret = 0;
	int i;
	struct usb_device *device = rt->chip->dev;

	if (rt->stream_state == STREAM_DISABLED) {
		/* reset panic state when starting a new stream */
		rt->panic = false;
		dev_dbg(&device->dev, "MIDI starting up: %s\n",
				__func__);
		/* submit our out urbs zero init */
		rt->stream_state = STREAM_STARTING;
		for (i = 0; i < MIDI_N_URBS; i++) {
			/* Indiciate no midi by sending 0xfd */
			memset(rt->in_urbs[i].buffer, 0xfd, MIDI_BUFSIZE);
			usb_anchor_urb(&rt->in_urbs[i].instance,
				       &rt->in_urbs[i].submitted);
			ret = usb_submit_urb(&rt->in_urbs[i].instance,
					     GFP_ATOMIC);
			if (ret) {
				dev_dbg(&device->dev, "%s: maybe something is wrong\n",
				 __func__);
				return ret;
			}
		}

		rt->stream_state = STREAM_RUNNING;
		dev_dbg(&device->dev, "%s: MIDI URB sent, should be good now\n",
				 __func__);
		return 0;
	}
	return ret;
}

/* call with stream_mutex locked */
void exbox_midi_stream_stop(struct midi_runtime *rt)
{
	int i;
	int time;
	struct usb_device *device = rt->chip->dev;

	if (rt->stream_state != STREAM_DISABLED) {
		rt->stream_state = STREAM_STOPPING;

		for (i = 0; i < MIDI_N_URBS; i++) {
			time = usb_wait_anchor_empty_timeout(
					&rt->in_urbs[i].submitted, 100);
			if (!time)
				usb_kill_anchored_urbs(
					&rt->in_urbs[i].submitted);
			usb_kill_urb(&rt->in_urbs[i].instance);
		}

		rt->stream_state = STREAM_DISABLED;
		dev_dbg(&device->dev, "USB MIDI stopped in %s\n",
				__func__);
	}
}

static int exbox_midi_in_open(struct snd_rawmidi_substream *alsa_sub)
{
	/* FIXME: Check if the device is already streaming, and if not, start
	 * the streams. Right now, streaming state is manually triggered from
	 * userspace.
	 */
	return 0;
}

static int exbox_midi_in_close(struct snd_rawmidi_substream *alsa_sub)
{
	return 0;
}

static void exbox_midi_in_trigger(
		struct snd_rawmidi_substream *alsa_sub, int up)
{
	struct midi_runtime *rt = alsa_sub->rmidi->private_data;
	unsigned long flags;

	spin_lock_irqsave(&rt->in_lock, flags);
	if (up)
		rt->in = alsa_sub;
	else
		rt->in = NULL;
	spin_unlock_irqrestore(&rt->in_lock, flags);
}

static struct snd_rawmidi_ops out_ops = {
	.open = exbox_midi_out_open,
	.close = exbox_midi_out_close,
	.trigger = exbox_midi_out_trigger,
	/* .drain = exbox_midi_out_drain */
};

static struct snd_rawmidi_ops in_ops = {
	.open = exbox_midi_in_open,
	.close = exbox_midi_in_close,
	.trigger = exbox_midi_in_trigger
};

int exbox_midi_init(struct exbox_chip *chip)
{
	int ret;
	int i;
	struct midi_runtime *rt = kzalloc(sizeof(struct midi_runtime),
			GFP_KERNEL);
	//struct comm_runtime *comm_rt = chip->comm;

	if (!rt)
		return -ENOMEM;

	rt->out_buffer = kzalloc(MIDI2PCM_MUX_FIFO_SIZE, GFP_KERNEL);
	if (!rt->out_buffer) {
		kfree(rt);
		return -ENOMEM;
	}

	rt->chip = chip;
	rt->stream_state = STREAM_DISABLED;
	init_waitqueue_head(&rt->stream_wait_queue);
	mutex_init(&rt->stream_mutex);
	spin_lock_init(&rt->in_lock);
	spin_lock_init(&rt->out_lock);

	for (i = 0; i < MIDI_N_URBS; i++) {
		exbox_midi_init_urb(&rt->in_urbs[i], chip, EP_MIDI_IN,
				    exbox_midi_in_handler);
	}


	ret = snd_rawmidi_new(chip->card, "EXBOXUMA", 0, 1, 1, &rt->instance);
	if (ret < 0) {
		kfree(rt->out_buffer);
		kfree(rt);
		dev_err(&chip->dev->dev, "unable to create midi.\n");
		return ret;
	}
	rt->instance->private_data = rt;
	strcpy(rt->instance->name, "D.O.Tec EXBOX UMA MIDI");
	rt->instance->info_flags = SNDRV_RAWMIDI_INFO_OUTPUT |
			SNDRV_RAWMIDI_INFO_INPUT |
			SNDRV_RAWMIDI_INFO_DUPLEX;
	snd_rawmidi_set_ops(rt->instance, SNDRV_RAWMIDI_STREAM_OUTPUT,
			&out_ops);
	snd_rawmidi_set_ops(rt->instance, SNDRV_RAWMIDI_STREAM_INPUT,
			&in_ops);

	chip->midi = rt;
	return 0;
}

void exbox_midi_abort(struct exbox_chip *chip)
{
	struct midi_runtime *rt = chip->midi;
	int i;

	if (!rt) {
		return;
	}

	usb_poison_urb(&rt->out_urb);
	for (i = 0; i < MIDI_N_URBS; i++) {
		usb_poison_urb(&rt->in_urbs[i].instance);
	}
}

void exbox_midi_destroy(struct exbox_chip *chip)
{
	struct midi_runtime *rt = chip->midi;
	int i;

	for (i = 0; i < MIDI_N_URBS; i++) {
		kfree(rt->in_urbs[i].buffer);
	}

	kfree(rt->out_buffer);
	kfree(rt);
	chip->midi = NULL;
}
