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

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <sound/info.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <linux/usb/audio.h>


#include "chip.h"
#include "pcm.h"
#include "midi.h"

MODULE_AUTHOR("Adrian Knoth <adi@drcomp.erfurt.thur.de>");
MODULE_DESCRIPTION("DirectOut EXBOX.UMA USB audio driver");
MODULE_LICENSE("GPL v2");
MODULE_SUPPORTED_DEVICE("{{directOut,EXBOX.UMA},"
			 "{directOut,EXBOX.XT}}");

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX; /* Index 0-max */
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR; /* Id for card */
static bool enable[SNDRV_CARDS] = SNDRV_DEFAULT_ENABLE_PNP; /* Enable this card */
static struct exbox_chip *chips[SNDRV_CARDS] = SNDRV_DEFAULT_PTR;
static struct usb_device *devices[SNDRV_CARDS] = SNDRV_DEFAULT_PTR;

static struct usb_driver exbox_usb_driver;


#define DRIVER_NAME "snd-usb-exbox"
#define CARD_NAME "EXBOX"

module_param_array(index, int, NULL, 0444);
MODULE_PARM_DESC(index, "Index value for " CARD_NAME " soundcard.");
module_param_array(id, charp, NULL, 0444);
MODULE_PARM_DESC(id, "ID string for " CARD_NAME " soundcard.");
module_param_array(enable, bool, NULL, 0444);
MODULE_PARM_DESC(enable, "Enable " CARD_NAME " soundcard.");

static DEFINE_MUTEX(register_mutex);

struct exbox_vendor_quirk {
	const char *device_name;
	u8 extra_freq;
};

static int my_control_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
          uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
          uinfo->count = 1;
          uinfo->value.integer.min = 0;
          uinfo->value.integer.max = 1;
          return 0;
}

static int my_control_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct exbox_chip *chip = snd_kcontrol_chip(kcontrol);
	//struct pcm_runtime *pcm = chip->pcm;
	u8 state = STREAM_DISABLED;

	//if (pcm)
		state = chip->device_streaming;

	ucontrol->value.integer.value[0] = (state == STREAM_RUNNING) ? 1 : 0;
	return 0;
}

static int my_control_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct exbox_chip *chip = snd_kcontrol_chip(kcontrol);
	struct pcm_runtime *pcm_rt = chip->pcm;
	struct midi_runtime *midi_rt = chip->midi;
        struct usb_device *device = pcm_rt->chip->dev;
	int ret;
	bool want_stream;

	if (!pcm_rt)
		return -ENODEV;


	want_stream = ucontrol->value.integer.value[0];

	if (!want_stream) {
		/* Stop the streams. Not sure if it really does what I think
		 * it does
		 */
		dev_err(&device->dev, "Turning streams off\n");


#if 1
		if (pcm_rt->stream_state == STREAM_RUNNING) {
			mutex_lock(&pcm_rt->stream_mutex);
			exbox_pcm_stream_stop(pcm_rt);
			mutex_unlock(&pcm_rt->stream_mutex);
		}
#endif

#if 1
		if (midi_rt->stream_state == STREAM_RUNNING) {
			mutex_lock(&midi_rt->stream_mutex);
			exbox_midi_stream_stop(midi_rt);
			mutex_unlock(&midi_rt->stream_mutex);
		} else {
			dev_err(&device->dev, "NOT stopping MIDI streams, already in state %i\n",
					midi_rt->stream_state);
		}
#endif

		snd_exbox_clear_problems(device, false);
		/* We are no longer streaming */
		chip->device_streaming = false;

		return 0;
	}

	if (want_stream) {
		/* FIXME: Do we need a mutex here? */
		if (!chip->device_streaming) {
			/* Turn on the stream */
			snd_exbox_clear_problems(device, true);
			usb_clear_halt(device, usb_sndbulkpipe(device, EP_OUT));
			usb_clear_halt(device, usb_rcvbulkpipe(device, EP_IN));
			usb_clear_halt(device, usb_rcvbulkpipe(device, EP_MIDI_IN));
			dev_err(&device->dev, "Turning streams on\n");
		}

#if 1
		if (midi_rt->stream_state == STREAM_DISABLED) {
			mutex_lock(&midi_rt->stream_mutex);
			ret = exbox_midi_stream_start(midi_rt);
			mutex_unlock(&midi_rt->stream_mutex);
			if (ret) {
				dev_err(&device->dev, "Error starting MIDI streams, aborting.\n");
				return ret;
			}
		} else {
			dev_err(&device->dev, "NOT starting MIDI streams, already in state %i\n",
					midi_rt->stream_state);
		}
#endif

#if 1
		if (pcm_rt->stream_state == STREAM_DISABLED) {
			mutex_lock(&pcm_rt->stream_mutex);
			ret = exbox_pcm_stream_start(pcm_rt);
			mutex_unlock(&pcm_rt->stream_mutex);
			if (ret) {
				dev_err(&device->dev, "Error starting PCM streams, aborting.\n");
				return ret;
			}
		}
#endif


		/* We are streaming */
		chip->device_streaming = true;

		return 0;
	}

	return 0;
}

static struct snd_kcontrol_new my_control = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Streaming enabled Switch",
	.index = 0,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
		SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	.private_value = 0xffff,
	.info = my_control_info,
	.get = my_control_get,
	.put = my_control_put
};

static int exbox_chip_create(struct usb_interface *intf,
			      struct usb_device *device, int idx,
			      const struct exbox_vendor_quirk *quirk,
			      struct exbox_chip **rchip)
{
	struct snd_card *card = NULL;
	struct exbox_chip *chip;
	int ret;
	int len;

	*rchip = NULL;

	/* if we are here, card can be registered in alsa. */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,15,0)
	ret = snd_card_new(&intf->dev, index[idx], id[idx], THIS_MODULE,
			   sizeof(*chip), &card);
#else
	ret = snd_card_create(index[idx], id[idx], THIS_MODULE,
			   sizeof(*chip), &card);
#endif

	if (ret < 0) {
		dev_err(&device->dev, "cannot create alsa card.\n");
		return ret;
	}

	strlcpy(card->driver, DRIVER_NAME, sizeof(card->driver));

	if (quirk && quirk->device_name)
		strlcpy(card->shortname, quirk->device_name, sizeof(card->shortname));
	else
		strlcpy(card->shortname, "EXBOX.UMA", sizeof(card->shortname));

	strlcat(card->longname, card->shortname, sizeof(card->longname));
	len = strlcat(card->longname, " at ", sizeof(card->longname));
	if (len < sizeof(card->longname))
		usb_make_path(device, card->longname + len,
			      sizeof(card->longname) - len);

	chip = card->private_data;
	chip->dev = device;
	chip->card = card;
	chip->regidx = idx;
	chip->intf_count = 1;

	/* Init MIDI-to-PCM mux FIFO. Since we receive MIDI from userspace in
	 * midi.c but send this data in the PCM out handler (pcm.c), we need
	 * to temporarily store the information in the FIFO.
	 *
	 */
	INIT_KFIFO(chip->midi2pcm);

	/* Create control interfaces */
	ret = snd_ctl_add(card, snd_ctl_new1(&my_control, chip));
	if (ret < 0)
		return ret;


	*rchip = chip;
	return 0;
}


/*
  Bit0: digital instead of analog input on "only 2 channel" units, doesn't apply here
X Bit1: internal mode (ignore sync from outside)
  Bit2: digitally locked (read only)
  Bit3: direct monitoring, doesn't apply here
X Bit4: 32bit input, doesn't apply here
X Bit5: 16 instead of 32 inputs, doesn't apply here
  Bit6: constant MIDI for port reading, doesn't apply here
  Bit7: unused
**/

static unsigned char snd_exbox_read_status(struct exbox_chip *chip)
{
	int rc;
	unsigned char status = 0;
	struct usb_device *dev = chip->dev;


	rc = usb_control_msg(dev,
			usb_rcvctrlpipe(dev, 0),
			0x49,
			0xc0,
			//USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_IN,
			0x0, 0,
			&status, sizeof(status), 1000);

	if (rc < 0) {
		dev_err(&dev->dev, "Error reading EXBOX status:%i\n",
				rc);
	}
	return status;
}

int snd_exbox_write_status(struct exbox_chip *chip, unsigned char status)
{
	int rc;
	struct usb_device *dev = chip->dev;

	rc = usb_control_msg(dev,
			usb_sndctrlpipe(dev, 0),
			0x49,
			0x40,
			status, 0, NULL, 0, 1000);

	if (rc < 0) {
		dev_err(&dev->dev, "Error writing status %02x\n",
				status);
	}

	return rc;
}

int snd_exbox_get_samplerate(struct exbox_chip *chip)
{
  int err;
  int rate;
  const size_t datalen = 3;
  u8 *data;
  struct usb_device *dev = chip->dev;

  data = kcalloc(sizeof(u8), datalen, GFP_KERNEL);
  if (!data)
	  return -ENOMEM;

  err = usb_control_msg(dev,
		  usb_rcvctrlpipe(dev, 0),
		  0x81,
		  0xa2,
		  0x0100,
		  EP_IN,
		  data,
		  datalen,
		  1000);

  if (err < 0) {
		dev_err(&dev->dev, "Error reading samplerate %i\n",
				err);
		rate = -1;
  } else {
	  rate = data[0] + (data[1] << 8);
  }

  kfree(data);
  return rate;

}

int snd_exbox_set_samplerate(struct exbox_chip *chip, unsigned int rate)
{
  int rc = 0;
  int readback_rate;
  const size_t datalen = 3;
  u8 *data;
  struct usb_device *dev = chip->dev;

  data = kcalloc(sizeof(u8), datalen, GFP_KERNEL);
  if (!data)
	  return -ENOMEM;

  data[0] = (rate & 0xFF);
  data[1] = (rate & 0xFF00) >> 8;
  data[2] = 0;

  data[0] = rate;
  data[1] = rate >> 8;
  data[2] = rate >> 16;


  rc = usb_control_msg(dev,
		  usb_sndctrlpipe(dev, 0),
		  UAC_SET_CUR,
		  USB_TYPE_CLASS | USB_RECIP_ENDPOINT | USB_DIR_OUT,
		  UAC_EP_CS_ATTR_SAMPLE_RATE << 8,
		  EP_IN,
		  data,
		  datalen,
		  1000);

  if (rc < 0) {
		dev_err(&dev->dev, "Error setting samplerate %i: %i\n",
			rate, rc);
		goto out;
  }

  rc = usb_control_msg(dev,
		  usb_sndctrlpipe(dev, 0),
		  0x01,
		  0x22,
		  0x0100,
		  EP_OUT,
		  data,
		  datalen,
		  1000);

  if (rc < 0) {
		dev_err(&dev->dev, "Error setting samplerate %i: %i\n",
			rate, rc);
		goto out;
  }

  readback_rate = snd_exbox_get_samplerate(chip);
  chip->samplerate = readback_rate;

  if (readback_rate != (unsigned) rate) {
		dev_err(&dev->dev, "Error setting samplerate to %iHz (device did not accept rate)\n",
			rate);
		rc = -1;
		goto out;
  }


out:
  kfree(data);
  return rc;
}

static void
snd_exbox_proc_read_debug(struct snd_info_entry *entry,
		struct snd_info_buffer *buffer)
{
	struct exbox_chip *chip = entry->private_data;
	unsigned char status = snd_exbox_read_status(chip);

	snd_iprintf(buffer, "PCM in_count: %lu\n", chip->in_count);
	snd_iprintf(buffer, "PCM out_count: %lu\n", chip->out_count);
	snd_iprintf(buffer, "MIDI in_count: %lu\n", chip->midi_in_count);
	snd_iprintf(buffer, "Device streaming status: %i\n",
			chip->device_streaming);
	snd_iprintf(buffer, "STATUS (0x%02x):\n", status);
	snd_iprintf(buffer, " Internal mode: %s\n", (status&0x02)?"on":"off");
	snd_iprintf(buffer, " Digitally locked: %s\n", (status&0x04)?"yes":"no");

	snd_iprintf(buffer, " Samplerate: %i\n",
			snd_exbox_get_samplerate(chip));

}



static void snd_exbox_proc_init(struct exbox_chip *chip)
{
	struct snd_info_entry *entry;


	/* debug file to read all hdspm registers */
	if (!snd_card_proc_new(chip->card, "exbox", &entry)) {
		snd_info_set_text_ops(entry, chip,
				snd_exbox_proc_read_debug);
	} else {
		dev_err(chip->card->dev, "Error setting up proc.\n");
	}

	return;
}

int snd_exbox_clear_problems(struct usb_device *usbdev, bool enable_streaming)
{
	int onoff = 0;

	if (enable_streaming) {
	   onoff = 1;
	}

	if (usb_set_interface(usbdev, 0, onoff) != 0) {
		dev_err(&usbdev->dev, "can't set first interface for " CARD_NAME " device.\n");
		return -EIO;
	}

        if (usb_set_interface(usbdev, 1, onoff) != 0) {
                dev_err(&usbdev->dev, "can't set second interface.\n");
                return -EIO;
        }

        return 0;
}

static void exbox_chip_destroy(struct exbox_chip *chip)
{
	if (chip) {
		if (chip->pcm)
			exbox_pcm_destroy(chip);
		if (chip->midi)
			exbox_midi_destroy(chip);
		if (chip->card)
			snd_card_free(chip->card);
	}
}

static int exbox_chip_probe(struct usb_interface *intf,
			     const struct usb_device_id *usb_id)
{
	const struct exbox_vendor_quirk *quirk = (struct exbox_vendor_quirk *)usb_id->driver_info;
	int ret;
	int i;
	struct exbox_chip *chip;
	struct usb_device *device = interface_to_usbdev(intf);
        int regidx = -1; /* index in module parameter array */

#if 1
	ret = usb_set_interface(device, 0, 1);
	if (ret != 0) {
		dev_err(&device->dev, "can't set first interface for " CARD_NAME " device.\n");
		return -EIO;
	}

        if (usb_set_interface(device, 1, 1) != 0) {
                dev_err(&device->dev, "can't set second interface.\n");
                return -EIO;
        }
#else
        ret = snd_exbox_clear_problems(device, false);
	if (ret != 0) {
		return ret;
	}
#endif

	/* check whether the card is already registered */
	chip = NULL;
	mutex_lock(&register_mutex);

        for (i = 0; i < SNDRV_CARDS; i++) {
                if (devices[i] == device) {
                        if (chips[i])
                                chips[i]->intf_count++;
                        usb_set_intfdata(intf, chips[i]);
#if 0
			usb_driver_claim_interface(&exbox_usb_driver,
					intf, chips[i]);
#endif
                        mutex_unlock(&register_mutex);
                        return 0;
                } else if (!devices[i] && regidx < 0)
                        regidx = i;
        }
        if (regidx < 0) {
                mutex_unlock(&register_mutex);
                dev_err(&intf->dev, "too many cards registered.\n");
                return -ENODEV;
        }
        devices[regidx] = device;

	ret = exbox_chip_create(intf, device, regidx, quirk, &chip);
	if (ret < 0)
		goto err;

	chips[regidx] = chip;

#if 0
	usb_driver_claim_interface(&exbox_usb_driver,
			intf, chips[regidx]);
#endif

	ret = exbox_pcm_init(chip, quirk ? quirk->extra_freq : 0);
	if (ret < 0)
		goto err_chip_destroy;

	ret = exbox_midi_init(chip);
	if (ret < 0)
		goto err_chip_destroy;

	dev_info(&device->dev, "proc init...\n");
	snd_exbox_proc_init(chip);

	ret = snd_card_register(chip->card);
	if (ret < 0) {
		dev_err(&device->dev, "cannot register " CARD_NAME " card\n");
		goto err_chip_destroy;
	}

	usb_set_intfdata(intf, chip);

	/* XXX: Maybe set to 0x32. */
	snd_exbox_write_status(chip, 0x36 | (1<<6));
	/* This will most likely fail, but that's no problem.*/
	snd_exbox_set_samplerate(chip, 48000);

	usb_clear_halt(device, usb_rcvbulkpipe(device, EP_IN));
	usb_clear_halt(device, usb_rcvbulkpipe(device, EP_MIDI_IN));
	usb_clear_halt(device, usb_sndbulkpipe(device, EP_OUT));
	chip->device_streaming = false;

	mutex_unlock(&register_mutex);

	return 0;

err_chip_destroy:
	exbox_chip_destroy(chip);
err:
	mutex_unlock(&register_mutex);
	return ret;
}

static void exbox_chip_disconnect(struct usb_interface *intf)
{
	struct exbox_chip *chip;
	struct snd_card *card;

	chip = usb_get_intfdata(intf);
	if (!chip)
		return;

	card = chip->card;
	chip->intf_count--;
	if (!chip->intf_count) {
		mutex_lock(&register_mutex);
		devices[chip->regidx] = NULL;
		chips[chip->regidx] = NULL;
		mutex_unlock(&register_mutex);

		//chip->shutdown = true;

		/* Make sure that the userspace cannot create new request */
		snd_card_disconnect(card);

		exbox_midi_abort(chip);
		exbox_pcm_abort(chip);
		exbox_chip_destroy(chip);
	}

}

static const struct usb_device_id device_table[] = {
	/*XXX: hier USB-IDs eintragen; .driver_info kann NULL sein */
	{
		USB_DEVICE(0x0a4a, 0xd064),
		.driver_info = (unsigned long)&(const struct exbox_vendor_quirk) {
			.device_name = "EXBOX.UMA",
			.extra_freq = 1,
		}
	},
	{}
};

MODULE_DEVICE_TABLE(usb, device_table);

static struct usb_driver exbox_usb_driver = {
	.name = DRIVER_NAME,
	.probe = exbox_chip_probe,
	.disconnect = exbox_chip_disconnect,
	.id_table = device_table,
};

module_usb_driver(exbox_usb_driver);
