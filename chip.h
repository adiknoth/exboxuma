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

#ifndef EXBOX_CHIP_H
#define EXBOX_CHIP_H

#include <linux/usb.h>
#include <linux/kfifo.h>
#include <sound/core.h>

/* MIDI2PCM_MUX_FIFO_SIZE defines the size of the kfifo. Must be power-of-two
 */
#define MIDI2PCM_MUX_FIFO_SIZE 64

struct pcm_runtime;
struct midi_runtime;

struct exbox_chip {
	struct usb_device *dev;
	struct snd_card *card;
	struct pcm_runtime *pcm;
	struct midi_runtime *midi;
	DECLARE_KFIFO(midi2pcm, u8, MIDI2PCM_MUX_FIFO_SIZE);
	int intf_count;
	int regidx;
	unsigned long in_count;
	unsigned long out_count;
	unsigned long midi_in_count;
	int samplerate;
	bool device_streaming;
};

#define EP_OUT          0x5
#define EP_IN           0x6
#define EP_MIDI_IN      0x3

int snd_exbox_set_samplerate(struct exbox_chip *chip, unsigned int rate);
int snd_exbox_write_status(struct exbox_chip *chip, unsigned char status);
int snd_exbox_clear_problems(struct usb_device *usbdev, bool enable_streaming);

enum { /* pcm streaming states */
	STREAM_DISABLED, /* no pcm streaming */
	STREAM_STARTING, /* pcm streaming requested, waiting to become ready */
	STREAM_RUNNING,  /* pcm streaming running */
	STREAM_STOPPING
};


#endif /* EXBOX_CHIP_H */
