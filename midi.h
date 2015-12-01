/*
 * Linux driver for D.O.Tec EXBOX.UMA (PloyTec USB interface)
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

#ifndef EXBOX_MIDI_H
#define EXBOX_MIDI_H

#include <linux/slab.h>
#include <sound/rawmidi.h>
#include "chip.h"

#define MIDI_N_URBS 1 /* Eight are also known to work */

int exbox_midi_init(struct exbox_chip *chip);
void exbox_midi_abort(struct exbox_chip *chip);
void exbox_midi_destroy(struct exbox_chip *chip);
int exbox_midi_stream_start(struct midi_runtime *rt);
void exbox_midi_stream_stop(struct midi_runtime *rt);

struct midi_urb {
	struct exbox_chip *chip;

	struct urb instance;
	struct usb_anchor submitted;

	u8 *buffer;
};

struct midi_runtime {
	struct exbox_chip *chip;
	struct snd_rawmidi *instance;

	struct snd_rawmidi_substream *in;
	char in_active;

	spinlock_t in_lock;
	spinlock_t out_lock;
	struct snd_rawmidi_substream *out;
	struct urb out_urb;
	u8 *out_buffer;
	int buffer_offset;
	struct midi_urb in_urbs[MIDI_N_URBS];

	struct mutex stream_mutex;
	u8 stream_state; /* one of STREAM_XXX */
	wait_queue_head_t stream_wait_queue;
	bool stream_wait_cond;
	bool panic; /* if set driver won't do anymore midi on device */
};

#endif /* EXBOX_MIDI_H */

