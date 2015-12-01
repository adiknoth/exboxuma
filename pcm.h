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

#ifndef EXBOX_PCM_H
#define EXBOX_PCM_H

#include <sound/pcm.h>
#include <linux/slab.h>
#include "chip.h"

struct exbox_chip;

int exbox_pcm_init(struct exbox_chip *chip, u8 extra_freq);
void exbox_pcm_abort(struct exbox_chip *chip);
void exbox_pcm_destroy(struct exbox_chip *chip);

int exbox_pcm_stream_start(struct pcm_runtime *rt);
void exbox_pcm_stream_stop(struct pcm_runtime *rt);

#define PCM_N_URBS      1024

struct pcm_urb {
	struct exbox_chip *chip;

	struct urb instance;
	struct usb_anchor submitted;
	struct pcm_urb *peer;

	u8 *buffer;
};

struct pcm_substream {
	spinlock_t lock;
	struct snd_pcm_substream *instance;

	bool active;
	snd_pcm_uframes_t dma_off;    /* current position in alsa dma_area */
	snd_pcm_uframes_t period_off; /* current position in current period */
};


struct pcm_runtime {
	struct exbox_chip *chip;
	struct snd_pcm *instance;

	struct pcm_substream playback;
	struct pcm_substream capture;
	bool panic; /* if set driver won't do anymore pcm on device */

	struct pcm_urb out_urbs[PCM_N_URBS];
	struct pcm_urb in_urbs[PCM_N_URBS];

	struct mutex stream_mutex;
	u8 stream_state; /* one of STREAM_XXX */
	u8 extra_freq;
	wait_queue_head_t stream_wait_queue;
	bool stream_wait_cond;
};

#endif /* EXBOX_PCM_H */
