/*
 * ctw6120.h - header for TWL4030 PM and audio CODEC device
 *
 * Copyright (C) 2010 Variscite LTD.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#ifndef __CTW6120_H_
#define __CTW6120_H_

#include <linux/types.h>


#define CTW6120_STATUS_REG		0x00
#define CTW6120_X_HIGH_REG		0x01
#define CTW6120_X_LOW_REG		0x02
#define CTW6120_Y_HIGH_REG		0x03
#define CTW6120_Y_LOW_REG		0x04
#define CTW6120_BUTTON_REG		0x05
#define CTW6120_MOVEMENT_REG		0x06
#define CTW6120_GESTURE_REG		0x09
#define CTW6120_SLEEP_MODE_REG		0x0A
#define CTW6120_SENSITIVITY_REG		0x0B
#define CTW6120_FIRMWARE_REG		0x0C


#define CTW6120_STATUS_NO_FINGERS	0
#define CTW6120_STATUS_1_FINGER		1
#define CTW6120_STATUS_2_FINGERS	2

#define CTW6120_BUTTON_LEFT		1
#define CTW6120_BUTTON_RIGHT		2
#define CTW6120_BUTTON_MID		4

#define CTW6120_ZOOM_IN			1
#define CTW6120_ZOOM_OUT		-1

#define CTW6120_GESTURE_NONE		0x00
#define CTW6120_GESTURE_ZOOM_IN		0x01
#define CTW6120_GESTURE_ZOOM_OUT	0x02
#define CTW6120_GESTURE_ST_PAN_UP	0x03
#define CTW6120_GESTURE_ST_ROTATE_CW	0x04
#define CTW6120_GESTURE_ST_PAN_RIGHT	0x05
#define CTW6120_GESTURE_ST_PAN_DOWN	0x06
#define CTW6120_GESTURE_ST_PAN_LEFT	0x07
#define CTW6120_GESTURE_ST_ROTATE_CCW	0x08
#define CTW6120_GESTURE_ST_CLICK	0x09
#define CTW6120_GESTURE_MT_PAN_UP	0x0A
#define CTW6120_GESTURE_MT_PAN_RIGHT	0x0B
#define CTW6120_GESTURE_MT_PAN_DOWN	0x0C
#define CTW6120_GESTURE_MT_PAN_LEFT	0x0D
#define CTW6120_GESTURE_ST_DOUBLE_CLICK	0x0E
#define CTW6120_GESTURE_MT_CLICK	0x0F

#define CTW6120_GESTURE_ZOOM		((CTW6120_GESTURE_ZOOM_IN) | (CTW6120_GESTURE_ZOOM_OUT))

#define CTW6120_SLEEP_ENTER		0x01
#define CTW6120_SLEEP_ESCAPE		0x00

#endif /* __CTW6120_H_ */