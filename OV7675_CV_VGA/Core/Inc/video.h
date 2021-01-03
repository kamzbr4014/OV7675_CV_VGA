/*
 * video.h
 *
 *  Created on: 20 gru 2020
 *      Author: kamil
 */

#ifndef SRC_VIDEO_H_
#define SRC_VIDEO_H_

#include "main.h"

#define	VID_HSIZE		50		// Horizontal resolution (in bytes)
#define	VID_VSIZE		200		// Vertical resolution (in lines)

#define	VID_PIXELS_X	(VID_HSIZE * 8)
#define	VID_PIXELS_Y	VID_VSIZE
#define	VID_PIXELS_XR	(VID_PIXELS_X + 16)
#define	VID_HSIZE_R		(VID_HSIZE + 2)

void Video_Init(void);
void vidClearScreen(void);

//---- TMP functions -------
void sysDelayMs(uint32_t dly);
void sysDelay1Ms(void);

#endif /* SRC_VIDEO_H_ */
