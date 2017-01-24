/*
 * Copyright 2017, Blender Foundation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contributor: IRIE Shinsuke
 */

#ifndef _COM_DistanceTransformOperation_h
#define _COM_DistanceTransformOperation_h
#include "COM_NodeOperation.h"

class DistanceTransformOperation : public NodeOperation {
private:
	SocketReader *m_valueReader;
	float *m_buffer;
	unsigned int m_width;
	unsigned int m_height;
	bool m_isCalculated;
	float m_threshold;
	bool m_invert;
public:
	DistanceTransformOperation();

	/**
	 * the inner loop of this program
	 */
	void executePixel(float output[4], int x, int y, void *data);

	/**
	 * Initialize the execution
	 */
	void initExecution();
	void *initializeTileData(rcti *rect);

	/**
	 * Deinitialize the execution
	 */
	void deinitExecution();

	void setThreshold(float threshold) { m_threshold = threshold; }
	void setInvert(bool invert) { m_invert = invert; }
	bool determineDependingAreaOfInterest(rcti *input, ReadBufferOperation *readOperation, rcti *output);
};

#endif
