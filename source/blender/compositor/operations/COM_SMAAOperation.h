/*
 * Copyright 2016, Blender Foundation.
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

#ifndef _COM_SMAAOperation_h
#define _COM_SMAAOperation_h
#include "COM_NodeOperation.h"

/*-----------------------------------------------------------------------------*/
/* Edge Detection (First Pass) */

class SMAAEdgeDetectionOperation : public NodeOperation {
protected:
	SocketReader *m_imageReader;
	SocketReader *m_valueReader;
	NodeAntiAliasingData m_config;
public:
	SMAAEdgeDetectionOperation();

	/**
	 * the inner loop of this program
	 */
	virtual void executePixel(float output[4], int x, int y, void *data) = 0;

	/**
	 * Initialize the execution
	 */
	void initExecution();

	/**
	 * Deinitialize the execution
	 */
	void deinitExecution();

	void setData(const NodeAntiAliasingData *data) { m_config = *data; }

	bool determineDependingAreaOfInterest(rcti *input, ReadBufferOperation *readOperation, rcti *output);

protected:
	void calculatePredicatedThreshold(int x, int y, float threshold[2]);
};

class SMAALumaEdgeDetectionOperation: public SMAAEdgeDetectionOperation {
public:
	void executePixel(float output[4], int x, int y, void *data);
};

class SMAAColorEdgeDetectionOperation: public SMAAEdgeDetectionOperation {
public:
	void executePixel(float output[4], int x, int y, void *data);
};

class SMAADepthEdgeDetectionOperation : public SMAAEdgeDetectionOperation {
public:
	void executePixel(float output[4], int x, int y, void *data);
	bool determineDependingAreaOfInterest(rcti *input, ReadBufferOperation *readOperation, rcti *output);
};

/*-----------------------------------------------------------------------------*/
/*  Blending Weight Calculation (Second Pass) */

class SMAABlendingWeightCalculationOperation : public NodeOperation {
private:
	SocketReader *m_imageReader;
	NodeAntiAliasingData m_config;
public:
	SMAABlendingWeightCalculationOperation();

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

	void setData(const NodeAntiAliasingData *data) { m_config = *data; }

	bool determineDependingAreaOfInterest(rcti *input, ReadBufferOperation *readOperation, rcti *output);
private:
	/* Diagonal Search Functions */
	int searchDiag1(int x, int y, int dx, int dy, float *end, bool *found);
	int searchDiag2(int x, int y, int dx, int dy, float *end, bool *found);
	void areaDiag(int d1, int d2, int e1, int e2, float weights[2]);
	void calculateDiagWeights(int x, int y, const float edges[2], float weights[2]);

	/* Horizontal/Vertical Search Functions */
	int searchXLeft(int x, int y);
	int searchXRight(int x, int y);
	int searchYUp(int x, int y);
	int searchYDown(int x, int y);
	void area(int d1, int d2, int e1, int e2, float weights[2]);

	/*  Corner Detection Functions */
	void detectHorizontalCornerPattern(float weights[2], int left, int right, int y, int d1, int d2);
	void detectVerticalCornerPattern(float weights[2], int x, int top, int bottom, int d1, int d2);
};

/*-----------------------------------------------------------------------------*/
/* Neighborhood Blending (Third Pass) */

class SMAANeighborhoodBlendingOperation : public NodeOperation {
private:
	SocketReader *m_image1Reader;
	SocketReader *m_image2Reader;
public:
	SMAANeighborhoodBlendingOperation();

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

	bool determineDependingAreaOfInterest(rcti *input, ReadBufferOperation *readOperation, rcti *output);
};

#endif
