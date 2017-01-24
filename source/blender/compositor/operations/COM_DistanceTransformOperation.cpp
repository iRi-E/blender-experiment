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

#include "COM_DistanceTransformOperation.h"
#include "MEM_guardedalloc.h"
#include "BLI_math.h"

static bool distance_transform_euclidean(int width, int height,
					 float threshold, bool invert,
					 const float *inbuf, float *outbuf)
{
	int *r = (int *)MEM_callocN(width * sizeof(int), "distance transform horizontal distance");
	int *f = (int *)MEM_callocN(width * height * sizeof(int), "distance transform square of horizontal distance");
	int *v = (int *)MEM_callocN(height * sizeof(int), "distance transform quadratic curve vertices");
	int *z = (int *)MEM_callocN(height * sizeof(int), "distance transform quadratic curve intersections");
	bool calculated = false;

	if (r != NULL && f != NULL && v != NULL && z != NULL) {
		int x, y;

		/* pass 1: horizontal processing */
		for (y = 0; y < height; y++) {
			const float *I = inbuf + y * width;
			int *F = f + y * width;

			/* thresholding */
			for (x = 0; x < width; x++) {
				if ((I[x] >= threshold) == !invert) {
					/* inside (distance > 0) */
					r[x] = width;
					F[x] = -1; /* -1 means not calculated yet */
				}
				else {
					/* outside (distance = 0) */
					r[x] = 0;
					// F[x] = 0; /* 0 won't be changed */
				}
			}

			/* left to right */
			for (x = 1; x < width; x++) {
				if (F[x] != 0 && F[x - 1] != -1) {
					r[x] = r[x - 1] + 1;
					F[x] = F[x - 1] + r[x - 1] + r[x];
				}
			}

			/* right to left */
			for (x = width - 2; x >= 0; x--) {
				if (F[x] != 0 && F[x + 1] != -1 && r[x] > r[x + 1]) {
					r[x] = r[x + 1] + 1;
					F[x] = F[x + 1] + r[x + 1] + r[x];
				}
			}
		}

		/* pass 2: vertical processing */
		for (x = 0; x < width; x++) {
			int k = 0;

			/* find the first valid distance */
			for (y = 0; y < height; y++) {
				if (f[x + y * width] != -1) {
					v[0] = y;
					break;
				}
			}

			/* is there a valid distance? */
			if (y == height)
				break;

			/* rearrange the quadratic curves so that they give minimum distances */
			for (y++ ; y < height; y++) {
				int f_xy = f[x + y * width];

				if (f_xy != -1) {
					int s;

					while (true) {
						/* calculate an intersection of two quadratic curves */
						s = ((f_xy - f[x + v[k] * width]) / (y - v[k]) + y + v[k]) / 2;

						/* discard previous z[k] and v[k] if s <= z[k] */
						if (k == 0 || s > z[k - 1])
							break;
						k--;
					}
					z[k] = s;
					k++;
					v[k] = y;
				}
			}
			z[k] = height;

			/* calculate the final Euclidean distances */
			k = 0;
			for (y = 0; y < height; y++) {
				while (z[k] < y)
					k++;
				int Ry = y - v[k];
				outbuf[x + y * width] = sqrtf((float)(Ry * Ry + f[x + v[k] * width]));
			}
		}

		/* is the whole image not masked? */
		if (x < width) {
			for (int i = width * height; i > 0; i--)
				*outbuf++ = FLT_MAX;
		}

		calculated = true;
	}

	if (r) MEM_freeN(r);
	if (f) MEM_freeN(f);
	if (v) MEM_freeN(v);
	if (z) MEM_freeN(z);

	return calculated;
}

DistanceTransformOperation::DistanceTransformOperation() : NodeOperation()
{
	this->addInputSocket(COM_DT_VALUE);
	this->addOutputSocket(COM_DT_VALUE);
	this->setComplex(true);
	this->m_valueReader = NULL;
	this->m_buffer = NULL;
	this->m_width = 0;
	this->m_height = 0;
	this->m_isCalculated = false;
	this->m_threshold = 0.5f;
	this->m_invert = false;
}

void DistanceTransformOperation::initExecution()
{
	this->m_valueReader = this->getInputSocketReader(0);
	NodeOperation::initMutex();
}

void *DistanceTransformOperation::initializeTileData(rcti *rect)
{
	lockMutex();

	if (!this->m_isCalculated) {
		MemoryBuffer *tile = (MemoryBuffer *)this->m_valueReader->initializeTileData(rect);
		float *inbuf = tile->getBuffer();
		unsigned int width = tile->getWidth();
		unsigned int height = tile->getHeight();

		if (width > 0 && height > 0) {
			this->m_buffer = (float *)MEM_callocN(width * height * sizeof(float), "distance transform buffer");
			this->m_width = width;
			this->m_height = height;

			if (this->m_buffer)
				this->m_isCalculated = distance_transform_euclidean(width, height,
										    this->m_threshold, this->m_invert,
										    inbuf, this->m_buffer);
		}
	}

	unlockMutex();
	return NULL;
}

void DistanceTransformOperation::executePixel(float output[4], int x, int y, void * /*data*/)
{
	if (this->m_isCalculated && this->m_buffer) {
		if (x >= 0 && x < this->m_width && y >= 0 && y < this->m_height)
			output[0] = this->m_buffer[x + y * this->m_width];
	}
}

void DistanceTransformOperation::deinitExecution()
{
	lockMutex();
	if (this->m_buffer)
		MEM_freeN(this->m_buffer);
	this->m_isCalculated = false;
	unlockMutex();

	this->m_valueReader = NULL;
	NodeOperation::deinitMutex();
}

bool DistanceTransformOperation::determineDependingAreaOfInterest(rcti * /*input*/, ReadBufferOperation *readOperation, rcti *output)
{
	rcti valueInput;
	if (this->m_isCalculated) return false;

	NodeOperation *operation = getInputOperation(0);
	valueInput.xmax = operation->getWidth();
	valueInput.xmin = 0;
	valueInput.ymax = operation->getHeight();
	valueInput.ymin = 0;

	if (operation->determineDependingAreaOfInterest(&valueInput, readOperation, output) ) {
		return true;
	}
	return false;
}
