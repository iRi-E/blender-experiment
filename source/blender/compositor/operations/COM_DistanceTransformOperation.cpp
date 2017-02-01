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
	int *h = (int *)MEM_callocN(width * height * sizeof(int), "distance transform relative coordinate x");
	int *v = (int *)MEM_callocN(height * sizeof(int), "distance transform quadratic curve vertices");
	int *Rx = (int *)MEM_callocN(height * sizeof(int), "distance transform relative coordinate x of quadratic curve vertices");
	int *z = (int *)MEM_callocN(height * sizeof(int), "distance transform quadratic curve intersections");
	bool calculated = false;

	if (r != NULL && f != NULL && h != NULL && v != NULL && Rx != NULL && z != NULL) {
		int x, y;

		/* first pass: horizontal processing */
		for (y = 0; y < height; y++) {
			const float *I = inbuf + y * width;
			int *F = f + y * width;
			int *H = h + y * width;

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
				// H[x] = 0;
			}

			/* left to right */
			for (x = 1; x < width; x++) {
				if (F[x] != 0 && F[x - 1] != -1) {
					r[x] = r[x - 1] + 1;
					F[x] = F[x - 1] + r[x - 1] + r[x];
					H[x] = r[x];
				}
			}

			/* right to left */
			for (x = width - 2; x >= 0; x--) {
				if (F[x] != 0 && F[x + 1] != -1 && r[x] > r[x + 1]) {
					r[x] = r[x + 1] + 1;
					F[x] = F[x + 1] + r[x + 1] + r[x];
					H[x] = -r[x];
				}
			}
		}

		/* second pass: vertical processing */
		for (x = 0; x < width; x++) {
			int k = 0;

			/* find the first valid distance */
			for (y = 0; y < height; y++) {
				if (f[x + y * width] != -1) {
					v[0] = y;
					Rx[0] = h[x + y * width];
					break;
				}
			}

			/* is there a valid distance? */
			if (y == height)
				break;

			/* rearrange the quadratic curves so that they give minimum distances */
			for (y++ ; y < height; y++) {
				int x_y = x + y * width;

				if (f[x_y] != -1) {
					int s;

					while (true) {
						/* calculate an intersection of two quadratic curves */
						s = ((f[x_y] - f[x + v[k] * width]) / (y - v[k]) + y + v[k]) / 2;

						/* discard previous z[k] and v[k] if s <= z[k] */
						if (k == 0 || s > z[k - 1])
							break;
						k--;
					}
					z[k] = s;
					k++;
					v[k] = y;
					Rx[k] = h[x_y];
				}
			}
			z[k] = height;

			/* calculate the final Euclidean distances */
			k = 0;
			for (y = 0; y < height; y++) {
				while (z[k] < y)
					k++;
				int Ry = y - v[k];
				float *ptr = &outbuf[(x + y * width) * 3];
				*ptr++ = sqrtf((float)(Ry * Ry + f[x + v[k] * width]));
				*ptr++ = (float)Rx[k];
				*ptr   = (float)Ry;
			}
		}

		/* is the whole image not masked? */
		if (x < width) {
			for (int i = width * height; i > 0; i--) {
				*outbuf++ = FLT_MAX;
				*outbuf++ = 0.0f;
				*outbuf++ = 0.0f;
			}
		}

		calculated = true;
	}

	if (r) MEM_freeN(r);
	if (f) MEM_freeN(f);
	if (h) MEM_freeN(h);
	if (v) MEM_freeN(v);
	if (Rx) MEM_freeN(Rx);
	if (z) MEM_freeN(z);

	return calculated;
}

DistanceTransformOperation::DistanceTransformOperation() : NodeOperation()
{
	this->addInputSocket(COM_DT_VALUE);
	this->addOutputSocket(COM_DT_VECTOR);
	this->setComplex(true);
	this->m_valueReader = NULL;
	this->m_isCalculated = false;
	this->m_factor = 1.0f;
	this->m_threshold = 0.5f;
	this->m_invert = false;
	this->m_relative = false;
}

void DistanceTransformOperation::initExecution()
{
	this->m_valueReader = this->getInputSocketReader(0);
}

/* small helper to pass data from initializeTileData to executePixel */
typedef struct tile_info {
	unsigned int width;
	unsigned int height;
	float *buffer;
} tile_info;

void *DistanceTransformOperation::initializeTileData(rcti *rect)
{
	MemoryBuffer *tile = (MemoryBuffer *)this->m_valueReader->initializeTileData(rect);
	float *inbuf = tile->getBuffer();
	unsigned int width = tile->getWidth();
	unsigned int height = tile->getHeight();

	if (width == 0 || height == 0)
		return NULL;

	if (this->m_relative)
		this->m_factor = 100.0f / max(width, height);

	tile_info *result = (tile_info *)MEM_mallocN(sizeof(tile_info), "distance transform tile");

	if (result) {
		result->width = width;
		result->height = height;
		result->buffer = (float *)MEM_callocN(width * height * 3 * sizeof(float), "distance transform cache");

		if (result->buffer)
			this->m_isCalculated = distance_transform_euclidean(width, height,
									    this->m_threshold, this->m_invert,
									    inbuf, result->buffer);
	}

	return result;
}

void DistanceTransformOperation::executePixel(float output[4], int x, int y, void *data)
{
	if (this->m_isCalculated) {
		tile_info *tile = (tile_info *)data;
		if (x >= 0 && x < tile->width && y >= 0 && y < tile->height) {
			float *ptr = &tile->buffer[(x + y * tile->width) * 3];
			output[0] = *ptr++ * this->m_factor; /* Distance */
			output[1] = *ptr++ * this->m_factor; /* Vector X */
			output[2] = *ptr   * this->m_factor; /* Vector Y */
		}
	}
}

void DistanceTransformOperation::deinitExecution()
{
	this->m_valueReader = NULL;
}

void DistanceTransformOperation::deinitializeTileData(rcti * /*rect*/, void *data)
{
	tile_info *tile = (tile_info *)data;
	if (tile->buffer)
		MEM_freeN(tile->buffer);
	MEM_freeN(tile);
}

bool DistanceTransformOperation::determineDependingAreaOfInterest(rcti * /*input*/, ReadBufferOperation *readOperation, rcti *output)
{
	rcti valueInput;

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
