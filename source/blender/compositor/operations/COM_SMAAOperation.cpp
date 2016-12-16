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

#include "COM_SMAAOperation.h"
#include "COM_SMAAAreaTexture.h"
#include "BLI_math.h"

/*
 * An implementation of Enhanced Subpixel Morphological Antialiasing (SMAA)
 *
 * The algorithm was proposed by:
 *   Jorge Jimenez, Jose I. Echevarria, Tiago Sousa, Diego Gutierrez
 *
 * Homepage URL:
 *   http://www.iryoku.com/smaa/
 *
 * This program provides only SMAA 1x mode, so the operation will be done
 * with no spatial multisampling nor temporal supersampling.
 *
 * Note: This program assumes the screen coordinates are DirectX style, so
 * the vertical direction is upside-down. "top" and "bottom" actually mean
 * bottom and top, respectively.
 */

/*-----------------------------------------------------------------------------*/
/* Non-Configurable Defines */

#define SMAA_AREATEX_SIZE 80
#define SMAA_AREATEX_MAX_DISTANCE 16
#define SMAA_AREATEX_MAX_DISTANCE_DIAG 20

/*-----------------------------------------------------------------------------*/
/* Internal Functions to Sample Pixel Color */

static int clamp_index(int x, int range)
{
	return CLAMPIS(x, 0, range - 1);
}

static void sample(SocketReader *reader, int x, int y, float color[4])
{
	reader->read(color, clamp_index(x, reader->getWidth()), clamp_index(y, reader->getHeight()), NULL);
}

static void sample_level_zero_yoffset(SocketReader *reader, int x, int y, float yoffset, float color[4])
{
	float iy = floorf(yoffset);
	float fy = yoffset - iy;
	y += (int)iy;

	float color00[4], color01[4];

	sample(reader, x + 0, y + 0, color00);
	sample(reader, x + 0, y + 1, color01);

	color[0] = interpf(color01[0], color00[0], fy);
	color[1] = interpf(color01[1], color00[1], fy);
	color[2] = interpf(color01[2], color00[2], fy);
	color[3] = interpf(color01[3], color00[3], fy);
}

static void sample_level_zero_xoffset(SocketReader *reader, int x, int y, float xoffset, float color[4])
{
	float ix = floorf(xoffset);
	float fx = xoffset - ix;
	x += (int)ix;

	float color00[4], color10[4];

	sample(reader, x + 0, y + 0, color00);
	sample(reader, x + 1, y + 0, color10);

	color[0] = interpf(color10[0], color00[0], fx);
	color[1] = interpf(color10[1], color00[1], fx);
	color[2] = interpf(color10[2], color00[2], fx);
	color[3] = interpf(color10[3], color00[3], fx);
}

static const float* areatex_sample_internal(const float *areatex, int x, int y)
{
	return &areatex[(clamp_index(x, SMAA_AREATEX_SIZE) + clamp_index(y, SMAA_AREATEX_SIZE) * SMAA_AREATEX_SIZE) * 2];
}

static void areatex_sample_level_zero(const float *areatex, float x, float y, float weights[2])
{
	float ix = floorf(x), iy = floorf(y);
	float fx = x - ix, fy = y - iy;
	int X = (int)ix, Y = (int)iy;

	const float *weights00 = areatex_sample_internal(areatex, X + 0, Y + 0);
	const float *weights10 = areatex_sample_internal(areatex, X + 1, Y + 0);
	const float *weights01 = areatex_sample_internal(areatex, X + 0, Y + 1);
	const float *weights11 = areatex_sample_internal(areatex, X + 1, Y + 1);

	weights[0] = interpf(interpf(weights11[0], weights01[0], fx), interpf(weights10[0], weights00[0], fx), fy);
	weights[1] = interpf(interpf(weights11[1], weights01[1], fx), interpf(weights10[1], weights00[1], fx), fy);
}

/*-----------------------------------------------------------------------------*/
/* Edge Detection (First Pass) */
/*-----------------------------------------------------------------------------*/

SMAAEdgeDetectionOperation::SMAAEdgeDetectionOperation() : NodeOperation()
{
	this->addInputSocket(COM_DT_COLOR); /* image */
	this->addInputSocket(COM_DT_VALUE); /* predication or depth */
	this->addOutputSocket(COM_DT_COLOR);
	this->setComplex(true);
	this->m_imageReader = NULL;
	this->m_valueReader = NULL;
	memset(&m_config, 0, sizeof(NodeAntiAliasingData));
}

void SMAAEdgeDetectionOperation::initExecution()
{
	this->m_imageReader = this->getInputSocketReader(0);
	this->m_valueReader = this->getInputSocketReader(1);
}

void SMAAEdgeDetectionOperation::deinitExecution()
{
	this->m_imageReader = NULL;
	this->m_valueReader = NULL;
}

bool SMAAEdgeDetectionOperation::determineDependingAreaOfInterest(rcti *input,
								  ReadBufferOperation *readOperation, rcti *output)
{
	rcti newInput;
	newInput.xmax = input->xmax + 1;
	newInput.xmin = input->xmin - 2;
	newInput.ymax = input->ymax + 1;
	newInput.ymin = input->ymin - 2;

	return NodeOperation::determineDependingAreaOfInterest(&newInput, readOperation, output);
}

/* Predication */

void SMAAEdgeDetectionOperation::calculatePredicatedThreshold(int x, int y, float threshold[2])
{
	float here[4], left[4], top[4];

	sample(m_valueReader, x, y, here);
	sample(m_valueReader, x - 1, y, left);
	sample(m_valueReader, x, y - 1, top);

	copy_v2_fl(threshold, 1.0f);

	if (fabsf(here[0] - left[0]) >= m_config.pred_thresh)
		threshold[0] -= m_config.pred_str;
	if (fabsf(here[0] - top[0]) >= m_config.pred_thresh)
		threshold[1] -= m_config.pred_str;

	mul_v2_fl(threshold, m_config.pred_scale * m_config.thresh);
}

/* Luma Edge Detection */

void SMAALumaEdgeDetectionOperation::executePixel(float output[4], int x, int y, void */*data*/)
{
	float threshold[2], color[4];
	float L, Lleft, Ltop, Dleft, Dtop;
	float Lright, Lbottom, Dright, Dbottom;
	float Lleftleft, Ltoptop, Dleftleft, Dtoptop;
	float delta_x, delta_y, finalDelta;

	/* RGB weights to convert to luma */
	float weights[3] = {0.2126f, 0.7152f, 0.0722f};

	/* Calculate the threshold: */
	if (m_config.pred)
		calculatePredicatedThreshold(x, y, threshold);
	else
		threshold[0] = threshold[1] = m_config.thresh;

	/* Calculate luma deltas: */
	sample(m_imageReader, x, y, color);
	L     = dot_v3v3(color, weights);
	sample(m_imageReader, x - 1, y, color);
	Lleft = dot_v3v3(color, weights);
	sample(m_imageReader, x, y - 1, color);
	Ltop  = dot_v3v3(color, weights);
	Dleft = fabsf(L - Lleft);
	Dtop  = fabsf(L - Ltop);

	/* We do the usual threshold: */
	output[0] = (Dleft >= threshold[0]) ? 1.0f : 0.0f;
	output[1] = (Dtop >= threshold[1]) ? 1.0f : 0.0f;
	output[2] = 0.0f;
	output[3] = 1.0f;

	/* Then discard if there is no edge: */
	if (is_zero_v2(output))
	    return;

	/* Calculate right and bottom deltas: */
	sample(m_imageReader, x + 1, y, color);
	Lright  = dot_v3v3(color, weights);
	sample(m_imageReader, x, y + 1, color);
	Lbottom = dot_v3v3(color, weights);
	Dright  = fabsf(L - Lright);
	Dbottom = fabsf(L - Lbottom);

	/* Calculate the maximum delta in the direct neighborhood: */
	delta_x = fmaxf(Dleft, Dright);
	delta_y = fmaxf(Dtop, Dbottom);

	/* Calculate left-left and top-top deltas: */
	sample(m_imageReader, x - 2, y, color);
	Lleftleft = dot_v3v3(color, weights);
	sample(m_imageReader, x, y - 2, color);
	Ltoptop   = dot_v3v3(color, weights);
	Dleftleft = fabsf(Lleft - Lleftleft);
	Dtoptop   = fabsf(Ltop - Ltoptop);

	/* Calculate the final maximum delta: */
	delta_x = fmaxf(delta_x, Dleftleft);
	delta_y = fmaxf(delta_y, Dtoptop);
	finalDelta = fmaxf(delta_x, delta_y);

	/* Local contrast adaptation: */
	if (finalDelta > m_config.adapt_fac * Dleft)
		output[0] =  0.0f;
	if (finalDelta > m_config.adapt_fac * Dtop)
		output[1] =  0.0f;
}

/* Color Edge Detection */

void SMAAColorEdgeDetectionOperation::executePixel(float output[4], int x, int y, void */*data*/)
{
	float threshold[2];
	float C[4], Cleft[4], Ctop[4], Dleft, Dtop;
	float Cright[4], Cbottom[4], Dright, Dbottom;
	float Cleftleft[4], Ctoptop[4], Dleftleft, Dtoptop;
	float delta_x, delta_y, finalDelta;

	/* Calculate the threshold: */
	if (m_config.pred)
		calculatePredicatedThreshold(x, y, threshold);
	else
		threshold[0] = threshold[1] = m_config.thresh;

	/* Calculate color deltas: */
	sample(m_imageReader, x, y, C);
	sample(m_imageReader, x - 1, y, Cleft);
	sample(m_imageReader, x, y - 1, Ctop);
	Dleft = fmaxf(fmaxf(fabsf(C[0] - Cleft[0]), fabsf(C[1] - Cleft[1])), fabsf(C[2] - Cleft[2]));
	Dtop  = fmaxf(fmaxf(fabsf(C[0] - Ctop[0]), fabsf(C[1] - Ctop[1])), fabsf(C[2] - Ctop[2]));

	/* We do the usual threshold: */
	output[0] = (Dleft >= threshold[0]) ? 1.0f : 0.0f;
	output[1] = (Dtop >= threshold[1]) ? 1.0f : 0.0f;
	output[2] = 0.0f;
	output[3] = 1.0f;

	/* Then discard if there is no edge: */
	if (is_zero_v2(output))
	    return;

	/* Calculate right and bottom deltas: */
	sample(m_imageReader, x + 1, y, Cright);
	sample(m_imageReader, x, y + 1, Cbottom);
	Dright  = fmaxf(fmaxf(fabsf(C[0] - Cright[0]), fabsf(C[1] - Cright[1])), fabsf(C[2] - Cright[2]));
	Dbottom = fmaxf(fmaxf(fabsf(C[0] - Cbottom[0]), fabsf(C[1] - Cbottom[1])), fabsf(C[2] - Cbottom[2]));

	/* Calculate the maximum delta in the direct neighborhood: */
	delta_x = fmaxf(Dleft, Dright);
	delta_y = fmaxf(Dtop, Dbottom);

	/* Calculate left-left and top-top deltas: */
	sample(m_imageReader, x - 2, y, Cleftleft);
	sample(m_imageReader, x, y - 2, Ctoptop);
	Dleftleft = fmaxf(fmaxf(fabsf(C[0] - Cleftleft[0]), fabsf(C[1] - Cleftleft[1])), fabsf(C[2] - Cleftleft[2]));
	Dtoptop   = fmaxf(fmaxf(fabsf(C[0] - Ctoptop[0]), fabsf(C[1] - Ctoptop[1])), fabsf(C[2] - Ctoptop[2]));

	/* Calculate the final maximum delta: */
	delta_x = fmaxf(delta_x, Dleftleft);
	delta_y = fmaxf(delta_y, Dtoptop);
	finalDelta = fmaxf(delta_x, delta_y);

	/* Local contrast adaptation: */
	if (finalDelta > m_config.adapt_fac * Dleft)
		output[0] =  0.0f;
	if (finalDelta > m_config.adapt_fac * Dtop)
		output[1] =  0.0f;
}

/* Depth Edge Detection */

void SMAADepthEdgeDetectionOperation::executePixel(float output[4], int x, int y, void */*data*/)
{
	float here[4], left[4], top[4];

	sample(m_valueReader, x, y, here);
	sample(m_valueReader, x - 1, y, left);
	sample(m_valueReader, x, y - 1, top);

	output[0] = (fabsf(here[0] - left[0]) >= m_config.dept_thresh) ? 1.0f : 0.0f;
	output[1] = (fabsf(here[0] - top[0]) >= m_config.dept_thresh) ? 1.0f : 0.0f;
	output[2] = 0.0f;
	output[3] = 1.0f;
}

bool SMAADepthEdgeDetectionOperation::determineDependingAreaOfInterest(rcti *input,
								       ReadBufferOperation *readOperation, rcti *output)
{
	rcti newInput;

	newInput.xmax = input->xmax;
	newInput.xmin = input->xmin - 1;
	newInput.ymax = input->ymax;
	newInput.ymin = input->ymin - 1;

	return NodeOperation::determineDependingAreaOfInterest(&newInput, readOperation, output);
}

/*-----------------------------------------------------------------------------*/
/* Blending Weight Calculation (Second Pass) */
/*-----------------------------------------------------------------------------*/

SMAABlendingWeightCalculationOperation::SMAABlendingWeightCalculationOperation() : NodeOperation()
{
	this->addInputSocket(COM_DT_COLOR); /* edges */
	this->addOutputSocket(COM_DT_COLOR);
	this->setComplex(true);
	this->m_imageReader = NULL;
	memset(&m_config, 0, sizeof(NodeAntiAliasingData));
}

void *SMAABlendingWeightCalculationOperation::initializeTileData(rcti *rect)
{
	return getInputOperation(0)->initializeTileData(rect);
}

void SMAABlendingWeightCalculationOperation::initExecution()
{
	this->m_imageReader = this->getInputSocketReader(0);
}

void SMAABlendingWeightCalculationOperation::executePixel(float output[4], int x, int y, void */*data*/)
{
	float e[4], w[2];

	zero_v4(output);
	sample(m_imageReader, x, y, e);

	/* Edge at north */
	if (e[1] > 0.0f) {
		if (m_config.diag) {
			/* Diagonals have both north and west edges, so searching for them in */
			/* one of the boundaries is enough. */
			calculateDiagWeights(x, y, e, w);

			/* We give priority to diagonals, so if we find a diagonal we skip  */
			/* horizontal/vertical processing. */
			if (!is_zero_v2(w)) {
				copy_v2_v2(output, w);
				return;
			}
		}

		/* Find the distance to the left and the right: */
		int left = searchXLeft(x, y);
		int right = searchXRight(x, y);
		int d[2] = {abs(left - x), abs(right - x)};

		/* Now fetch the left and right crossing edges, two at a time using bilinear */
		/* filtering. Sampling at -0.25 enables to discern what value each edge has: */
		float c[4], e1, e2;
		sample_level_zero_yoffset(m_imageReader, left, y, -0.25f, c);
		e1 = c[0];
		sample_level_zero_yoffset(m_imageReader, right + 1, y, -0.25f, c);
		e2 = c[0];

		/* area() below needs a sqrt, as the areas texture is compressed */
		/* quadratically: */
		float sqrt_d[2] = {sqrtf((float)d[0]), sqrtf((float)d[1])};

		/* Ok, we know how this pattern looks like, now it is time for getting */
		/* the actual area: */
		area(sqrt_d, e1, e2, w);

		/* Fix corners: */
		if (m_config.corner)
			detectHorizontalCornerPattern(w, left, right, y, d);

		copy_v2_v2(output, w);
	}

	/* Edge at west */
	if (e[0] > 0.0f) {
		/* Find the distance to the top and the bottom: */
		int top = searchYUp(x, y);
		int bottom = searchYDown(x, y);
		int d[2] = {abs(top - y), abs(bottom - y)};

		/* Fetch the top ang bottom crossing edges: */
		float c[4], e1, e2;
		sample_level_zero_xoffset(m_imageReader, x, top, -0.25f, c);
		e1 = c[1];
		sample_level_zero_xoffset(m_imageReader, x, bottom + 1, -0.25f, c);
		e2 = c[1];

		/* area() below needs a sqrt, as the areas texture is compressed */
		/* quadratically: */
		float sqrt_d[2] = {sqrtf((float)d[0]), sqrtf((float)d[1])};

		/* Get the area for this direction: */
		area(sqrt_d, e1, e2, w);

		/* Fix corners: */
		if (m_config.corner)
			detectVerticalCornerPattern(w, x, top, bottom, d);

		copy_v2_v2(output + 2, w);
	}
}

void SMAABlendingWeightCalculationOperation::deinitExecution()
{
	this->m_imageReader = NULL;
}

bool SMAABlendingWeightCalculationOperation::determineDependingAreaOfInterest(rcti *input,
									      ReadBufferOperation *readOperation, rcti *output)
{
	rcti newInput;
	int max_distance = max(m_config.search_steps * 2, m_config.diag ? m_config.search_steps_diag : 0);

	newInput.xmax = input->xmax + max_distance;
	newInput.xmin = input->xmin - max_distance;
	newInput.ymax = input->ymax + max_distance;
	newInput.ymin = input->ymin - max_distance;

	return NodeOperation::determineDependingAreaOfInterest(&newInput, readOperation, output);
}

/*-----------------------------------------------------------------------------*/
/* Diagonal Search Functions */

/**
 * These functions allows to perform diagonal pattern searches.
 */
int SMAABlendingWeightCalculationOperation::searchDiag1(int x, int y, int dx, int dy, float *end, bool *found)
{
	float e[4];
	int d = -1;
	*found = false;

	while (d < m_config.search_steps_diag - 1) {
		x += dx;
		y += dy;
		d++;
		sample(m_imageReader, x, y, e);
		if (e[0] <= 0.9f || e[1] <= 0.9f) {
			*found = true;
			break;
		}
	}

	*end = e[1];
	return d;
}

int SMAABlendingWeightCalculationOperation::searchDiag2(int x, int y, int dx, int dy, float *end, bool *found)
{
	float e1[4], e2[4];
	int d = -1;
	*found = false;

	while (d < m_config.search_steps_diag - 1) {
		x += dx;
		y += dy;
		d++;
		sample(m_imageReader, x + 1, y, e1);
		sample(m_imageReader, x, y, e2);
		if (e1[0] <= 0.9f || e2[1] <= 0.9f) {
			*found = true;
			break;
		}
	}

	*end = e2[1];
	return d;
}

/**
 * Similar to area(), this calculates the area corresponding to a certain
 * diagonal distance and crossing edges 'e'.
 */
void SMAABlendingWeightCalculationOperation::areaDiag(int d1, int d2, int e1, int e2, float weights[2])
{
	float x = (float)(SMAA_AREATEX_MAX_DISTANCE_DIAG * e1 + d1);
	float y = (float)(SMAA_AREATEX_MAX_DISTANCE_DIAG * e2 + d2);

	/* We do a bias for mapping to texel space: */
	x += 0.5f;
	y += 0.5f;

	/* Do it! */
	areatex_sample_level_zero(areatex_diag, x, y, weights);
}

/**
 * This searches for diagonal patterns and returns the corresponding weights.
 */
void SMAABlendingWeightCalculationOperation::calculateDiagWeights(int x, int y, float e[2], float weights[2])
{
	int d1, d2;
	bool d1_found, d2_found;
	float end, edges[4];

	zero_v2(weights);

	/* Search for the line ends: */
	if (e[0] > 0.0f) {
		d1 = searchDiag1(x, y, -1, 1, &end, &d1_found);
		d1 += (int)end;
	}
	else {
		d1 = 0;
		d1_found = true;
	}
	d2 = searchDiag1(x, y, 1, -1, &end, &d2_found);

	if (d1 + d2 > 2) { /* d1 + d2 + 1 > 3 */
		int c[2];
		int e1 = 0, e2 = 0;

		if (d1_found) {
			/* Fetch the crossing edges: */
			int coords_x = x - d1, coords_y = y + d1;

			sample(m_imageReader, coords_x - 1, coords_y, edges);
			c[0] = (int)edges[1];
			sample(m_imageReader, coords_x, coords_y, edges);
			c[1] = (int)edges[0];

			/* Merge crossing edges at each side into a single value: */
			e1 = 2 * c[0] + c[1];
		}

		if (d2_found) {
			/* Fetch the crossing edges: */
			int coords_x = x + d2, coords_y = y - d2;

			sample(m_imageReader, coords_x + 1, coords_y, edges);
			c[0] = (int)edges[1];
			sample(m_imageReader, coords_x + 1, coords_y - 1, edges);
			c[1] = (int)edges[0];

			/* Merge crossing edges at each side into a single value: */
			e2 = 2 * c[0] + c[1];
		}

		/* Fetch the areas for this line: */
		areaDiag(d1, d2, e1, e2, weights);
	}

	/* Search for the line ends: */
	d1 = searchDiag2(x, y, -1, -1, &end, &d1_found);
	sample(m_imageReader, x + 1, y, edges);
	if (edges[0] > 0.0f) {
		d2 = searchDiag2(x, y, 1, 1, &end, &d2_found);
		d2 += (int)end;
	}
	else {
		d2 = 0;
		d2_found = true;
	}

	if (d1 + d2 > 2) { /* d1 + d2 + 1 > 3 */
		int c[2];
		int e1 = 0, e2 = 0;

		if (d1_found) {
			/* Fetch the crossing edges: */
			int coords_x = x - d1, coords_y = y - d1;

			sample(m_imageReader, coords_x - 1, coords_y, edges);
			c[0] = (int)edges[1];
			sample(m_imageReader, coords_x, coords_y - 1, edges);
			c[1] = (int)edges[0];

			/* Merge crossing edges at each side into a single value: */
			e1 = 2 * c[0] + c[1];
		}

		if (d2_found) {
			/* Fetch the crossing edges: */
			int coords_x = x + d2, coords_y = y + d2;

			sample(m_imageReader, coords_x + 1, coords_y, edges);
			c[0] = (int)edges[1];
			c[1] = (int)edges[0];

			/* Merge crossing edges at each side into a single value: */
			e2 = 2 * c[0] + c[1];
		}

		/* Fetch the areas for this line: */
		float w[2];
		areaDiag(d1, d2, e1, e2, w);
		weights[0] += w[1];
		weights[1] += w[0];
	}
}

/*-----------------------------------------------------------------------------*/
/* Horizontal/Vertical Search Functions */

int SMAABlendingWeightCalculationOperation::searchXLeft(int x, int y)
{
	int end = x - 2 * m_config.search_steps;
	float e[4];

	while (x >= end) {
		sample(m_imageReader, x, y, e);
		if (e[1] == 0.0f || /* Is the edge not activated? */
		    e[0] != 0.0f)   /* Or is there a crossing edge that breaks the line? */
			break;
		sample(m_imageReader, x, y - 1, e);
		if (e[0] != 0.0f)   /* Or is there a crossing edge that breaks the line? */
			break;
		x--;
	}

	return x;
}

int SMAABlendingWeightCalculationOperation::searchXRight(int x, int y)
{
	int end = x + 2 * m_config.search_steps;
	float e[4];

	while (x <= end) {
		sample(m_imageReader, x + 1, y, e);
		if (e[1] == 0.0f || /* Is the edge not activated? */
		    e[0] != 0.0f)   /* Or is there a crossing edge that breaks the line? */
			break;
		sample(m_imageReader, x + 1, y - 1, e);
		if (e[0] != 0.0f)   /* Or is there a crossing edge that breaks the line? */
			break;
		x++;
	}

	return x;
}

int SMAABlendingWeightCalculationOperation::searchYUp(int x, int y)
{
	int end = y - 2 * m_config.search_steps;
	float e[4];

	while (y >= end) {
		sample(m_imageReader, x, y, e);
		if (e[0] == 0.0f || /* Is the edge not activated? */
		    e[1] != 0.0f)   /* Or is there a crossing edge that breaks the line? */
			break;
		sample(m_imageReader, x - 1, y, e);
		if (e[1] != 0.0f)   /* Or is there a crossing edge that breaks the line? */
			break;
		y--;
	}

	return y;
}

int SMAABlendingWeightCalculationOperation::searchYDown(int x, int y)
{
	int end = y + 2 * m_config.search_steps;
	float e[4];

	while (y <= end) {
		sample(m_imageReader, x, y + 1, e);
		if (e[0] == 0.0f || /* Is the edge not activated? */
		    e[1] != 0.0f)   /* Or is there a crossing edge that breaks the line? */
			break;
		sample(m_imageReader, x - 1, y + 1, e);
		if (e[1] != 0.0f)   /* Or is there a crossing edge that breaks the line? */
			break;
		y++;
	}

	return y;
}

void SMAABlendingWeightCalculationOperation::area(float dist[2], float e1, float e2, float weights[2])
{
	float x = SMAA_AREATEX_MAX_DISTANCE * roundf(4.0f * e1) + dist[0];
	float y = SMAA_AREATEX_MAX_DISTANCE * roundf(4.0f * e2) + dist[1];

	/* We do a bias for mapping to texel space: */
	x += 0.5f;
	y += 0.5f;

	/* Do it! */
	areatex_sample_level_zero(areatex, x, y, weights);
}


/*-----------------------------------------------------------------------------*/
/* Corner Detection Functions */

void SMAABlendingWeightCalculationOperation::detectHorizontalCornerPattern(float weights[2],
									   int left, int right, int y, int d[2])
{
	float factor[2] = {1.0f, 1.0f};
	float rounding = 1.0f - m_config.rounding / 100.0f;
	float e[4];

	/* Reduce blending for pixels in the center of a line. */
	rounding /= (d[0] == d[1]) ? 2.0f : 1.0f;

	/* Near the left corner */
	if (d[0] <= d[1]) {
		sample(m_imageReader, left, y + 1, e);
		factor[0] -= rounding * e[0];
		sample(m_imageReader, left, y - 2, e);
		factor[1] -= rounding * e[0];
	}
	/* Near the right corner */
	if (d[0] >= d[1]) {
		sample(m_imageReader, right + 1, y + 1, e);
		factor[0] -= rounding * e[0];
		sample(m_imageReader, right + 1, y - 2, e);
		factor[1] -= rounding * e[0];
	}

	weights[0] *= CLAMPIS(factor[0], 0.0f, 1.0f);
	weights[1] *= CLAMPIS(factor[1], 0.0f, 1.0f);
}

void SMAABlendingWeightCalculationOperation::detectVerticalCornerPattern(float weights[2],
									 int x, int top, int bottom, int d[2])
{
	float factor[2] = {1.0f, 1.0f};
	float rounding = 1.0f - m_config.rounding / 100.0f;
	float e[4];

	/* Reduce blending for pixels in the center of a line. */
	rounding /= (d[0] == d[1]) ? 2.0f : 1.0f;

	/* Near the top corner */
	if (d[0] <= d[1]) {
		sample(m_imageReader, x + 1, top, e);
		factor[0] -= rounding * e[1];
		sample(m_imageReader, x - 2, top, e);
		factor[1] -= rounding * e[1];
	}
	/* Near the bottom corner */
	if (d[0] >= d[1]) {
		sample(m_imageReader, x + 1, bottom + 1, e);
		factor[0] -= rounding * e[1];
		sample(m_imageReader, x - 2, bottom + 1, e);
		factor[1] -= rounding * e[1];
	}

	weights[0] *= CLAMPIS(factor[0], 0.0f, 1.0f);
	weights[1] *= CLAMPIS(factor[1], 0.0f, 1.0f);
}

/*-----------------------------------------------------------------------------*/
/* Neighborhood Blending (Third Pass) */
/*-----------------------------------------------------------------------------*/

SMAANeighborhoodBlendingOperation::SMAANeighborhoodBlendingOperation() : NodeOperation()
{
	this->addInputSocket(COM_DT_COLOR); /* image */
	this->addInputSocket(COM_DT_COLOR); /* blend */
	this->addOutputSocket(COM_DT_COLOR);
	this->setComplex(true);
	this->m_image1Reader = NULL;
	this->m_image2Reader = NULL;
}

void *SMAANeighborhoodBlendingOperation::initializeTileData(rcti *rect)
{
	return getInputOperation(0)->initializeTileData(rect);
}

void SMAANeighborhoodBlendingOperation::initExecution()
{
	this->m_image1Reader = this->getInputSocketReader(0);
	this->m_image2Reader = this->getInputSocketReader(1);
}

void SMAANeighborhoodBlendingOperation::executePixel(float output[4], int x, int y, void*/*data*/)
{
	float e[4], left, top, right, bottom;

	/* Fetch the blending weights for current pixel: */
	sample(m_image2Reader, x, y, e);
	left = e[2];
	top  = e[0];
	sample(m_image2Reader, x + 1, y, e);
	right = e[3];
	sample(m_image2Reader, x, y + 1, e);
	bottom = e[1];

	/* Is there any blending weight with a value greater than 0.0? */
	if (right + bottom + left + top < 1e-5f) {
		sample(m_image1Reader, x, y, output);
		return;
	}

	/* Calculate the blending offsets: */
	void (*samplefunc)(SocketReader *reader, int x, int y, float xoffset, float color[4]);
	float offset1, offset2, weight1, weight2, color1[4], color2[4];

	if (fmaxf(right, left) > fmaxf(bottom, top)) { /* max(horizontal) > max(vertical) */
		samplefunc = sample_level_zero_xoffset;
		offset1 = right;
		offset2 = -left;
		weight1 = right / (right + left);
		weight2 = left / (right + left);
	}
	else {
		samplefunc = sample_level_zero_yoffset;
		offset1 = bottom;
		offset2 = -top;
		weight1 = bottom / (bottom + top);
		weight2 = top / (bottom + top);
	}

	/* We exploit bilinear filtering to mix current pixel with the chosen neighbor: */
	samplefunc(m_image1Reader, x, y, offset1, color1);
	samplefunc(m_image1Reader, x, y, offset2, color2);

	mul_v4_v4fl(output, color1, weight1);
	madd_v4_v4fl(output, color2, weight2);
}

void SMAANeighborhoodBlendingOperation::deinitExecution()
{
	this->m_image1Reader = NULL;
	this->m_image2Reader = NULL;
}

bool SMAANeighborhoodBlendingOperation::determineDependingAreaOfInterest(rcti *input,
									 ReadBufferOperation *readOperation, rcti *output)
{
	rcti newInput;

	newInput.xmax = input->xmax + 1;
	newInput.xmin = input->xmin - 1;
	newInput.ymax = input->ymax + 1;
	newInput.ymin = input->ymin - 1;

	return NodeOperation::determineDependingAreaOfInterest(&newInput, readOperation, output);
}
