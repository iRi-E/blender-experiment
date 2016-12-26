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
#include "IMB_colormanagement.h"

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

static inline void sample(SocketReader *reader, int x, int y, float color[4])
{
	reader->read(color, CLAMPIS(x, 0, reader->getWidth() - 1), CLAMPIS(y, 0, reader->getHeight() -1), NULL);
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

static inline const float* areatex_sample_internal(const float *areatex, int x, int y)
{
	return &areatex[(CLAMPIS(x, 0, SMAA_AREATEX_SIZE - 1) +
			 CLAMPIS(y, 0, SMAA_AREATEX_SIZE - 1) * SMAA_AREATEX_SIZE) * 2];
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

	/* Calculate the threshold: */
	if (m_config.pred)
		calculatePredicatedThreshold(x, y, threshold);
	else
		threshold[0] = threshold[1] = m_config.thresh;

	/* Calculate luma deltas: */
	sample(m_imageReader, x, y, color);
	L     = IMB_colormanagement_get_luminance(color);
	sample(m_imageReader, x - 1, y, color);
	Lleft = IMB_colormanagement_get_luminance(color);
	sample(m_imageReader, x, y - 1, color);
	Ltop  = IMB_colormanagement_get_luminance(color);
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
	Lright  = IMB_colormanagement_get_luminance(color);
	sample(m_imageReader, x, y + 1, color);
	Lbottom = IMB_colormanagement_get_luminance(color);
	Dright  = fabsf(L - Lright);
	Dbottom = fabsf(L - Lbottom);

	/* Calculate the maximum delta in the direct neighborhood: */
	delta_x = fmaxf(Dleft, Dright);
	delta_y = fmaxf(Dtop, Dbottom);

	/* Calculate left-left and top-top deltas: */
	sample(m_imageReader, x - 2, y, color);
	Lleftleft = IMB_colormanagement_get_luminance(color);
	sample(m_imageReader, x, y - 2, color);
	Ltoptop   = IMB_colormanagement_get_luminance(color);
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

static float color_delta(const float color1[4], const float color2[4])
{
	return fmaxf(fmaxf(fabsf(color1[0] - color2[0]), fabsf(color1[1] - color2[1])), fabsf(color1[2] - color2[2]));
}

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
	Dleft = color_delta(C, Cleft);
	Dtop  = color_delta(C, Ctop);

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
	Dright  = color_delta(C, Cright);
	Dbottom = color_delta(C, Cbottom);

	/* Calculate the maximum delta in the direct neighborhood: */
	delta_x = fmaxf(Dleft, Dright);
	delta_y = fmaxf(Dtop, Dbottom);

	/* Calculate left-left and top-top deltas: */
	sample(m_imageReader, x - 2, y, Cleftleft);
	sample(m_imageReader, x, y - 2, Ctoptop);
	Dleftleft = color_delta(Cleft, Cleftleft);
	Dtoptop   = color_delta(Ctop, Ctoptop);

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
	float edges[4], c[4];

	zero_v4(output);
	sample(m_imageReader, x, y, edges);

	/* Edge at north */
	if (edges[1] > 0.0f) {
		if (m_config.diag) {
			/* Diagonals have both north and west edges, so calculating weights for them */
			/* in one of the boundaries is enough. */
			calculateDiagWeights(x, y, edges, output);

			/* We give priority to diagonals, so if we find a diagonal we skip  */
			/* horizontal/vertical processing. */
			if (!is_zero_v2(output))
				return;
		}

		/* Find the distance to the left and the right: */
		int left = searchXLeft(x, y);
		int right = searchXRight(x, y);
		int d1 = x - left, d2 = right - x;

		/* Fetch the left and right crossing edges: */
		int e1 = 0, e2 = 0;
		sample(m_imageReader, left, y - 1, c);
		if (c[0] > 0.0)
			e1 += 1;
		sample(m_imageReader, left, y, c);
		if (c[0] > 0.0)
			e1 += 3;
		sample(m_imageReader, right + 1, y - 1, c);
		if (c[0] > 0.0)
			e2 += 1;
		sample(m_imageReader, right + 1, y, c);
		if (c[0] > 0.0)
			e2 += 3;

		/* Ok, we know how this pattern looks like, now it is time for getting */
		/* the actual area: */
		area(d1, d2, e1, e2, output); /* R, G */

		/* Fix corners: */
		if (m_config.corner)
			detectHorizontalCornerPattern(output, left, right, y, d1, d2);
	}

	/* Edge at west */
	if (edges[0] > 0.0f) {
		/* Did we already do diagonal search for this west edge from the left neighboring pixel? */
		if (m_config.diag && isVerticalSearchUnneeded(x, y))
			return;

		/* Find the distance to the top and the bottom: */
		int top = searchYUp(x, y);
		int bottom = searchYDown(x, y);
		int d1 = y - top, d2 = bottom - y;

		/* Fetch the top ang bottom crossing edges: */
		int e1 = 0, e2 = 0;
		sample(m_imageReader, x - 1, top, c);
		if (c[1] > 0.0)
			e1 += 1;
		sample(m_imageReader, x, top, c);
		if (c[1] > 0.0)
			e1 += 3;
		sample(m_imageReader, x - 1, bottom + 1, c);
		if (c[1] > 0.0)
			e2 += 1;
		sample(m_imageReader, x, bottom + 1, c);
		if (c[1] > 0.0)
			e2 += 3;

		/* Get the area for this direction: */
		area(d1, d2, e1, e2, output + 2); /* B, A */

		/* Fix corners: */
		if (m_config.corner)
			detectVerticalCornerPattern(output + 2, x, top, bottom, d1, d2);
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

	newInput.xmax = input->xmax + max(m_config.search_steps,
					  m_config.diag ? m_config.search_steps_diag + 1: 0);
	newInput.xmin = input->xmin - max(max(m_config.search_steps - 1, 1),
					  m_config.diag ? m_config.search_steps_diag + 1: 0);
	newInput.ymax = input->ymax + max(m_config.search_steps,
					  m_config.diag ? m_config.search_steps_diag : 0);
	newInput.ymin = input->ymin - max(max(m_config.search_steps - 1, 1),
					  m_config.diag ? m_config.search_steps_diag : 0);

	return NodeOperation::determineDependingAreaOfInterest(&newInput, readOperation, output);
}

/*-----------------------------------------------------------------------------*/
/* Diagonal Search Functions */

/**
 * These functions allows to perform diagonal pattern searches.
 */
int SMAABlendingWeightCalculationOperation::searchDiag1(int x, int y, int dir, bool *found)
{
	float e[4];
	int end = x + m_config.search_steps_diag * dir;
	*found = false;

	while (x != end) {
		x += dir;
		y -= dir;
		sample(m_imageReader, x, y, e);
		if (e[1] == 0.0f) {
			*found = true;
			break;
		}
		if (e[0] == 0.0f) {
			*found = true;
			return (dir < 0) ? x : x - dir;
		}
	}

	return x - dir;
}

int SMAABlendingWeightCalculationOperation::searchDiag2(int x, int y, int dir, bool *found)
{
	float e[4];
	int end = x + m_config.search_steps_diag * dir;
	*found = false;

	while (x != end) {
		x += dir;
		y += dir;
		sample(m_imageReader, x, y, e);
		if (e[1] == 0.0f) {
			*found = true;
			break;
		}
		sample(m_imageReader, x + 1, y, e);
		if (e[0] == 0.0f) {
			*found = true;
			return (dir > 0) ? x : x - dir;
		}
	}

	return x - dir;
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
void SMAABlendingWeightCalculationOperation::calculateDiagWeights(int x, int y, const float edges[2], float weights[2])
{
	int d1, d2;
	bool d1_found, d2_found;
	float e[4], c[4];

	zero_v2(weights);

	/* Search for the line ends: */
	if (edges[0] > 0.0f) {
		d1 = x - searchDiag1(x, y, -1, &d1_found);
	}
	else {
		d1 = 0;
		d1_found = true;
	}
	d2 = searchDiag1(x, y, 1, &d2_found) - x;

	if (d1 + d2 > 2) { /* d1 + d2 + 1 > 3 */
		int e1 = 0, e2 = 0;

		if (d1_found) {
			/* Fetch the crossing edges: */
			int coords_x = x - d1, coords_y = y + d1;

			sample(m_imageReader, coords_x - 1, coords_y, c);
			if (c[1] > 0.0)
				e1 += 2;
			sample(m_imageReader, coords_x, coords_y, c);
			if (c[0] > 0.0)
				e1 += 1;
		}

		if (d2_found) {
			/* Fetch the crossing edges: */
			int coords_x = x + d2, coords_y = y - d2;

			sample(m_imageReader, coords_x + 1, coords_y, c);
			if (c[1] > 0.0)
				e2 += 2;
			sample(m_imageReader, coords_x + 1, coords_y - 1, c);
			if (c[0] > 0.0)
				e2 += 1;
		}

		/* Fetch the areas for this line: */
		areaDiag(d1, d2, e1, e2, weights);
	}

	/* Search for the line ends: */
	d1 = x - searchDiag2(x, y, -1, &d1_found);
	sample(m_imageReader, x + 1, y, e);
	if (e[0] > 0.0f) {
		d2 = searchDiag2(x, y, 1, &d2_found) - x;
	}
	else {
		d2 = 0;
		d2_found = true;
	}

	if (d1 + d2 > 2) { /* d1 + d2 + 1 > 3 */
		int e1 = 0, e2 = 0;

		if (d1_found) {
			/* Fetch the crossing edges: */
			int coords_x = x - d1, coords_y = y - d1;

			sample(m_imageReader, coords_x - 1, coords_y, c);
			if (c[1] > 0.0)
				e1 += 2;
			sample(m_imageReader, coords_x, coords_y - 1, c);
			if (c[0] > 0.0)
				e1 += 1;
		}

		if (d2_found) {
			/* Fetch the crossing edges: */
			int coords_x = x + d2, coords_y = y + d2;

			sample(m_imageReader, coords_x + 1, coords_y, c);
			if (c[1] > 0.0)
				e2 += 2;
			if (c[0] > 0.0)
				e2 += 1;
		}

		/* Fetch the areas for this line: */
		float w[2];
		areaDiag(d1, d2, e1, e2, w);
		weights[0] += w[1];
		weights[1] += w[0];
	}
}

bool SMAABlendingWeightCalculationOperation::isVerticalSearchUnneeded(int x, int y)
{
	int d1, d2;
	bool found;
	float e[4];

	/* Search for the line ends: */
	sample(m_imageReader, x - 1, y, e);
	if (e[1] > 0.0f)
		d1 = x - searchDiag2(x - 1, y, -1, &found);
	else
		d1 = 0;
	d2 = searchDiag2(x - 1, y, 1, &found) - x;

	return (d1 + d2 > 2); /* d1 + d2 + 1 > 3 */
}


/*-----------------------------------------------------------------------------*/
/* Horizontal/Vertical Search Functions */

int SMAABlendingWeightCalculationOperation::searchXLeft(int x, int y)
{
	int end = x - m_config.search_steps;
	float e[4];

	while (x > end) {
		sample(m_imageReader, x, y, e);
		if (e[1] == 0.0f)   /* Is the edge not activated? */
			break;
		if (e[0] != 0.0f)   /* Or is there a crossing edge that breaks the line? */
			return x;
		sample(m_imageReader, x, y - 1, e);
		if (e[0] != 0.0f)   /* Or is there a crossing edge that breaks the line? */
			return x;
		x--;
	}

	return x + 1;
}

int SMAABlendingWeightCalculationOperation::searchXRight(int x, int y)
{
	int end = x + m_config.search_steps;
	float e[4];

	while (x < end) {
		x++;
		sample(m_imageReader, x, y, e);
		if (e[1] == 0.0f || /* Is the edge not activated? */
		    e[0] != 0.0f)   /* Or is there a crossing edge that breaks the line? */
			break;
		sample(m_imageReader, x, y - 1, e);
		if (e[0] != 0.0f)   /* Or is there a crossing edge that breaks the line? */
			break;
	}

	return x - 1;
}

int SMAABlendingWeightCalculationOperation::searchYUp(int x, int y)
{
	int end = y - m_config.search_steps;
	float e[4];

	while (y > end) {
		sample(m_imageReader, x, y, e);
		if (e[0] == 0.0f)   /* Is the edge not activated? */
			break;
		if (e[1] != 0.0f)   /* Or is there a crossing edge that breaks the line? */
			return y;
		sample(m_imageReader, x - 1, y, e);
		if (e[1] != 0.0f)   /* Or is there a crossing edge that breaks the line? */
			return y;
		y--;
	}

	return y + 1;
}

int SMAABlendingWeightCalculationOperation::searchYDown(int x, int y)
{
	int end = y + m_config.search_steps;
	float e[4];

	while (y < end) {
		y++;
		sample(m_imageReader, x, y, e);
		if (e[0] == 0.0f || /* Is the edge not activated? */
		    e[1] != 0.0f)   /* Or is there a crossing edge that breaks the line? */
			break;
		sample(m_imageReader, x - 1, y, e);
		if (e[1] != 0.0f)   /* Or is there a crossing edge that breaks the line? */
			break;
	}

	return y - 1;
}

void SMAABlendingWeightCalculationOperation::area(int d1, int d2, int e1, int e2, float weights[2])
{
	/* The areas texture is compressed  quadratically: */
	float x = (float)(SMAA_AREATEX_MAX_DISTANCE * e1) + sqrtf((float)d1);
	float y = (float)(SMAA_AREATEX_MAX_DISTANCE * e2) + sqrtf((float)d2);

	/* We do a bias for mapping to texel space: */
	x += 0.5f;
	y += 0.5f;

	/* Do it! */
	areatex_sample_level_zero(areatex, x, y, weights);
}


/*-----------------------------------------------------------------------------*/
/* Corner Detection Functions */

void SMAABlendingWeightCalculationOperation::detectHorizontalCornerPattern(float weights[2],
									   int left, int right, int y, int d1, int d2)
{
	float factor[2] = {1.0f, 1.0f};
	float rounding = 1.0f - m_config.rounding / 100.0f;
	float e[4];

	/* Reduce blending for pixels in the center of a line. */
	rounding *= (d1 == d2) ? 0.5f : 1.0f;

	/* Near the left corner */
	if (d1 <= d2) {
		sample(m_imageReader, left, y + 1, e);
		factor[0] -= rounding * e[0];
		sample(m_imageReader, left, y - 2, e);
		factor[1] -= rounding * e[0];
	}
	/* Near the right corner */
	if (d1 >= d2) {
		sample(m_imageReader, right + 1, y + 1, e);
		factor[0] -= rounding * e[0];
		sample(m_imageReader, right + 1, y - 2, e);
		factor[1] -= rounding * e[0];
	}

	weights[0] *= CLAMPIS(factor[0], 0.0f, 1.0f);
	weights[1] *= CLAMPIS(factor[1], 0.0f, 1.0f);
}

void SMAABlendingWeightCalculationOperation::detectVerticalCornerPattern(float weights[2],
									 int x, int top, int bottom, int d1, int d2)
{
	float factor[2] = {1.0f, 1.0f};
	float rounding = 1.0f - m_config.rounding / 100.0f;
	float e[4];

	/* Reduce blending for pixels in the center of a line. */
	rounding *= (d1 == d2) ? 0.5f : 1.0f;

	/* Near the top corner */
	if (d1 <= d2) {
		sample(m_imageReader, x + 1, top, e);
		factor[0] -= rounding * e[1];
		sample(m_imageReader, x - 2, top, e);
		factor[1] -= rounding * e[1];
	}
	/* Near the bottom corner */
	if (d1 >= d2) {
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
