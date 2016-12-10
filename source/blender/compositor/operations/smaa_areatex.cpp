/**
 * Copyright (C) 2016 IRIE Shinsuke
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to
 * do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software. As clarification, there
 * is no requirement that the copyright notice and permission be included in
 * binary distributions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <cmath>

/**
 * This program is C++ rewrite of AreaTex.py included in SMAA ditribution.
 *
 *   SMAA in GitHub: https://github.com/iryoku/smaa
 */

/*------------------------------------------------------------------------------*/
/* Type Definitions */

class Int2;
class Dbl2;

class Int2 {
public:
	int x, y;

	Int2() { this->x = this->y = 0; }
	Int2(int x) { this->x = this->y = x; }
	Int2(int x, int y) { this->x = x; this->y = y; }

	operator Dbl2();

	Int2 operator + (Int2 other) { return Int2(x + other.x, y + other.y); }
	Int2 operator * (Int2 other) { return Int2(x * other.x, y * other.y); }
};

class Dbl2 {
public:
	double x, y;

	Dbl2() { this->x = this->y = 0.0; }
	Dbl2(double x) { this->x = this->y = x; }
	Dbl2(double x, double y) { this->x = x; this->y = y; }

	Dbl2 apply(double (* func)(double)) { return Dbl2(func(x), func(y)); }

	operator Int2();

	Dbl2 operator + (Dbl2 other) { return Dbl2(x + other.x, y + other.y); }
	Dbl2 operator - (Dbl2 other) { return Dbl2(x - other.x, y - other.y); }
	Dbl2 operator * (Dbl2 other) { return Dbl2(x * other.x, y * other.y); }
	Dbl2 operator / (Dbl2 other) { return Dbl2(x / other.x, y / other.y); }
	Dbl2 operator += (Dbl2 other) { return Dbl2(x += other.x, y += other.y); }
};

Int2::operator Dbl2() { return Dbl2((double)x, (double)y); }
Dbl2::operator Int2() { return Int2((int)x, (int)y); }

/*------------------------------------------------------------------------------*/
/* Data to Calculate Areatex */

const double subsample_offsets_ortho[] = {0.0,    /* 0 */
					  -0.25,  /* 1 */
					  0.25,   /* 2 */
					  -0.125, /* 3 */
					  0.125,  /* 4 */
					  -0.375, /* 5 */
					  0.375}; /* 6 */

const Dbl2 subsample_offsets_diag[] = {{ 0.00,   0.00},   /* 0 */
				       { 0.25,  -0.25},   /* 1 */
				       {-0.25,   0.25},   /* 2 */
				       { 0.125, -0.125},  /* 3 */
				       {-0.125,  0.125}}; /* 4 */

/* Texture sizes: */
/* (it's quite possible that this is not easily configurable) */
#define SIZE_ORTHO 16 /* 16 * 5 slots = 80 */
#define SIZE_DIAG  20 /* 20 * 4 slots = 80 */

/* Number of samples for calculating areas in the diagonal textures: */
/* (diagonal areas are calculated using brute force sampling) */
#define SAMPLES_DIAG 30

/* Maximum distance for smoothing u-shapes: */
#define SMOOTH_MAX_DISTANCE 32

/*------------------------------------------------------------------------------*/
/* Miscellaneous Utility Functions */

/* Linear interpolation: */
static Dbl2 lerp(Dbl2 a, Dbl2 b, double p)
{
	return a + (b - a) * Dbl2(p);
}

/* Saturates a value to [0..1] range: */
static double saturate(double x)
{
	return 0.0 < x ? (x < 1.0 ? x : 1.0) : 0.0;
}

/*------------------------------------------------------------------------------*/
/* Mapping Tables (for placing each pattern subtexture into its place) */

const Int2 edgesortho[] = {{0, 0}, {3, 0}, {0, 3}, {3, 3}, {1, 0}, {4, 0}, {1, 3}, {4, 3},
			   {0, 1}, {3, 1}, {0, 4}, {3, 4}, {1, 1}, {4, 1}, {1, 4}, {4, 4}};

const Int2 edgesdiag[]  = {{0, 0}, {1, 0}, {0, 2}, {1, 2}, {2, 0}, {3, 0}, {2, 2}, {3, 2},
			   {0, 1}, {1, 1}, {0, 3}, {1, 3}, {2, 1}, {3, 1}, {2, 3}, {3, 3}};

/*------------------------------------------------------------------------------*/
/* Horizontal/Vertical Areas */

/* Smoothing function for small u-patterns: */
static Dbl2 smootharea(double d, Dbl2 a1, Dbl2 a2)
{
	Dbl2 b1 = (a1 * Dbl2(2.0)).apply(sqrt) * Dbl2(0.5);
	Dbl2 b2 = (a2 * Dbl2(2.0)).apply(sqrt) * Dbl2(0.5);;
	double p = saturate(d / (double)SMOOTH_MAX_DISTANCE);
	return lerp(b1, a1, p) + lerp(b2, a2, p);
}

/* Calculates the area under the line p1->p2, for the pixel x..x+1: */
static Dbl2 area(Dbl2 p1, Dbl2 p2, int x)
{
	Dbl2 d = {p2.x - p1.x, p2.y - p1.y};
	double x1 = (double)x;
	double x2 = x + 1.0;
	double y1 = p1.y + d.y * (x1 - p1.x) / d.x;
	double y2 = p1.y + d.y * (x2 - p1.x) / d.x;

	if ((x1 >= p1.x && x1 < p2.x) || (x2 > p1.x && x2 <= p2.x)) { /* inside? */
		if ((copysign(1.0, y1) == copysign(1.0, y2) ||
		     fabs(y1) < 1e-4 || fabs(y2) < 1e-4)) { /* trapezoid? */
			double a = (y1 + y2) / 2.0;
			if (a < 0.0)
				return Dbl2(fabs(a), 0.0);
			else
				return Dbl2(0.0, fabs(a));
		}
		else { /* Then, we got two triangles: */
			double x = -p1.y * d.x / d.y + p1.x, xi;
			double a1 = x > p1.x ? y1 * modf(x, &xi) / 2.0 : 0.0;
			double a2 = x < p2.x ? y2 * (1.0 - modf(x, &xi)) / 2.0 : 0.0;
			double a = fabs(a1) > fabs(a2) ? a1 : -a2;
			if (a < 0.0)
				return Dbl2(fabs(a1), fabs(a2));
			else
				return Dbl2(fabs(a2), fabs(a1));
		}
	}
	else
		return Dbl2(0.0, 0.0);
}

/* Calculates the area for a given pattern and distances to the left and to the */
/* right, biased by an offset: */
static Dbl2 areaortho(int pattern, int left, int right, double offset)
{
	Dbl2 a1, a2;

	/*
	 * o1           |
	 *      .-------´
	 * o2   |
	 *
	 *      <---d--->
	 */
	double d = (double)(left + right + 1);

	double o1 = 0.5 + offset;
	double o2 = 0.5 + offset - 1.0;

	switch (pattern) {
	case 0:
		/*
		 *
		 *    ------
		 *
		 */
		return Dbl2(0.0, 0.0);
		break;

	case 1:
		/*
		 *
		 *   .------
		 *   |
		 *
		 * We only offset L patterns in the crossing edge side, to make it
		 * converge with the unfiltered pattern 0 (we don't want to filter the
		 * pattern 0 to avoid artifacts).
		 */
		if (left <= right)
			return area(Dbl2(0.0, o2), Dbl2(d / 2.0, 0.0), left);
		else
			return Dbl2(0.0, 0.0);
		break;

	case 2:
		/*
		 *
		 *    ------.
		 *          |
		 */
		if (left >= right)
			return area(Dbl2(d / 2.0, 0.0), Dbl2(d, o2), left);
		else
			return Dbl2(0.0, 0.0);
		break;

	case 3:
	{
		/*
		 *
		 *   .------.
		 *   |      |
		 */
		a1 = area(Dbl2(0.0, o2), Dbl2(d / 2.0, 0.0), left);
		a2 = area(Dbl2(d / 2.0, 0.0), Dbl2(d, o2), left);
		return smootharea(d, a1, a2);
		break;
	}

	case 4:
		/*
		 *   |
		 *   `------
		 *
		 */
		if (left <= right)
			return area(Dbl2(0.0, o1), Dbl2(d / 2.0, 0.0), left);
		else
			return Dbl2(0.0, 0.0);
		break;

	case 5:
		/*
		 *   |
		 *   +------
		 *   |
		 */
		return Dbl2(0.0, 0.0);
		break;

	case 6:
		/*
		 *   |
		 *   `------.
		 *          |
		 *
		 * A problem of not offseting L patterns (see above), is that for certain
		 * max search distances, the pixels in the center of a Z pattern will
		 * detect the full Z pattern, while the pixels in the sides will detect a
		 * L pattern. To avoid discontinuities, we blend the full offsetted Z
		 * revectorization with partially offsetted L patterns.
		 */
		if (fabs(offset) > 0.0) {
			a1 = area(Dbl2(0.0, o1), Dbl2(d, o2), left);
			a2 = area(Dbl2(0.0, o1), Dbl2(d / 2.0, 0.0), left);
			a2 += area(Dbl2(d / 2.0, 0.0), Dbl2(d, o2), left);
			return (a1 + a2) / Dbl2(2.0);
		}
		else
			return area(Dbl2(0.0, o1), Dbl2(d, o2), left);
		break;

	case 7:
		/*
		 *   |
		 *   +------.
		 *   |      |
		 */
		return area(Dbl2(0.0, o1), Dbl2(d, o2), left);
		break;

	case 8:
		/*
		 *          |
		 *    ------´
		 *
		 */
		if (left >= right)
			return area(Dbl2(d / 2.0, 0.0), Dbl2(d, o1), left);
		else
			return Dbl2(0.0, 0.0);
		break;

	case 9:
		/*
		 *          |
		 *   .------´
		 *   |
		 */
		if (fabs(offset) > 0.0) {
			a1 = area(Dbl2(0.0, o2), Dbl2(d, o1), left);
			a2 = area(Dbl2(0.0, o2), Dbl2(d / 2.0, 0.0), left);
			a2 += area(Dbl2(d / 2.0, 0.0), Dbl2(d, o1), left);
			return (a1 + a2) / Dbl2(2.0);
		}
		else
			return area(Dbl2(0.0, o2), Dbl2(d, o1), left);
		break;

	case 10:
		/*
		 *          |
		 *    ------+
		 *          |
		 */
		return Dbl2(0.0, 0.0);
		break;

	case 11:
		/*
		 *          |
		 *   .------+
		 *   |      |
		 */
		return area(Dbl2(0.0, o2), Dbl2(d, o1), left);
		break;

	case 12:
	{
		/*
		 *   |      |
		 *   `------´
		 *
		 */
		a1 = area(Dbl2(0.0, o1), Dbl2(d / 2.0, 0.0), left);
		a2 = area(Dbl2(d / 2.0, 0.0), Dbl2(d, o1), left);
		return smootharea(d, a1, a2);
		break;
	}

	case 13:
		/*
		 *   |      |
		 *   +------´
		 *   |
		 */
		return area(Dbl2(0.0, o2), Dbl2(d, o1), left);
		break;

	case 14:
		/*
		 *   |      |
		 *   `------+
		 *          |
		 */
		return area(Dbl2(0.0, o1), Dbl2(d, o2), left);
		break;

	case 15:
		/*
		 *   |      |
		 *   +------+
		 *   |      |
		 */
		return Dbl2(0.0, 0.0);
		break;
	}

	return Dbl2(0.0, 0.0);
}

/*------------------------------------------------------------------------------*/
/* Diagonal Areas */

static int inside(Dbl2 p1, Dbl2 p2, Dbl2 p)
{
	if (p1.x != p2.x || p1.y != p2.y) {
		double x = p.x, y = p.y;
		double xm = (p1.x + p2.x) / 2.0, ym = (p1.y + p2.y) / 2.0;
		double a = p2.y - p1.y;
		double b = p1.x - p2.x;
		return (a * (x - xm) + b * (y - ym) > 0) ? 1 : 0;
	}
	else
                return 1;
}

/* Calculates the area under the line p1->p2 for the pixel 'p' using brute */
/* force sampling: */
/* (quick and dirty solution, but it works) */
static double area1(Dbl2 p1, Dbl2 p2, Int2 p)
{
	int a = 0;
	int x, y;
	for (x = 0; x < SAMPLES_DIAG; x++) {
		for (y = 0; y < SAMPLES_DIAG; y++) {
			a += inside(p1, p2, Dbl2((double)p.x + (double)x / (double)(SAMPLES_DIAG - 1),
						 (double)p.y + (double)y / (double)(SAMPLES_DIAG - 1)));
		}
	}
	return (double)a / (double)(SAMPLES_DIAG * SAMPLES_DIAG);
}

/* Calculates the area under the line p1->p2: */
/* (includes the pixel and its opposite) */
static Dbl2 area(int pattern, Dbl2 p1, Dbl2 p2, int left, Dbl2 offset)
{
	Int2 e = edgesdiag[pattern];
	if (e.x > 0)
		p1 += offset;
	if (e.y > 0)
		p2 += offset;
	double a1 = area1(p1, p2, Int2(1, 0) + Int2(left));
	double a2 = area1(p1, p2, Int2(1, 1) + Int2(left));
	return Dbl2(1.0 - a1, a2);
}

/* Calculates the area for a given pattern and distances to the left and to the */
/* right, biased by an offset: */
static Dbl2 areadiag(int pattern, int left, int right, Dbl2 offset)
{
	Dbl2 a1, a2;

	double d = (double)(left + right + 1);

	/*
	 * There is some Black Magic around diagonal area calculations. Unlike
	 * orthogonal patterns, the 'null' pattern (one without crossing edges) must be
	 * filtered, and the ends of both the 'null' and L patterns are not known: L
	 * and U patterns have different endings, and we don't know what is the
	 * adjacent pattern. So, what we do is calculate a blend of both possibilites.
	 */
	switch (pattern) {
	case 0:
		/*
		 *
		 *         .-´
		 *       .-´
		 *     .-´
		 *   .-´
		 *   ´
		 *
		 */
		a1 = area(pattern, Dbl2(1.0, 1.0), Dbl2(1.0, 1.0) + Dbl2(d), left, offset); /* 1st possibility */
		a2 = area(pattern, Dbl2(1.0, 0.0), Dbl2(1.0, 0.0) + Dbl2(d), left, offset); /* 2nd possibility */
		return (a1 + a2) / Dbl2(2.0); /* Blend them */
		break;

	case 1:
		/*
		 *
		 *         .-´
		 *       .-´
		 *     .-´
		 *   .-´
		 *   |
		 *   |
		 */
		a1 = area(pattern, Dbl2(1.0, 0.0), Dbl2(0.0, 0.0) + Dbl2(d), left, offset);
		a2 = area(pattern, Dbl2(1.0, 0.0), Dbl2(1.0, 0.0) + Dbl2(d), left, offset);
		return (a1 + a2) / Dbl2(2.0);
		break;

	case 2:
		/*
		 *
		 *         .----
		 *       .-´
		 *     .-´
		 *   .-´
		 *   ´
		 *
		 */
		a1 = area(pattern, Dbl2(0.0, 0.0), Dbl2(1.0, 0.0) + Dbl2(d), left, offset);
		a2 = area(pattern, Dbl2(1.0, 0.0), Dbl2(1.0, 0.0) + Dbl2(d), left, offset);
		return (a1 + a2) / Dbl2(2.0);
		break;

	case 3:
		/*
		 *
		 *         .----
		 *       .-´
		 *     .-´
		 *   .-´
		 *   |
		 *   |
		 */
		return area(pattern, Dbl2(1.0, 0.0), Dbl2(1.0, 0.0) + Dbl2(d), left, offset);
		break;

	case 4:
		/*
		 *
		 *         .-´
		 *       .-´
		 *     .-´
		 * ----´
		 *
		 *
		 */
		a1 = area(pattern, Dbl2(1.0, 1.0), Dbl2(0.0, 0.0) + Dbl2(d), left, offset);
		a2 = area(pattern, Dbl2(1.0, 1.0), Dbl2(1.0, 0.0) + Dbl2(d), left, offset);
		return (a1 + a2) / Dbl2(2.0);
		break;

	case 5:
		/*
		 *
		 *         .-´
		 *       .-´
		 *     .-´
		 * --.-´
		 *   |
		 *   |
		 */
		a1 = area(pattern, Dbl2(1.0, 1.0), Dbl2(0.0, 0.0) + Dbl2(d), left, offset);
		a2 = area(pattern, Dbl2(1.0, 0.0), Dbl2(1.0, 0.0) + Dbl2(d), left, offset);
		return (a1 + a2) / Dbl2(2.0);
		break;

	case 6:
		/*
		 *
		 *         .----
		 *       .-´
		 *     .-´
		 * ----´
		 *
		 *
		 */
		return area(pattern, Dbl2(1.0, 1.0), Dbl2(1.0, 0.0) + Dbl2(d), left, offset);
		break;

	case 7:
		/*
		 *
		 *         .----
		 *       .-´
		 *     .-´
		 * --.-´
		 *   |
		 *   |
		 */
		a1 = area(pattern, Dbl2(1.0, 1.0), Dbl2(1.0, 0.0) + Dbl2(d), left, offset);
		a2 = area(pattern, Dbl2(1.0, 0.0), Dbl2(1.0, 0.0) + Dbl2(d), left, offset);
		return (a1 + a2) / Dbl2(2.0);
		break;

	case 8:
		/*
		 *         |
		 *         |
		 *       .-´
		 *     .-´
		 *   .-´
		 *   ´
		 *
		 */
		a1 = area(pattern, Dbl2(0.0, 0.0), Dbl2(1.0, 1.0) + Dbl2(d), left, offset);
		a2 = area(pattern, Dbl2(1.0, 0.0), Dbl2(1.0, 1.0) + Dbl2(d), left, offset);
		return (a1 + a2) / Dbl2(2.0);
		break;

	case 9:
		/*
		 *         |
		 *         |
		 *       .-´
		 *     .-´
		 *   .-´
		 *   |
		 *   |
		 */
		return area(pattern, Dbl2(1.0, 0.0), Dbl2(1.0, 1.0) + Dbl2(d), left, offset);
		break;

	case 10:
		/*
		 *         |
		 *         .----
		 *       .-´
		 *     .-´
		 *   .-´
		 *   ´
		 *
		 */
		a1 = area(pattern, Dbl2(0.0, 0.0), Dbl2(1.0, 1.0) + Dbl2(d), left, offset);
		a2 = area(pattern, Dbl2(1.0, 0.0), Dbl2(1.0, 0.0) + Dbl2(d), left, offset);
		return (a1 + a2) / Dbl2(2.0);
		break;

	case 11:
		/*
		 *         |
		 *         .----
		 *       .-´
		 *     .-´
		 *   .-´
		 *   |
		 *   |
		 */
		a1 = area(pattern, Dbl2(1.0, 0.0), Dbl2(1.0, 1.0) + Dbl2(d), left, offset);
		a2 = area(pattern, Dbl2(1.0, 0.0), Dbl2(1.0, 0.0) + Dbl2(d), left, offset);
		return (a1 + a2) / Dbl2(2.0);
		break;

	case 12:
		/*
		 *         |
		 *         |
		 *       .-´
		 *     .-´
		 * ----´
		 *
		 *
		 */
		return area(pattern, Dbl2(1.0, 1.0), Dbl2(1.0, 1.0) + Dbl2(d), left, offset);
		break;

	case 13:
		/*
		 *         |
		 *         |
		 *       .-´
		 *     .-´
		 * --.-´
		 *   |
		 *   |
		 */
		a1 = area(pattern, Dbl2(1.0, 1.0), Dbl2(1.0, 1.0) + Dbl2(d), left, offset);
		a2 = area(pattern, Dbl2(1.0, 0.0), Dbl2(1.0, 1.0) + Dbl2(d), left, offset);
		return (a1 + a2) / Dbl2(2.0);
		break;

	case 14:
		/*
		 *         |
		 *         .----
		 *       .-´
		 *     .-´
		 * ----´
		 *
		 *
		 */
		a1 = area(pattern, Dbl2(1.0, 1.0), Dbl2(1.0, 1.0) + Dbl2(d), left, offset);
		a2 = area(pattern, Dbl2(1.0, 1.0), Dbl2(1.0, 0.0) + Dbl2(d), left, offset);
		return (a1 + a2) / Dbl2(2.0);
		break;

	case 15:
		/*
		 *         |
		 *         .----
		 *       .-´
		 *     .-´
		 * --.-´
		 *   |
		 *   |
		 */
		a1 = area(pattern, Dbl2(1.0, 1.0), Dbl2(1.0, 1.0) + Dbl2(d), left, offset);
		a2 = area(pattern, Dbl2(1.0, 0.0), Dbl2(1.0, 0.0) + Dbl2(d), left, offset);
		return (a1 + a2) / Dbl2(2.0);
		break;
	}

	return Dbl2(0.0, 0.0);
}

/*------------------------------------------------------------------------------*/
/* Main Loops */

/* Buffers to Store AreaTex Data Temporarily */
static double ortho[5 * SIZE_ORTHO][5 * SIZE_ORTHO][2];
static double diag[4 * SIZE_DIAG][4 * SIZE_DIAG][2];

static void areatex_ortho(int offset_index, bool quantize)
{
	double offset = subsample_offsets_ortho[offset_index];
	Int2 pos = {0, (5 * SIZE_ORTHO * offset_index)};
	Int2 e;
	int pattern, left, right;

	for (pattern = 0; pattern < 16; pattern++) {
		e = Int2(SIZE_ORTHO) * edgesortho[pattern];
		for (left = 0; left < SIZE_ORTHO; left++) {
			for (right = 0; right < SIZE_ORTHO; right++) {
				Dbl2 p = areaortho(pattern, left * left, right * right, offset);
				Int2 coords = pos + e + Int2(left, right);

				if (quantize)
					p = (Dbl2)((Int2)(p * Dbl2(255.0))) / Dbl2(255.0);

				ortho[coords.y][coords.x][0] = p.x;
				ortho[coords.y][coords.x][1] = p.y;
			}
		}
	}
	return;
}

static void areatex_diag(int offset_index, bool quantize)
{
	Dbl2 offset = subsample_offsets_diag[offset_index];
	Int2 pos = {0, (4 * SIZE_DIAG * offset_index)};
	Int2 e;
	int pattern, left, right;

	for (pattern = 0; pattern < 16; pattern++) {
		e = Int2(SIZE_DIAG) * edgesdiag[pattern];
		for (left = 0; left < SIZE_DIAG; left++) {
			for (right = 0; right < SIZE_DIAG; right++) {
				Dbl2 p = areadiag(pattern, left, right, offset);
				Int2 coords = pos + e + Int2(left, right);

				if (quantize)
					p = (Dbl2)((Int2)(p * Dbl2(255.0))) / Dbl2(255.0);

				diag[coords.y][coords.x][0] = p.x;
				diag[coords.y][coords.x][1] = p.y;
			}
		}
	}
	return;
}

/*------------------------------------------------------------------------------*/
/* Write Header File to Specified Location on Disk */

static void write_double_array(FILE *fp, const double *ptr, int length, const char *array_name)
{
	fprintf(fp, "static const float %s[%d] = {", array_name, length);

	for (int n = 0; n < length; n++) {
		if (n > 0)
			fprintf(fp, ",");
		fprintf(fp, (n % 8 != 0) ? " " : "\n\t");
		fprintf(fp, "%1.8lf", *(ptr++));
	}

	fprintf(fp, "\n};\n\n");
}

static int generate_header(const char *path)
{
	FILE *fp = fopen(path, "w");
	if (!fp) {
		fprintf(stderr, "Unable to open file: %s\n", path);
		return 1;
	}

	fprintf(stderr, "Generating %s\n", path);

	fprintf(fp, "/* This file was generated by smaa_areatex.cpp */\n\n");

	fprintf(fp, "/* Horizontal/Vertical Areas */\n");
	write_double_array(fp, (double *)ortho,
			   sizeof(ortho) / sizeof(double),
			   "areatex");

	fprintf(fp, "/* Diagonal Areas */\n");
	write_double_array(fp, (double *)diag,
			   sizeof(diag) / sizeof(double),
			   "areatex_diag");

	fclose(fp);

	return 0;
}

int main(int argc, char **argv)
{
	char *outfile = argv[1];
	bool quantize = false;

	if (argc == 3 && strcmp(argv[1], "-q") == 0) {
		outfile = argv[2];
		quantize = true;
	} else if (argc != 2) {
		fprintf(stderr, "Usage: %s [OPTION] OUTFILE\n", argv[0]);
		fprintf(stderr, "Option: -q Quantize data to 256 levels\n");
		return 1;
	}

	/* Calculate areatex data */
	/* SMAA 1x uses offset index 0 only */
	areatex_ortho(0, quantize);
	areatex_diag(0, quantize);

	/* Generate C++ source file */
	return generate_header(outfile);
}
