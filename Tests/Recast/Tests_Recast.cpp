#include <stdio.h>
#include <string.h>
#include <limits>

#include "catch2/catch_amalgamated.hpp"

#include "Recast.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourAlloc.h"

TEST_CASE("rcSwap", "[recast]")
{
	SECTION("Swap two values")
	{
		int one = 1;
		int two = 2;
		rcSwap(one, two);
		REQUIRE(one == 2);
		REQUIRE(two == 1);
	}
}

TEST_CASE("rcMin", "[recast]")
{
	SECTION("Min returns the lowest value.")
	{
		REQUIRE(rcMin(1, 2) == 1);
		REQUIRE(rcMin(2, 1) == 1);
	}

	SECTION("Min with equal args")
	{
		REQUIRE(rcMin(1, 1) == 1);
	}
}

TEST_CASE("rcMax", "[recast]")
{
	SECTION("Max returns the greatest value.")
	{
		REQUIRE(rcMax(1, 2) == 2);
		REQUIRE(rcMax(2, 1) == 2);
	}

	SECTION("Max with equal args")
	{
		REQUIRE(rcMax(1, 1) == 1);
	}
}

TEST_CASE("rcAbs", "[recast]")
{
	SECTION("Abs returns the absolute value.")
	{
		REQUIRE(rcAbs(-1) == 1);
		REQUIRE(rcAbs(1) == 1);
		REQUIRE(rcAbs(0) == 0);
	}
}

TEST_CASE("rcSqr", "[recast]")
{
	SECTION("Sqr squares a number")
	{
		REQUIRE(rcSqr(2) == 4);
		REQUIRE(rcSqr(-4) == 16);
		REQUIRE(rcSqr(0) == 0);
	}
}

TEST_CASE("rcClamp", "[recast]")
{
	SECTION("Higher than range")
	{
		REQUIRE(rcClamp(2, 0, 1) == 1);
	}

	SECTION("Within range")
	{
		REQUIRE(rcClamp(1, 0, 2) == 1);
	}

	SECTION("Lower than range")
	{
		REQUIRE(rcClamp(0, 1, 2) == 1);
	}
}

TEST_CASE("rcSqrt", "[recast]")
{
	SECTION("Sqrt gets the sqrt of a number")
	{
		REQUIRE(rcSqrt(4) == Catch::Approx(2));
		REQUIRE(rcSqrt(81) == Catch::Approx(9));
	}
}

TEST_CASE("rcVcross", "[recast]")
{
	SECTION("Computes cross product")
	{
		float v1[3] = {3, -3, 1};
		float v2[3] = {4, 9, 2};
		float result[3];
		rcVcross(result, v1, v2);
		REQUIRE(result[0] == Catch::Approx(-15));
		REQUIRE(result[1] == Catch::Approx(-2));
		REQUIRE(result[2] == Catch::Approx(39));
	}

	SECTION("Cross product with itself is zero")
	{
		float v1[3] = {3, -3, 1};
		float result[3];
		rcVcross(result, v1, v1);
		REQUIRE(result[0] == Catch::Approx(0));
		REQUIRE(result[1] == Catch::Approx(0));
		REQUIRE(result[2] == Catch::Approx(0));
	}
}

TEST_CASE("rcVdot", "[recast]")
{
	SECTION("Dot normalized vector with itself")
	{
		float v1[] = { 1, 0, 0 };
		float result = rcVdot(v1, v1);
		REQUIRE(result == Catch::Approx(1));
	}

	SECTION("Dot zero vector with anything is zero")
	{
		float v1[] = { 1, 2, 3 };
		float v2[] = { 0, 0, 0 };

		float result = rcVdot(v1, v2);
		REQUIRE(result == Catch::Approx(0));
	}
}

TEST_CASE("rcVmad", "[recast]")
{
	SECTION("scaled add two vectors")
	{
		float v1[3] = {1, 2, 3};
		float v2[3] = {0, 2, 4};
		float result[3];
		rcVmad(result, v1, v2, 2);
		REQUIRE(result[0] == Catch::Approx(1));
		REQUIRE(result[1] == Catch::Approx(6));
		REQUIRE(result[2] == Catch::Approx(11));
	}

	SECTION("second vector is scaled, first is not")
	{
		float v1[3] = {1, 2, 3};
		float v2[3] = {5, 6, 7};
		float result[3];
		rcVmad(result, v1, v2, 0);
		REQUIRE(result[0] == Catch::Approx(1));
		REQUIRE(result[1] == Catch::Approx(2));
		REQUIRE(result[2] == Catch::Approx(3));
	}
}

TEST_CASE("rcVadd", "[recast]")
{
	SECTION("add two vectors")
	{
		float v1[3] = {1, 2, 3};
		float v2[3] = {5, 6, 7};
		float result[3];
		rcVadd(result, v1, v2);
		REQUIRE(result[0] == Catch::Approx(6));
		REQUIRE(result[1] == Catch::Approx(8));
		REQUIRE(result[2] == Catch::Approx(10));
	}
}

TEST_CASE("rcVsub", "[recast]")
{
	SECTION("subtract two vectors")
	{
		float v1[3] = {5, 4, 3};
		float v2[3] = {1, 2, 3};
		float result[3];
		rcVsub(result, v1, v2);
		REQUIRE(result[0] == Catch::Approx(4));
		REQUIRE(result[1] == Catch::Approx(2));
		REQUIRE(result[2] == Catch::Approx(0));
	}
}

TEST_CASE("rcVmin", "[recast]")
{
	SECTION("selects the min component from the vectors")
	{
		float v1[3] = {5, 4, 0};
		float v2[3] = {1, 2, 9};
		rcVmin(v1, v2);
		REQUIRE(v1[0] == Catch::Approx(1));
		REQUIRE(v1[1] == Catch::Approx(2));
		REQUIRE(v1[2] == Catch::Approx(0));
	}

	SECTION("v1 is min")
	{
		float v1[3] = {1, 2, 3};
		float v2[3] = {4, 5, 6};
		rcVmin(v1, v2);
		REQUIRE(v1[0] == Catch::Approx(1));
		REQUIRE(v1[1] == Catch::Approx(2));
		REQUIRE(v1[2] == Catch::Approx(3));
	}

	SECTION("v2 is min")
	{
		float v1[3] = {4, 5, 6};
		float v2[3] = {1, 2, 3};
		rcVmin(v1, v2);
		REQUIRE(v1[0] == Catch::Approx(1));
		REQUIRE(v1[1] == Catch::Approx(2));
		REQUIRE(v1[2] == Catch::Approx(3));
	}
}

TEST_CASE("rcVmax", "[recast]")
{
	SECTION("selects the max component from the vectors")
	{
		float v1[3] = {5, 4, 0};
		float v2[3] = {1, 2, 9};
		rcVmax(v1, v2);
		REQUIRE(v1[0] == Catch::Approx(5));
		REQUIRE(v1[1] == Catch::Approx(4));
		REQUIRE(v1[2] == Catch::Approx(9));
	}

	SECTION("v2 is max")
	{
		float v1[3] = {1, 2, 3};
		float v2[3] = {4, 5, 6};
		rcVmax(v1, v2);
		REQUIRE(v1[0] == Catch::Approx(4));
		REQUIRE(v1[1] == Catch::Approx(5));
		REQUIRE(v1[2] == Catch::Approx(6));
	}

	SECTION("v1 is max")
	{
		float v1[3] = {4, 5, 6};
		float v2[3] = {1, 2, 3};
		rcVmax(v1, v2);
		REQUIRE(v1[0] == Catch::Approx(4));
		REQUIRE(v1[1] == Catch::Approx(5));
		REQUIRE(v1[2] == Catch::Approx(6));
	}
}

TEST_CASE("rcVcopy", "[recast]")
{
	SECTION("copies a vector into another vector")
	{
		float v1[3] = {5, 4, 0};
		float result[3] = {1, 2, 9};
		rcVcopy(result, v1);
		REQUIRE(result[0] == Catch::Approx(5));
		REQUIRE(result[1] == Catch::Approx(4));
		REQUIRE(result[2] == Catch::Approx(0));
		REQUIRE(v1[0] == Catch::Approx(5));
		REQUIRE(v1[1] == Catch::Approx(4));
		REQUIRE(v1[2] == Catch::Approx(0));
	}
}

TEST_CASE("rcVdist", "[recast]")
{
	SECTION("distance between two vectors")
	{
		float v1[3] = {3, 1, 3};
		float v2[3] = {1, 3, 1};
		float result = rcVdist(v1, v2);

		REQUIRE(result == Catch::Approx(3.4641f));
	}

	SECTION("Distance from zero is magnitude")
	{
		float v1[3] = {3, 1, 3};
		float v2[3] = {0, 0, 0};
		float distance = rcVdist(v1, v2);
		float magnitude = rcSqrt(rcSqr(v1[0]) + rcSqr(v1[1]) + rcSqr(v1[2]));
		REQUIRE(distance == Catch::Approx(magnitude));
	}
}

TEST_CASE("rcVdistSqr", "[recast]")
{
	SECTION("squared distance between two vectors")
	{
		float v1[3] = {3, 1, 3};
		float v2[3] = {1, 3, 1};
		float result = rcVdistSqr(v1, v2);

		REQUIRE(result == Catch::Approx(12));
	}

	SECTION("squared distance from zero is squared magnitude")
	{
		float v1[3] = {3, 1, 3};
		float v2[3] = {0, 0, 0};
		float distance = rcVdistSqr(v1, v2);
		float magnitude = rcSqr(v1[0]) + rcSqr(v1[1]) + rcSqr(v1[2]);
		REQUIRE(distance == Catch::Approx(magnitude));
	}
}

TEST_CASE("rcVnormalize", "[recast]")
{
	SECTION("normalizing reduces magnitude to 1")
	{
		float v[3] = {3, 3, 3};
		rcVnormalize(v);
		REQUIRE(v[0] == Catch::Approx(rcSqrt(1.0f / 3.0f)));
		REQUIRE(v[1] == Catch::Approx(rcSqrt(1.0f / 3.0f)));
		REQUIRE(v[2] == Catch::Approx(rcSqrt(1.0f / 3.0f)));
		float magnitude = rcSqrt(rcSqr(v[0]) + rcSqr(v[1]) + rcSqr(v[2]));
		REQUIRE(magnitude == Catch::Approx(1));
	}
}

TEST_CASE("rcCalcBounds", "[recast]")
{
	SECTION("bounds of one vector")
	{
		float verts[] = {1, 2, 3};
		float bmin[3];
		float bmax[3];
		rcCalcBounds(verts, 1, bmin, bmax);

		REQUIRE(bmin[0] == Catch::Approx(verts[0]));
		REQUIRE(bmin[1] == Catch::Approx(verts[1]));
		REQUIRE(bmin[2] == Catch::Approx(verts[2]));

		REQUIRE(bmax[0] == Catch::Approx(verts[0]));
		REQUIRE(bmax[1] == Catch::Approx(verts[1]));
		REQUIRE(bmax[2] == Catch::Approx(verts[2]));
	}

	SECTION("bounds of more than one vector")
	{
		float verts[] = {
			1, 2, 3,
			0, 2, 5
		};
		float bmin[3];
		float bmax[3];
		rcCalcBounds(verts, 2, bmin, bmax);

		REQUIRE(bmin[0] == Catch::Approx(0));
		REQUIRE(bmin[1] == Catch::Approx(2));
		REQUIRE(bmin[2] == Catch::Approx(3));

		REQUIRE(bmax[0] == Catch::Approx(1));
		REQUIRE(bmax[1] == Catch::Approx(2));
		REQUIRE(bmax[2] == Catch::Approx(5));
	}
}

TEST_CASE("rcCalcGridSize", "[recast]")
{
	SECTION("computes the size of an x & z axis grid")
	{
		float verts[] = {
			1, 2, 3,
			0, 2, 6
		};
		float bmin[3];
		float bmax[3];
		rcCalcBounds(verts, 2, bmin, bmax);

		float cellSize = 1.5f;

		int width;
		int height;

		rcCalcGridSize(bmin, bmax, cellSize, &width, &height);

		REQUIRE(width == 1);
		REQUIRE(height == 2);
	}
}

TEST_CASE("rcCreateHeightfield", "[recast]")
{
	SECTION("create a heightfield")
	{
		float verts[] = {
			1, 2, 3,
			0, 2, 6
		};
		float bmin[3];
		float bmax[3];
		rcCalcBounds(verts, 2, bmin, bmax);

		float cellSize = 1.5f;
		float cellHeight = 2;

		int width;
		int height;

		rcCalcGridSize(bmin, bmax, cellSize, &width, &height);

		rcHeightfield heightfield;

		bool result = rcCreateHeightfield(0, heightfield, width, height, bmin, bmax, cellSize, cellHeight);

		REQUIRE(result);

		REQUIRE(heightfield.width == width);
		REQUIRE(heightfield.height == height);

		REQUIRE(heightfield.bmin[0] == Catch::Approx(bmin[0]));
		REQUIRE(heightfield.bmin[1] == Catch::Approx(bmin[1]));
		REQUIRE(heightfield.bmin[2] == Catch::Approx(bmin[2]));

		REQUIRE(heightfield.bmax[0] == Catch::Approx(bmax[0]));
		REQUIRE(heightfield.bmax[1] == Catch::Approx(bmax[1]));
		REQUIRE(heightfield.bmax[2] == Catch::Approx(bmax[2]));

		REQUIRE(heightfield.cs == Catch::Approx(cellSize));
		REQUIRE(heightfield.ch == Catch::Approx(cellHeight));

		REQUIRE(heightfield.spans != 0);
		REQUIRE(heightfield.pools == 0);
		REQUIRE(heightfield.freelist == 0);
	}
}

TEST_CASE("rcMarkWalkableTriangles", "[recast]")
{
	rcContext* ctx = 0;
	float walkableSlopeAngle = 45;
	float verts[] = {
		0, 0, 0,
		1, 0, 0,
		0, 0, -1
	};
	int nv = 3;
	int walkable_tri[] = { 0, 1, 2 };
	int unwalkable_tri[] = { 0, 2, 1 };
	int nt = 1;
	unsigned char areas[] = { RC_NULL_AREA };

	SECTION("One walkable triangle")
	{
		rcMarkWalkableTriangles(ctx, walkableSlopeAngle, verts, nv, walkable_tri, nt, areas);
		REQUIRE(areas[0] == RC_WALKABLE_AREA);
	}

	SECTION("One non-walkable triangle")
	{
		rcMarkWalkableTriangles(ctx, walkableSlopeAngle, verts, nv, unwalkable_tri, nt, areas);
		REQUIRE(areas[0] == RC_NULL_AREA);
	}

	SECTION("Non-walkable triangle area id's are not modified")
	{
		areas[0] = 42;
		rcMarkWalkableTriangles(ctx, walkableSlopeAngle, verts, nv, unwalkable_tri, nt, areas);
		REQUIRE(areas[0] == 42);
	}

	SECTION("Slopes equal to the max slope are considered unwalkable.")
	{
		walkableSlopeAngle = 0;
		rcMarkWalkableTriangles(ctx, walkableSlopeAngle, verts, nv, walkable_tri, nt, areas);
		REQUIRE(areas[0] == RC_NULL_AREA);
	}
}

TEST_CASE("rcClearUnwalkableTriangles", "[recast]")
{
	rcContext* ctx = 0;
	float walkableSlopeAngle = 45;
	float verts[] = {
		0, 0, 0,
		1, 0, 0,
		0, 0, -1
	};
	int nv = 3;
	int walkable_tri[] = { 0, 1, 2 };
	int unwalkable_tri[] = { 0, 2, 1 };
	int nt = 1;
	unsigned char areas[] = { 42 };

	SECTION("Sets area ID of unwalkable triangle to RC_NULL_AREA")
	{
		rcClearUnwalkableTriangles(ctx, walkableSlopeAngle, verts, nv, unwalkable_tri, nt, areas);
		REQUIRE(areas[0] == RC_NULL_AREA);
	}

	SECTION("Does not modify walkable triangle aread ID's")
	{
		rcClearUnwalkableTriangles(ctx, walkableSlopeAngle, verts, nv, walkable_tri, nt, areas);
		REQUIRE(areas[0] == 42);
	}

	SECTION("Slopes equal to the max slope are considered unwalkable.")
	{
		walkableSlopeAngle = 0;
		rcClearUnwalkableTriangles(ctx, walkableSlopeAngle, verts, nv, walkable_tri, nt, areas);
		REQUIRE(areas[0] == RC_NULL_AREA);
	}
}

TEST_CASE("rcRasterizeTriangles", "[recast]")
{
	rcContext ctx;
	float verts[] = {
		0, 0, 0,
		1, 0, 0,
		0, 0, -1,
		0, 0, 1
	};
	int tris[] = {
		0, 1, 2,
		0, 3, 1
	};
	unsigned char areas[] = {
		1,
		2
	};
	float bmin[3];
	float bmax[3];
	rcCalcBounds(verts, 4, bmin, bmax);

	float cellSize = .5f;
	float cellHeight = .5f;

	int width;
	int height;

	rcCalcGridSize(bmin, bmax, cellSize, &width, &height);

	rcHeightfield solid; 
	REQUIRE(rcCreateHeightfield(&ctx, solid, width, height, bmin, bmax, cellSize, cellHeight));

	int flagMergeThr = 1;

	SECTION("Rasterize some triangles")
	{
		REQUIRE(rcRasterizeTriangles(&ctx, verts, 4, tris, areas, 2, solid, flagMergeThr));

		REQUIRE(solid.spans[0 + 0 * width]);
		REQUIRE(solid.spans[0 + 1 * width]);
		REQUIRE(solid.spans[0 + 2 * width]);
		REQUIRE(solid.spans[0 + 3 * width]);
		REQUIRE(!solid.spans[1 + 0 * width]);
		REQUIRE(solid.spans[1 + 1 * width]);
		REQUIRE(solid.spans[1 + 2 * width]);
		REQUIRE(!solid.spans[1 + 3 * width]);

		REQUIRE(solid.spans[0 + 0 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 0 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 0 * width]->area == 1);
		REQUIRE(!solid.spans[0 + 0 * width]->next);

		REQUIRE(solid.spans[0 + 1 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 1 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 1 * width]->area == 1);
		REQUIRE(!solid.spans[0 + 1 * width]->next);

		REQUIRE(solid.spans[0 + 2 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 2 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 2 * width]->area == 2);
		REQUIRE(!solid.spans[0 + 2 * width]->next);

		REQUIRE(solid.spans[0 + 3 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 3 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 3 * width]->area == 2);
		REQUIRE(!solid.spans[0 + 3 * width]->next);

		REQUIRE(solid.spans[1 + 1 * width]->smin == 0);
		REQUIRE(solid.spans[1 + 1 * width]->smax == 1);
		REQUIRE(solid.spans[1 + 1 * width]->area == 1);
		REQUIRE(!solid.spans[1 + 1 * width]->next);

		REQUIRE(solid.spans[1 + 2 * width]->smin == 0);
		REQUIRE(solid.spans[1 + 2 * width]->smax == 1);
		REQUIRE(solid.spans[1 + 2 * width]->area == 2);
		REQUIRE(!solid.spans[1 + 2 * width]->next);
	}

	SECTION("Unsigned short overload")
	{
		unsigned short utris[] = {
			0, 1, 2,
			0, 3, 1
		};
		REQUIRE(rcRasterizeTriangles(&ctx, verts, 4, utris, areas, 2, solid, flagMergeThr));

		REQUIRE(solid.spans[0 + 0 * width]);
		REQUIRE(solid.spans[0 + 1 * width]);
		REQUIRE(solid.spans[0 + 2 * width]);
		REQUIRE(solid.spans[0 + 3 * width]);
		REQUIRE(!solid.spans[1 + 0 * width]);
		REQUIRE(solid.spans[1 + 1 * width]);
		REQUIRE(solid.spans[1 + 2 * width]);
		REQUIRE(!solid.spans[1 + 3 * width]);

		REQUIRE(solid.spans[0 + 0 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 0 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 0 * width]->area == 1);
		REQUIRE(!solid.spans[0 + 0 * width]->next);

		REQUIRE(solid.spans[0 + 1 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 1 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 1 * width]->area == 1);
		REQUIRE(!solid.spans[0 + 1 * width]->next);

		REQUIRE(solid.spans[0 + 2 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 2 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 2 * width]->area == 2);
		REQUIRE(!solid.spans[0 + 2 * width]->next);

		REQUIRE(solid.spans[0 + 3 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 3 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 3 * width]->area == 2);
		REQUIRE(!solid.spans[0 + 3 * width]->next);

		REQUIRE(solid.spans[1 + 1 * width]->smin == 0);
		REQUIRE(solid.spans[1 + 1 * width]->smax == 1);
		REQUIRE(solid.spans[1 + 1 * width]->area == 1);
		REQUIRE(!solid.spans[1 + 1 * width]->next);

		REQUIRE(solid.spans[1 + 2 * width]->smin == 0);
		REQUIRE(solid.spans[1 + 2 * width]->smax == 1);
		REQUIRE(solid.spans[1 + 2 * width]->area == 2);
		REQUIRE(!solid.spans[1 + 2 * width]->next);
	}

	SECTION("Triangle list overload")
	{
		float vertsList[] = {
			0, 0, 0,
			1, 0, 0,
			0, 0, -1,
			0, 0, 0,
			0, 0, 1,
			1, 0, 0,
		};

		REQUIRE(rcRasterizeTriangles(&ctx, vertsList, areas, 2, solid, flagMergeThr));

		REQUIRE(solid.spans[0 + 0 * width]);
		REQUIRE(solid.spans[0 + 1 * width]);
		REQUIRE(solid.spans[0 + 2 * width]);
		REQUIRE(solid.spans[0 + 3 * width]);
		REQUIRE(!solid.spans[1 + 0 * width]);
		REQUIRE(solid.spans[1 + 1 * width]);
		REQUIRE(solid.spans[1 + 2 * width]);
		REQUIRE(!solid.spans[1 + 3 * width]);

		REQUIRE(solid.spans[0 + 0 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 0 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 0 * width]->area == 1);
		REQUIRE(!solid.spans[0 + 0 * width]->next);

		REQUIRE(solid.spans[0 + 1 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 1 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 1 * width]->area == 1);
		REQUIRE(!solid.spans[0 + 1 * width]->next);

		REQUIRE(solid.spans[0 + 2 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 2 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 2 * width]->area == 2);
		REQUIRE(!solid.spans[0 + 2 * width]->next);

		REQUIRE(solid.spans[0 + 3 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 3 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 3 * width]->area == 2);
		REQUIRE(!solid.spans[0 + 3 * width]->next);

		REQUIRE(solid.spans[1 + 1 * width]->smin == 0);
		REQUIRE(solid.spans[1 + 1 * width]->smax == 1);
		REQUIRE(solid.spans[1 + 1 * width]->area == 1);
		REQUIRE(!solid.spans[1 + 1 * width]->next);

		REQUIRE(solid.spans[1 + 2 * width]->smin == 0);
		REQUIRE(solid.spans[1 + 2 * width]->smax == 1);
		REQUIRE(solid.spans[1 + 2 * width]->area == 2);
		REQUIRE(!solid.spans[1 + 2 * width]->next);
	}
}

TEST_CASE("rcClassifyTriangle", "[recast][degenerate]")
{
	const float NaN = std::numeric_limits<float>::quiet_NaN();
	const float Inf = std::numeric_limits<float>::infinity();

	SECTION("A well formed triangle is valid")
	{
		const float v0[] = {0, 0, 0};
		const float v1[] = {2, 0, 0};
		const float v2[] = {0, 0, 2};
		REQUIRE(rcClassifyTriangle(v0, v1, v2) == RC_TRIANGLE_VALID);
	}

	SECTION("A roughly equilateral triangle is valid")
	{
		const float v0[] = {1, 2, 3};
		const float v1[] = {1.1f, 2, 3};
		const float v2[] = {1.05f, 2.1f, 3};
		REQUIRE(rcClassifyTriangle(v0, v1, v2) == RC_TRIANGLE_VALID);
	}

	SECTION("Coincident vertices are degenerate")
	{
		const float v0[] = {0.5f, 0.0f, 0.5f};
		REQUIRE(rcClassifyTriangle(v0, v0, v0) == RC_TRIANGLE_DEGENERATE);

		const float v1[] = {2.0f, 0.0f, 0.5f};
		REQUIRE(rcClassifyTriangle(v0, v0, v1) == RC_TRIANGLE_DEGENERATE);
	}

	SECTION("Collinear vertices are degenerate")
	{
		const float v0[] = {1.0f, 0.0f, 0.5f};
		const float v1[] = {2.0f, 0.0f, 0.5f};
		const float v2[] = {4.0f, 0.0f, 0.5f};
		REQUIRE(rcClassifyTriangle(v0, v1, v2) == RC_TRIANGLE_DEGENERATE);
	}

	SECTION("An extremely short edge is degenerate")
	{
		const float v0[] = {1.0f, 0.0f, 1.0f};
		const float v1[] = {1.0f, 0.0f, 1.0f + 1e-9f};
		const float v2[] = {1.0f, 3.0f, 1.0f};
		REQUIRE(rcClassifyTriangle(v0, v1, v2) == RC_TRIANGLE_DEGENERATE);
	}

	SECTION("Non-finite coordinates are non-finite")
	{
		const float v0[] = {0, 0, 0};
		const float v1[] = {2, 0, 0};
		const float nanY[] = {0, NaN, 2};
		const float infZ[] = {0, 0, Inf};
		REQUIRE(rcClassifyTriangle(v0, v1, nanY) == RC_TRIANGLE_NON_FINITE);
		REQUIRE(rcClassifyTriangle(v0, v1, infZ) == RC_TRIANGLE_NON_FINITE);
	}

	SECTION("Huge finite coordinates are non-finite")
	{
		const float v0[] = {0, 0, 0};
		const float v1[] = {3.0e38f, 0, 0};
		const float v2[] = {0, 3.0e38f, 0};
		REQUIRE(rcClassifyTriangle(v0, v1, v2) == RC_TRIANGLE_NON_FINITE);
	}

	SECTION("Classification is independent of winding order")
	{
		const float v0[] = {0, 0, 0};
		const float v1[] = {2, 0, 0};
		const float v2[] = {0, 0, 2};
		REQUIRE(rcClassifyTriangle(v0, v1, v2) == rcClassifyTriangle(v0, v2, v1));
	}

	SECTION("Classification is translation invariant")
	{
		const float v0[] = {0, 0, 0};
		const float v1[] = {2, 0, 0};
		const float v2[] = {0, 0, 2};
		const float t0[] = {1000, 500, -250};
		const float t1[] = {1002, 500, -250};
		const float t2[] = {1000, 500, -248};
		REQUIRE(rcClassifyTriangle(v0, v1, v2) == rcClassifyTriangle(t0, t1, t2));
	}
}

TEST_CASE("rcMarkWalkableTriangles with degenerate input", "[recast][degenerate]")
{
	rcContext ctx(false);
	const float walkableSlopeAngle = 45.0f;

	const float NaN = std::numeric_limits<float>::quiet_NaN();

	const float verts[] = {
		// 0,1,2: flat, valid triangle.
		0, 0, 0,
		1, 0, 0,
		0, 0, -1,
		// 3,4: collinear with 0,1.
		2, 0, 0,
		3, 0, 0,
		// 5: non-finite.
		0, NaN, 0,
	};
	const int nv = 6;

	const int tris[] = {
		0, 1, 2, // valid
		0, 1, 3, // collinear
		0, 3, 4, // collinear
		0, 1, 5, // non-finite
		3, 3, 3, // coincident
	};
	const int nt = 5;

	SECTION("Degenerate triangles are not marked walkable")
	{
		unsigned char areas[nt];
		for (int i = 0; i < nt; ++i) areas[i] = RC_NULL_AREA;

		rcMarkWalkableTriangles(&ctx, walkableSlopeAngle, verts, nv, tris, nt, areas);

		REQUIRE(areas[0] == RC_WALKABLE_AREA);
		REQUIRE(areas[1] == RC_NULL_AREA);
		REQUIRE(areas[2] == RC_NULL_AREA);
		REQUIRE(areas[3] == RC_NULL_AREA);
		REQUIRE(areas[4] == RC_NULL_AREA);
	}

	SECTION("Degenerate triangles do not clear a pre-assigned area id")
	{
		unsigned char areas[nt];
		for (int i = 0; i < nt; ++i) areas[i] = 42;

		rcMarkWalkableTriangles(&ctx, walkableSlopeAngle, verts, nv, tris, nt, areas);

		REQUIRE(areas[0] == RC_WALKABLE_AREA);
		REQUIRE(areas[1] == 42);
		REQUIRE(areas[2] == 42);
		REQUIRE(areas[3] == 42);
		REQUIRE(areas[4] == 42);
	}

	SECTION("The result is stable across repeated calls")
	{
		unsigned char first[nt];
		unsigned char second[nt];
		for (int i = 0; i < nt; ++i) { first[i] = RC_NULL_AREA; second[i] = RC_NULL_AREA; }

		rcMarkWalkableTriangles(&ctx, walkableSlopeAngle, verts, nv, tris, nt, first);
		rcMarkWalkableTriangles(&ctx, walkableSlopeAngle, verts, nv, tris, nt, second);

		for (int i = 0; i < nt; ++i) REQUIRE(first[i] == second[i]);
	}
}

TEST_CASE("rcClearUnwalkableTriangles with degenerate input", "[recast][degenerate]")
{
	rcContext ctx(false);
	const float walkableSlopeAngle = 45.0f;

	const float NaN = std::numeric_limits<float>::quiet_NaN();

	const float verts[] = {
		0, 0, 0,
		1, 0, 0,
		0, 0, -1,
		2, 0, 0,
		0, NaN, 0,
	};
	const int nv = 5;

	const int tris[] = {
		0, 1, 2, // valid, walkable
		0, 1, 3, // collinear
		0, 1, 4, // non-finite
		3, 3, 3, // coincident
	};
	const int nt = 4;

	SECTION("Degenerate triangles are always made unwalkable")
	{
		unsigned char areas[nt] = {42, 42, 42, 42};

		rcClearUnwalkableTriangles(&ctx, walkableSlopeAngle, verts, nv, tris, nt, areas);

		REQUIRE(areas[0] == 42);
		REQUIRE(areas[1] == RC_NULL_AREA);
		REQUIRE(areas[2] == RC_NULL_AREA);
		REQUIRE(areas[3] == RC_NULL_AREA);
	}
}

TEST_CASE("rcCalcBounds ignores non-finite vertices", "[recast][degenerate]")
{
	const float NaN = std::numeric_limits<float>::quiet_NaN();

	SECTION("A single bad vertex does not poison the bounds")
	{
		const float verts[] = {
			1, 2, 3,
			NaN, NaN, NaN,
			4, 5, 6,
		};
		float bmin[3], bmax[3];
		rcCalcBounds(verts, 3, bmin, bmax);

		REQUIRE(bmin[0] == 1);
		REQUIRE(bmin[1] == 2);
		REQUIRE(bmin[2] == 3);
		REQUIRE(bmax[0] == 4);
		REQUIRE(bmax[1] == 5);
		REQUIRE(bmax[2] == 6);
	}

	SECTION("All vertices non-finite yields a zero sized box")
	{
		const float verts[] = { NaN, 0, 0, 0, NaN, 0 };
		float bmin[3], bmax[3];
		rcCalcBounds(verts, 2, bmin, bmax);

		for (int i = 0; i < 3; ++i)
		{
			REQUIRE(bmin[i] == 0);
			REQUIRE(bmax[i] == 0);
		}
	}
}

TEST_CASE("rcCalcGridSize with invalid input", "[recast][degenerate]")
{
	const float NaN = std::numeric_limits<float>::quiet_NaN();

	SECTION("A non-positive cell size yields an empty grid")
	{
		const float bmin[] = {0, 0, 0};
		const float bmax[] = {10, 10, 10};
		int w = 1, h = 1;
		rcCalcGridSize(bmin, bmax, 0.0f, &w, &h);
		REQUIRE(w == 0);
		REQUIRE(h == 0);

		rcCalcGridSize(bmin, bmax, -1.0f, &w, &h);
		REQUIRE(w == 0);
		REQUIRE(h == 0);
	}

	SECTION("Non-finite bounds yield an empty grid")
	{
		const float bmin[] = {0, 0, 0};
		const float bmax[] = {NaN, 10, 10};
		int w = 1, h = 1;
		rcCalcGridSize(bmin, bmax, 1.0f, &w, &h);
		REQUIRE(w == 0);
		REQUIRE(h == 0);
	}

	SECTION("Huge extents are clamped instead of overflowing")
	{
		const float bmin[] = {0, 0, 0};
		const float bmax[] = {3.0e38f, 0, 3.0e38f};
		int w = 1, h = 1;
		rcCalcGridSize(bmin, bmax, 0.01f, &w, &h);
		REQUIRE(w > 0);
		REQUIRE(h > 0);
	}
}

TEST_CASE("rcCreateHeightfield rejects invalid input", "[recast][degenerate]")
{
	rcContext ctx(false);
	const float NaN = std::numeric_limits<float>::quiet_NaN();
	const float bmin[] = {0, 0, 0};
	const float bmax[] = {10, 10, 10};

	SECTION("Zero or negative cell size is rejected")
	{
		rcHeightfield hf;
		REQUIRE(!rcCreateHeightfield(&ctx, hf, 10, 10, bmin, bmax, 0.0f, 1.0f));
		REQUIRE(!rcCreateHeightfield(&ctx, hf, 10, 10, bmin, bmax, 1.0f, 0.0f));
	}

	SECTION("Non-finite bounds are rejected")
	{
		rcHeightfield hf;
		const float badMax[] = {NaN, 10, 10};
		REQUIRE(!rcCreateHeightfield(&ctx, hf, 10, 10, bmin, badMax, 1.0f, 1.0f));
	}

	SECTION("An empty heightfield is valid and has no span array")
	{
		rcHeightfield hf;
		REQUIRE(rcCreateHeightfield(&ctx, hf, 0, 0, bmin, bmax, 1.0f, 1.0f));
		REQUIRE(hf.width == 0);
		REQUIRE(hf.height == 0);
		REQUIRE(hf.spans == nullptr);
	}
}

namespace
{
/// Runs the complete Recast build pipeline over the supplied triangles and
/// returns whether it completed. Used to verify that malformed input cannot
/// leave the pipeline in a state that makes later queries crash.
bool buildNavMeshData(const float* verts, int numVerts,
                      const int* tris, int numTris,
                      unsigned char** outData, int* outDataSize)
{
	rcContext ctx(false);

	const float minBounds[3] = {-4.0f, -2.0f, -4.0f};
	const float maxBounds[3] = {24.0f, 8.0f, 24.0f};
	const float cs = 0.5f;
	const float ch = 0.25f;

	int width = 0, height = 0;
	rcCalcGridSize(minBounds, maxBounds, cs, &width, &height);
	if (width <= 0 || height <= 0)
	{
		return false;
	}

	rcHeightfield* solid = rcAllocHeightfield();
	if (!solid) return false;
	bool ok = rcCreateHeightfield(&ctx, *solid, width, height, minBounds, maxBounds, cs, ch);
	if (!ok) return false;

	unsigned char* triAreas = new unsigned char[numTris > 0 ? numTris : 1];
	memset(triAreas, 0, numTris > 0 ? numTris : 1);
	rcMarkWalkableTriangles(&ctx, 45.0f, verts, numVerts, tris, numTris, triAreas);

	ok = rcRasterizeTriangles(&ctx, verts, numVerts, tris, triAreas, numTris, *solid, 1);
	delete[] triAreas;
	if (!ok) return false;

	rcFilterLedgeSpans(&ctx, 2, 1, *solid);
	rcFilterWalkableLowHeightSpans(&ctx, 2, *solid);

	rcCompactHeightfield* chf = rcAllocCompactHeightfield();
	if (!chf) return false;
	ok = rcBuildCompactHeightfield(&ctx, 2, 1, *solid, *chf);
	if (!ok) return false;

	ok = rcErodeWalkableArea(&ctx, 1, *chf);
	if (!ok) return false;

	ok = rcBuildDistanceField(&ctx, *chf);
	if (!ok) return false;

	ok = rcBuildRegions(&ctx, *chf, 0, 8, 20);
	if (!ok) return false;

	rcContourSet* cset = rcAllocContourSet();
	if (!cset) return false;
	ok = rcBuildContours(&ctx, *chf, 1.3f, 12, *cset);
	if (!ok) return false;

	rcPolyMesh* pmesh = rcAllocPolyMesh();
	if (!pmesh) return false;
	ok = rcBuildPolyMesh(&ctx, *cset, 6, *pmesh);
	if (!ok) return false;

	rcPolyMeshDetail* dmesh = rcAllocPolyMeshDetail();
	if (!dmesh) return false;
	ok = rcBuildPolyMeshDetail(&ctx, *pmesh, *chf, 1.1f, 1.3f, *dmesh);
	if (!ok) return false;

	bool created = false;
	if (pmesh->nverts > 0 && pmesh->npolys > 0)
	{
		// Mark all polygons as walkable for the default query filter.
		for (int i = 0; i < pmesh->npolys; ++i)
		{
			pmesh->flags[i] = 1;
			if (pmesh->areas[i] == RC_NULL_AREA)
			{
				pmesh->areas[i] = RC_WALKABLE_AREA;
			}
		}

		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.verts = pmesh->verts;
		params.vertCount = pmesh->nverts;
		params.polys = pmesh->polys;
		params.polyAreas = pmesh->areas;
		params.polyFlags = pmesh->flags;
		params.polyCount = pmesh->npolys;
		params.nvp = pmesh->nvp;
		if (dmesh)
		{
			params.detailMeshes = dmesh->meshes;
			params.detailVerts = dmesh->verts;
			params.detailVertsCount = dmesh->nverts;
			params.detailTris = dmesh->tris;
			params.detailTriCount = dmesh->ntris;
		}
		params.walkableHeight = 2.0f;
		params.walkableRadius = 0.5f;
		params.walkableClimb = 0.5f;
		params.cs = cs;
		params.ch = ch;
		params.bmin[0] = pmesh->bmin[0];
		params.bmin[1] = pmesh->bmin[1];
		params.bmin[2] = pmesh->bmin[2];
		params.bmax[0] = pmesh->bmax[0];
		params.bmax[1] = pmesh->bmax[1];
		params.bmax[2] = pmesh->bmax[2];
		params.buildBvTree = true;
		created = dtCreateNavMeshData(&params, outData, outDataSize);
	}

	rcFreePolyMeshDetail(dmesh);
	rcFreePolyMesh(pmesh);
	rcFreeContourSet(cset);
	rcFreeCompactHeightfield(chf);
	rcFreeHeightField(solid);
	return created;
}
} // namespace

TEST_CASE("Malformed input cannot break downstream pathfinding", "[recast][degenerate][pipeline]")
{
	const float NaN = std::numeric_limits<float>::quiet_NaN();

	// A valid floor quad plus a pile of malformed triangles living above it.
	const float verts[] = {
		// Valid floor (two triangles).
		0, 0, 0,
		20, 0, 0,
		0, 0, 20,
		20, 0, 20,
		// Degenerate helpers.
		10, 0, 10,     // 4
		15, 0, 10,     // 5
		5, 0, 10,      // 6
		10, NaN, 10,   // 7
	};
	const int numVerts = 8;
	const int tris[] = {
		// Clockwise winding so the normals point up and the floor is walkable.
		0, 2, 1,
		1, 2, 3,
		4, 5, 6, // collinear
		4, 4, 4, // coincident
		4, 5, 7, // non-finite
		4, 5, 5, // zero area sliver
	};
	const int numTris = 6;

	unsigned char* data = nullptr;
	int dataSize = 0;
	const bool built = buildNavMeshData(verts, numVerts, tris, numTris, &data, &dataSize);
	REQUIRE(built);
	REQUIRE(data != nullptr);
	REQUIRE(dataSize > 0);

	dtNavMesh* navMesh = dtAllocNavMesh();
	REQUIRE(navMesh != nullptr);
	REQUIRE(dtStatusSucceed(navMesh->init(data, dataSize, DT_TILE_FREE_DATA)));

	dtNavMeshQuery* query = dtAllocNavMeshQuery();
	REQUIRE(query != nullptr);
	REQUIRE(dtStatusSucceed(query->init(navMesh, 2048)));

	dtQueryFilter filter;
	const float extents[3] = {2.0f, 4.0f, 2.0f};

	// Find a walkable polygon near each end of the floor.
	dtPolyRef startRef = 0, endRef = 0;
	const float startPos[3] = {2.0f, 0.5f, 2.0f};
	const float endPos[3] = {18.0f, 0.5f, 18.0f};
	float nearestStart[3], nearestEnd[3];

	const dtStatus startStatus = query->findNearestPoly(startPos, extents, &filter, &startRef, nearestStart);
	const dtStatus endStatus = query->findNearestPoly(endPos, extents, &filter, &endRef, nearestEnd);
	REQUIRE(dtStatusSucceed(startStatus));
	REQUIRE(dtStatusSucceed(endStatus));
	REQUIRE(startRef != 0);
	REQUIRE(endRef != 0);

	dtPolyRef path[256];
	int pathCount = 0;
	const dtStatus pathStatus = query->findPath(startRef, endRef, nearestStart, nearestEnd, &filter, path, &pathCount, 256);
	REQUIRE(dtStatusSucceed(pathStatus));
	REQUIRE(pathCount > 0);

	dtFreeNavMeshQuery(query);
	dtFreeNavMesh(navMesh);
}
