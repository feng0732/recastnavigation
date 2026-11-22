#include <limits>

#include "Recast.h"
#include "catch2/catch_amalgamated.hpp"

TEST_CASE("rcAddSpan", "[recast][rasterization]")
{
	rcContext ctx(false);

	constexpr int xSize = 4;
	constexpr int ySize = 10;
	constexpr int zSize = 4;

	constexpr float cellSize = 1.0f;
	constexpr float cellHeight = 2.0f;

	constexpr float minBounds[3] {0.0f, 0.0f, 0.0f};
	constexpr float maxBounds[3] {cellSize * xSize, cellHeight * ySize, cellSize * zSize};

	rcHeightfield hf;
	REQUIRE(rcCreateHeightfield(&ctx, hf, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight));

	constexpr unsigned char area = 42;
	constexpr int flagMergeThr = 1;

	SECTION("Add a span to an empty heightfield.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 1, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 1);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);
	}

	SECTION("Adding invalid or zero-size spans does nothing.")
	{
		// min == max
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 0, area, flagMergeThr));
		REQUIRE(hf.spans[0] == nullptr);

		// min > maxs
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 1, 0, area, flagMergeThr));
		REQUIRE(hf.spans[0] == nullptr);
	}

	SECTION("Two spans that are not touching are not merged.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 1, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 1);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);

		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 2, 3, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->next != nullptr);
		REQUIRE(hf.spans[0]->next->smin == 2);
		REQUIRE(hf.spans[0]->next->smax == 3);
		REQUIRE(hf.spans[0]->next->area == area);
		REQUIRE(hf.spans[0]->next->next == nullptr);
	}

	SECTION("Two spans with different area ids within the flag merge threshold are merged and the highest area ID is used.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 1, 42, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 1);
		REQUIRE(hf.spans[0]->area == 42);
		REQUIRE(hf.spans[0]->next == nullptr);

		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 1, 2, 24, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 2);
		REQUIRE(hf.spans[0]->area == 42); // Higher area ID takes precedent
		REQUIRE(hf.spans[0]->next == nullptr);
	}

	SECTION("Two spans with different area ids outside the flag merge threshold are merged and the area ID of the last span added is used.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 1, 42, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 1);
		REQUIRE(hf.spans[0]->area == 42);
		REQUIRE(hf.spans[0]->next == nullptr);

		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 1, 8, 24, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 8);
		REQUIRE(hf.spans[0]->area == 24); // Area ID of the last-added span takes precedent
		REQUIRE(hf.spans[0]->next == nullptr);
	}

	SECTION("Add a span that gets merged with an existing span.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 1, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 1);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);

		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 1, 2, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 2);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);
	}

	SECTION("Add a span that merges with two spans above and below.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 1, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 1);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);

		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 2, 3, area, flagMergeThr));
		REQUIRE(hf.spans[0]->next != nullptr);
		REQUIRE(hf.spans[0]->next->smin == 2);
		REQUIRE(hf.spans[0]->next->smax == 3);
		REQUIRE(hf.spans[0]->next->area == area);
		REQUIRE(hf.spans[0]->next->next == nullptr);

		// After adding the third span, they should all get merged into a single span.
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 1, 2, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 3);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);
	}

	SECTION("Spans are insertion-sorted in ascending order of Y value.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 2, 3, area, flagMergeThr));
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 1, area, flagMergeThr));
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 6, 7, area, flagMergeThr));

		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 1);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next != nullptr);

		REQUIRE(hf.spans[0]->next->smin == 2);
		REQUIRE(hf.spans[0]->next->smax == 3);
		REQUIRE(hf.spans[0]->next->area == area);
		REQUIRE(hf.spans[0]->next->next != nullptr);

		REQUIRE(hf.spans[0]->next->next->smin == 6);
		REQUIRE(hf.spans[0]->next->next->smax == 7);
		REQUIRE(hf.spans[0]->next->next->area == area);
		REQUIRE(hf.spans[0]->next->next->next == nullptr);
	}

	SECTION("Adding a span inside another span merges them.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 8, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 8);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);

		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 2, 3, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 8);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);
	}

	SECTION("Overlapping spans are merged.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 4, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 4);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);

		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 2, 6, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 6);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);
	}

}

TEST_CASE("allocSpan", "[recast][rasterization]")
{
	rcContext ctx(false);

	constexpr int xSize = 50;
	constexpr int zSize = 50;
	constexpr int ySize = 10;

	constexpr float cellSize = 1.0f;
	constexpr float cellHeight = 2.0f;

	constexpr float minBounds[3] {0.0f, 0.0f, 0.0f};
	constexpr float maxBounds[3] {cellSize * xSize, cellHeight * ySize, cellSize * zSize};

	rcHeightfield hf;
	REQUIRE(rcCreateHeightfield(&ctx, hf, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight));

	constexpr unsigned char area = 42;
	constexpr int flagMergeThr = 1;

	SECTION("Attempting to add more spans than the span pool size allocates a new page")
	{
		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				REQUIRE(rcAddSpan(&ctx, hf, x, z, 0, 1, area, flagMergeThr));
			}
		}
	}
}

TEST_CASE("rcRasterizeTriangle", "[recast][rasterization]")
{
	rcContext ctx(false);

	constexpr int xSize = 10;
	constexpr int ySize = 10;
	constexpr int zSize = 10;

	constexpr float cellSize = 1.0f;
	constexpr float cellHeight = 1.0f;

	constexpr float minBounds[3] {0.0f, 0.0f, 0.0f};
	constexpr float maxBounds[3] {cellSize * xSize, cellHeight * ySize, cellSize * zSize};

	rcHeightfield hf;
	REQUIRE(rcCreateHeightfield(&ctx, hf, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight));

	SECTION("Simple triangle in XZ plane")
	{
		// Triangle in the XZ plane with vertices (0,0,0), (2,0,0), (0,0,2)
		//
		//   Z ^
		//     |
		//     |\
		//     |_\__> X
		//
		//
		//   Z ....
		//     X...
		//     XX..  X
		//
		// Should fill cells: (0,0), (0,1), (1,0)

		// Clockwise winding order so the normal points in the positive Y
		float v0[] = {0.0f, 0.0f, 0.0f};
		float v1[] = {2.0f, 0.0f, 0.0f};
		float v2[] = {0.0f, 0.0f, 2.0f};

		REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

		// Check that only expected cells have spans
		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				rcSpan* span = hf.spans[x + z * hf.width];
				if ((x == 0 && z == 0) || (x == 0 && z == 1) || (x == 1 && z == 0))
				{
					REQUIRE(span != nullptr);
					REQUIRE(span->smin == 0);
					REQUIRE(span->smax == 1);
					REQUIRE(span->area == RC_WALKABLE_AREA);
					REQUIRE(span->next == nullptr);
				}
				else
				{
					REQUIRE(span == nullptr);
				}
			}
		}
	}

	SECTION("Simple triangle inside a single voxel")
	{
		// Clockwise winding order so the normal points in the positive Y
		float v0[] = {0.0f, 0.0f, 0.0f};
		float v1[] = {1.0f, 0.0f, 0.0f};
		float v2[] = {0.0f, 0.0f, 1.0f};

		REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

		// Check that only expected cells have spans
		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				rcSpan* span = hf.spans[x + z * hf.width];
				if (x == 0 && z == 0)
				{
					REQUIRE(span != nullptr);
					REQUIRE(span->smin == 0);
					REQUIRE(span->smax == 1);
					REQUIRE(span->area == RC_WALKABLE_AREA);
					REQUIRE(span->next == nullptr);
				}
				else
				{
					REQUIRE(span == nullptr);
				}
			}
		}
	}

	SECTION("Triangles are clipped by the heightfield bounds")
	{
		// When clipped, this should be the same as the triangle in the previous test
		// Should fill cells: (0,0), (0,1), (1,0)

		// Clockwise winding order so the normal points in the positive Y
		float v0[] = {-2.0f, 0.0f, -2.0f};
		float v1[] = {4.0f, 0.0f, -2.0f};
		float v2[] = {-2.0f, 0.0f, 4.0f};

		REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

		// Check that only expected cells have spans
		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				rcSpan* span = hf.spans[x + z * hf.width];
				if ((x == 0 && z == 0) || (x == 0 && z == 1) || (x == 1 && z == 0))
				{
					REQUIRE(span != nullptr);
					REQUIRE(span->smin == 0);
					REQUIRE(span->smax == 1);
					REQUIRE(span->area == RC_WALKABLE_AREA);
					REQUIRE(span->next == nullptr);
				}
				else
				{
					REQUIRE(span == nullptr);
				}
			}
		}
	}

	SECTION("Triangle outside of heightfield bounds rasterizes to nothing")
	{
		{ // Outside xz bounds
			float v0[] = {-5.0f, 0.0f, -5.0f};
			float v1[] = {-5.0f, 0.0f, 5.0f};
			float v2[] = {5.0f, 0.0f, -5.0f};
			REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

			// Check that no spans were added
			for (int x = 0; x < xSize; x++)
			{
				for (int z = 0; z < zSize; z++)
				{
					REQUIRE(hf.spans[x + z * hf.width] == nullptr);
				}
			}
		}

		{  // below y bounds
			float v0[] = {0.0f, -1.0f, 0.0f};
			float v1[] = {5.0f, -1.0f, 5.0f};
			float v2[] = {5.0f, -1.0f, 0.0f};
			REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

			// Check that no spans were added
			for (int x = 0; x < xSize; x++)
			{
				for (int z = 0; z < zSize; z++)
				{
					REQUIRE(hf.spans[x + z * hf.width] == nullptr);
				}
			}
		}

		{  // above y bounds
			float v0[] = {0.0f, 40.0f, 0.0f};
			float v1[] = {5.0f, 40.0f, 5.0f};
			float v2[] = {5.0f, 40.0f, 0.0f};
			REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

			// Check that no spans were added
			for (int x = 0; x < xSize; x++)
			{
				for (int z = 0; z < zSize; z++)
				{
					REQUIRE(hf.spans[x + z * hf.width] == nullptr);
				}
			}
		}
	}

	SECTION("Voxels are rasterized if the triangle overlaps any part of it at all")
	{
		// This triangle when clipped barely overlaps the area of the cell at 0,0

		// Clockwise winding order so the normal points in the positive Y
		float v0[] = {-1.0f, 0.0f, -1.0f};
		float v1[] = {1.01f, 0.0f, -1.0f};
		float v2[] = {-1.0f, 0.0f, 1.01f};

		REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

		// Check that only expected cells have spans
		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				rcSpan* span = hf.spans[x + z * hf.width];
				if (x == 0 && z == 0)
				{
					REQUIRE(span != nullptr);
					REQUIRE(span->smin == 0);
					REQUIRE(span->smax == 1);
					REQUIRE(span->area == RC_WALKABLE_AREA);
					REQUIRE(span->next == nullptr);
				}
				else
				{
					REQUIRE(span == nullptr);
				}
			}
		}
	}

	SECTION("A rasterized triangle vertical span includes all the voxels it is in any way part of.")
	{
		// Clockwise winding order so the normal points in the positive Y
		float v0[] = {0.0f, 0.0f, 0.0f};
		float v1[] = {0.5f, 0.0f, 0.5f};
		float v2[] = {0.5f, 2.01f, 0.5f};

		REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

		// Check that only expected cells have spans
		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				rcSpan* span = hf.spans[x + z * hf.width];
				if (x == 0 && z == 0)
				{
					REQUIRE(span != nullptr);
					REQUIRE(span->smin == 0);
					REQUIRE(span->smax == 3);
					REQUIRE(span->area == RC_WALKABLE_AREA);
					REQUIRE(span->next == nullptr);
				}
				else
				{
					REQUIRE(span == nullptr);
				}
			}
		}
	}

	SECTION("Sloped triangle produces varying span heights")
	{
		//   Y ^
		//    2|     /v2
		//     |    /  \
		//    1|   /    \
		//     |  /      \
		//   --+-v0------v1----> X
		//     |  0   2   4
		//
		//   Should rasterize to...
		//
		//   Y ^
		//    2|  |--|--|
		//     |  |     |
		//    1|--|  +  |--|
		//     |           |
		//   --+--------------> X
		//     |
		//

		float v0[] = {0.0f, 0.0f, 0.5f};
		float v1[] = {4.0f, 0.0f, 0.5f};
		float v2[] = {2.0f, 2.0f, 0.5f};

		REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

		// Check that only expected cells have spans
		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				rcSpan* span = hf.spans[x + z * hf.width];
				if ((x == 0 && z == 0) || (x == 3 && z == 0))
				{
					REQUIRE(span != nullptr);
					REQUIRE(span->smin == 0);
					REQUIRE(span->smax == 1);
					REQUIRE(span->area == RC_WALKABLE_AREA);
					REQUIRE(span->next == nullptr);
				}
				else if ((x == 1 && z == 0) || (x == 2 && z == 0))
				{
					REQUIRE(span != nullptr);
					REQUIRE(span->smin == 0);
					REQUIRE(span->smax == 2);
					REQUIRE(span->area == RC_WALKABLE_AREA);
					REQUIRE(span->next == nullptr);
				}
				else
				{
					REQUIRE(span == nullptr);
				}
			}
		}
	}

	SECTION("Triangle crossing Y bounds gets clamped")
	{
		// This trangle should be clipped to both the max and min Y bounds of the heightfield.
		float v0[] = {0.0f, -5.0f, 0.0f};
		float v1[] = {1.0f, -5.0f, 1.0f};
		float v2[] = {0.5f, 15.0f, 0.5f};

		REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

		// Check that only expected cells have spans
		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				rcSpan* span = hf.spans[x + z * hf.width];
				if (x == 0 && z == 0)
				{
					REQUIRE(span != nullptr);
					REQUIRE(span->smin == 0);
					REQUIRE(span->smax == 10);
					REQUIRE(span->area == RC_WALKABLE_AREA);
					REQUIRE(span->next == nullptr);
				}
				else
				{
					REQUIRE(span == nullptr);
				}
			}
		}
	}

	SECTION("Degenerate triangles rasterize to nothing")
	{
		auto expectNoSpans = [&]()
		{
			for (int x = 0; x < xSize; x++)
			{
				for (int z = 0; z < zSize; z++)
				{
					REQUIRE(hf.spans[x + z * hf.width] == nullptr);
				}
			}
		};

		{ // Co-linear points
			float v0[] = {1.0f, 0.0f, 0.5f};
			float v1[] = {2.0f, 0.0f, 0.5f};
			float v2[] = {4.0f, 0.0f, 0.5f};
			REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));
			expectNoSpans();
		}

		{ // All vertices are the same point
			float v0[] = {0.5f, 0.0f, 0.5f};
			REQUIRE(rcRasterizeTriangle(&ctx, v0, v0, v0, RC_WALKABLE_AREA, hf, 1));
			expectNoSpans();
		}

		{ // Two coincident vertices, zero area
			float v0[] = {0.5f, 0.0f, 0.5f};
			float v1[] = {0.5f, 0.0f, 0.5f};
			float v2[] = {3.5f, 0.0f, 0.5f};
			REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));
			expectNoSpans();
		}

		{ // Vertical sliver with an extremely short base
			float v0[] = {1.0f, 0.0f, 1.0f};
			float v1[] = {1.0f + 1e-8f, 0.0f, 1.0f};
			float v2[] = {1.0f, 3.0f, 1.0f};
			REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));
			expectNoSpans();
		}

		{ // Non-finite coordinates
			float v0[] = {0.0f, 0.0f, 0.0f};
			float v1[] = {2.0f, 0.0f, 0.0f};
			float v2[] = {0.0f, std::numeric_limits<float>::quiet_NaN(), 2.0f};
			REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));
			expectNoSpans();

			float w2[] = {0.0f, std::numeric_limits<float>::infinity(), 2.0f};
			REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, w2, RC_WALKABLE_AREA, hf, 1));
			expectNoSpans();
		}

		{ // Huge but finite coordinates
			float v0[] = {0.0f, 0.0f, 0.0f};
			float v1[] = {3.0e38f, 0.0f, 0.0f};
			float v2[] = {0.0f, 3.0e38f, 0.0f};
			REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));
			expectNoSpans();
		}
	}

	SECTION("Degenerate triangles do not disturb a valid triangle sharing the same cells")
	{
		// The valid triangle below covers cells (0,0), (0,1) and (1,0).
		float valid0[] = {0.0f, 0.0f, 0.0f};
		float valid1[] = {2.0f, 0.0f, 0.0f};
		float valid2[] = {0.0f, 0.0f, 2.0f};
		REQUIRE(rcRasterizeTriangle(&ctx, valid0, valid1, valid2, RC_WALKABLE_AREA, hf, 1));

		// Overlapping degenerates of every shape, at heights that would otherwise
		// create extra spans or walkable flags.
		{
			float v0[] = {0.0f, 0.5f, 0.0f};
			float v1[] = {1.0f, 0.5f, 0.0f};
			float v2[] = {2.0f, 0.5f, 0.0f};
			REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

			float n0[] = {0.0f, 0.0f, 0.0f};
			float n1[] = {1.0f, 0.0f, 0.0f};
			float n2[] = {0.0f, std::numeric_limits<float>::quiet_NaN(), 1.0f};
			REQUIRE(rcRasterizeTriangle(&ctx, n0, n1, n2, RC_WALKABLE_AREA, hf, 1));
		}

		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				rcSpan* span = hf.spans[x + z * hf.width];
				if ((x == 0 && z == 0) || (x == 0 && z == 1) || (x == 1 && z == 0))
				{
					REQUIRE(span != nullptr);
					REQUIRE(span->smin == 0);
					REQUIRE(span->smax == 1);
					REQUIRE(span->area == RC_WALKABLE_AREA);
					REQUIRE(span->next == nullptr);
				}
				else
				{
					REQUIRE(span == nullptr);
				}
			}
		}
	}

	SECTION("Triangles outside the heightfield with bounding boxes that overlap the heightfield rasterize nothing")
	{
		// This is a minimal repro case for the issue fixed in PR #476 (https://github.com/recastnavigation/recastnavigation/pull/476)
		float v0[] = {-10.0, 5.5, -10.0};
		float v1[] = {-10.0, 5.5, 3};
		float v2[] = {3.0, 5.5, -10.0};
		REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

		// Check that no spans were added
		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				REQUIRE(hf.spans[x + z * hf.width] == nullptr);
			}
		}
	}
}

TEST_CASE("rcRasterizeTriangles rejects malformed index data", "[recast][rasterization][degenerate]")
{
	rcContext ctx(false);

	constexpr int xSize = 5;
	constexpr int zSize = 5;
	constexpr float cellSize = 1.0f;
	constexpr float cellHeight = 1.0f;
	const float minBounds[3] = {0.0f, 0.0f, 0.0f};
	const float maxBounds[3] = {cellSize * xSize, cellHeight * xSize, cellSize * zSize};

	rcHeightfield hf;
	REQUIRE(rcCreateHeightfield(&ctx, hf, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight));

	const float verts[] = {
		0.0f, 0.0f, 0.0f,
		2.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 2.0f,
	};
	const int numVerts = 3;
	const unsigned char areas[] = {RC_WALKABLE_AREA, RC_WALKABLE_AREA};

	SECTION("Out of range vertex indices are skipped")
	{
		const int tris[] = {
			0, 1, 2,
			0, 1, 99,
		};
		REQUIRE(rcRasterizeTriangles(&ctx, verts, numVerts, tris, areas, 2, hf, 1));

		// Only the valid triangle contributed spans.
		REQUIRE(hf.spans[0 + 0 * xSize] != nullptr);
		REQUIRE(hf.spans[0 + 1 * xSize] != nullptr);
		REQUIRE(hf.spans[1 + 0 * xSize] != nullptr);
	}

	SECTION("Negative vertex indices are skipped")
	{
		const int tris[] = { -1, 1, 2 };
		REQUIRE(rcRasterizeTriangles(&ctx, verts, numVerts, tris, areas, 1, hf, 1));
		for (int x = 0; x < xSize; ++x)
			for (int z = 0; z < zSize; ++z)
				REQUIRE(hf.spans[x + z * xSize] == nullptr);
	}

	SECTION("A negative triangle count is rejected")
	{
		const int tris[] = { 0, 1, 2 };
		REQUIRE(!rcRasterizeTriangles(&ctx, verts, numVerts, tris, areas, -1, hf, 1));
	}

	SECTION("A null index array with triangles is rejected")
	{
		REQUIRE(!rcRasterizeTriangles(&ctx, verts, numVerts, static_cast<const int*>(nullptr), areas, 1, hf, 1));
	}

	SECTION("A zero triangle count succeeds and adds nothing")
	{
		REQUIRE(rcRasterizeTriangles(&ctx, verts, numVerts, static_cast<const int*>(nullptr), areas, 0, hf, 1));
	}
}

TEST_CASE("rcAddSpan rejects out of bounds columns", "[recast][rasterization][degenerate]")
{
	rcContext ctx(false);

	constexpr int xSize = 4;
	constexpr int zSize = 4;
	const float minBounds[3] = {0.0f, 0.0f, 0.0f};
	const float maxBounds[3] = {4.0f, 10.0f, 4.0f};

	rcHeightfield hf;
	REQUIRE(rcCreateHeightfield(&ctx, hf, xSize, zSize, minBounds, maxBounds, 1.0f, 2.0f));

	// Negative and too-large coordinates must be ignored, not written.
	REQUIRE(rcAddSpan(&ctx, hf, -1, 0, 0, 1, 42, 1));
	REQUIRE(rcAddSpan(&ctx, hf, 0, -1, 0, 1, 42, 1));
	REQUIRE(rcAddSpan(&ctx, hf, xSize, 0, 0, 1, 42, 1));
	REQUIRE(rcAddSpan(&ctx, hf, 0, zSize, 0, 1, 42, 1));

	for (int i = 0; i < xSize * zSize; ++i)
	{
		REQUIRE(hf.spans[i] == nullptr);
	}

	// In bounds still works.
	REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 1, 42, 1));
	REQUIRE(hf.spans[0] != nullptr);
}

TEST_CASE("rcRasterizeTriangle handles an empty heightfield", "[recast][rasterization][degenerate]")
{
	rcContext ctx(false);

	const float minBounds[3] = {0.0f, 0.0f, 0.0f};
	const float maxBounds[3] = {0.0f, 0.0f, 0.0f};

	rcHeightfield hf;
	REQUIRE(rcCreateHeightfield(&ctx, hf, 0, 0, minBounds, maxBounds, 1.0f, 1.0f));

	const float v0[] = {0.0f, 0.0f, 0.0f};
	const float v1[] = {2.0f, 0.0f, 0.0f};
	const float v2[] = {0.0f, 0.0f, 2.0f};

	// No grid to write into; the call must succeed without touching memory.
	REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));
}

TEST_CASE("Rasterization is consistent across tile boundaries", "[recast][rasterization][degenerate]")
{
	// A triangle spanning a boundary between two tiles, together with a
	// degenerate triangle sitting exactly on that boundary. Rasterizing the
	// whole span at once must produce the same spikes as rasterizing each tile
	// separately with the same triangle set.
	rcContext ctx(false);

	constexpr float cellSize = 1.0f;
	constexpr float cellHeight = 1.0f;

	const float tile0Min[3] = {0.0f, 0.0f, 0.0f};
	const float tile0Max[3] = {8.0f, 8.0f, 8.0f};
	const float tile1Min[3] = {8.0f, 0.0f, 0.0f};
	const float tile1Max[3] = {16.0f, 8.0f, 8.0f};

	const float v0[] = {6.0f, 0.0f, 2.0f};
	const float v1[] = {10.0f, 0.0f, 2.0f};
	const float v2[] = {8.0f, 0.0f, 6.0f};

	const float d0[] = {8.0f, 0.0f, 0.0f};
	const float d1[] = {8.0f, 0.0f, 8.0f};

	auto rasterizeInto = [&](rcHeightfield& hf)
	{
		REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));
		REQUIRE(rcRasterizeTriangle(&ctx, d0, d0, d1, RC_WALKABLE_AREA, hf, 1));
	};

	rcHeightfield whole;
	REQUIRE(rcCreateHeightfield(&ctx, whole, 16, 8, tile0Min, tile1Max, cellSize, cellHeight));
	rasterizeInto(whole);

	rcHeightfield t0, t1;
	REQUIRE(rcCreateHeightfield(&ctx, t0, 8, 8, tile0Min, tile0Max, cellSize, cellHeight));
	REQUIRE(rcCreateHeightfield(&ctx, t1, 8, 8, tile1Min, tile1Max, cellSize, cellHeight));
	rasterizeInto(t0);
	rasterizeInto(t1);

	// Compare the overlapping columns.
	for (int z = 0; z < 8; ++z)
	{
		for (int x = 0; x < 8; ++x)
		{
			const rcSpan* a = whole.spans[x + z * whole.width];
			const rcSpan* b = t0.spans[x + z * t0.width];
			REQUIRE((a == nullptr) == (b == nullptr));
			if (a && b)
			{
				REQUIRE(a->smin == b->smin);
				REQUIRE(a->smax == b->smax);
				REQUIRE(a->area == b->area);
			}

			const rcSpan* c = whole.spans[(x + 8) + z * whole.width];
			const rcSpan* d = t1.spans[x + z * t1.width];
			REQUIRE((c == nullptr) == (d == nullptr));
			if (c && d)
			{
				REQUIRE(c->smin == d->smin);
				REQUIRE(c->smax == d->smax);
				REQUIRE(c->area == d->area);
			}
		}
	}
}

TEST_CASE("Repeated rasterization into a fresh heightfield is identical", "[recast][rasterization][degenerate]")
{
	rcContext ctx(false);

	constexpr int xSize = 8;
	constexpr int zSize = 8;
	const float minBounds[3] = {0.0f, 0.0f, 0.0f};
	const float maxBounds[3] = {8.0f, 8.0f, 8.0f};

	const float verts[] = {
		0.0f, 0.0f, 0.0f,
		4.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 4.0f,
		2.0f, 0.0f, 2.0f, // collinear helper
		std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f,
	};
	const int tris[] = {
		0, 1, 2,
		0, 1, 3, // collinear
		0, 1, 4, // non-finite
	};
	const unsigned char areas[] = {RC_WALKABLE_AREA, RC_WALKABLE_AREA, RC_WALKABLE_AREA};

	auto build = [&](rcHeightfield& hf)
	{
		REQUIRE(rcCreateHeightfield(&ctx, hf, xSize, zSize, minBounds, maxBounds, 1.0f, 1.0f));
		REQUIRE(rcRasterizeTriangles(&ctx, verts, 5, tris, areas, 3, hf, 1));
	};

	rcHeightfield first, second;
	build(first);
	build(second);

	for (int i = 0; i < xSize * zSize; ++i)
	{
		const rcSpan* a = first.spans[i];
		const rcSpan* b = second.spans[i];
		REQUIRE((a == nullptr) == (b == nullptr));
		while (a && b)
		{
			REQUIRE(a->smin == b->smin);
			REQUIRE(a->smax == b->smax);
			REQUIRE(a->area == b->area);
			a = a->next;
			b = b->next;
		}
		REQUIRE(a == nullptr);
		REQUIRE(b == nullptr);
	}
}

TEST_CASE("Rasterization never produces malformed spans", "[recast][rasterization][degenerate]")
{
	// Deterministic pseudo-random sweep over hostile vertex data. Every call
	// must succeed, and every produced span must be well formed and inside the
	// heightfield.
	rcContext ctx(false);

	constexpr int xSize = 16;
	constexpr int zSize = 16;
	const float minBounds[3] = {0.0f, 0.0f, 0.0f};
	const float maxBounds[3] = {16.0f, 8.0f, 16.0f};

	const float NaN = std::numeric_limits<float>::quiet_NaN();
	const float Inf = std::numeric_limits<float>::infinity();

	rcHeightfield hf;
	REQUIRE(rcCreateHeightfield(&ctx, hf, xSize, zSize, minBounds, maxBounds, 1.0f, 1.0f));

	unsigned int state = 0x12345678u;
	auto next = [&state]()
	{
		state = state * 1664525u + 1013904223u;
		return state;
	};
	auto nextValue = [&next, NaN, Inf]() -> float
	{
		switch (next() % 10u)
		{
		case 0: return NaN;
		case 1: return Inf;
		case 2: return -Inf;
		case 3: return 3.0e38f * ((next() & 1u) ? 1.0f : -1.0f);
		default: return ((float)(next() % 2000u) / 1000.0f - 1.0f) * 10.0f;
		}
	};

	for (int i = 0; i < 2000; ++i)
	{
		float v[9];
		for (int k = 0; k < 9; ++k) v[k] = nextValue();

		// Frequently force exact degeneracy.
		if (next() % 3u == 0) { v[3] = v[0]; v[4] = v[1]; v[5] = v[2]; }
		if (next() % 3u == 0) { v[6] = v[0]; v[7] = v[1]; v[8] = v[2]; }

		REQUIRE(rcRasterizeTriangle(&ctx, v, v + 3, v + 6, RC_WALKABLE_AREA, hf, 1));
	}

	for (int i = 0; i < xSize * zSize; ++i)
	{
		for (const rcSpan* span = hf.spans[i]; span != nullptr; span = span->next)
		{
			REQUIRE(span->smin < span->smax);
			REQUIRE(span->smax <= RC_SPAN_MAX_HEIGHT);
		}
	}
}
