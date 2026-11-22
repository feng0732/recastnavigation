//
// Tests for dtNavMeshQuery::findNearestReachablePoly, the multi-target
// "nearest reachable" path query.
//
// The test navigation meshes are built through the full Recast pipeline from
// simple flat slab geometry (arbitrary xz polygons with a constant thickness),
// so the tests exercise real polygon links, region boundary handling and area
// costs instead of hand-crafted mesh internals. Separate slabs produce
// disconnected regions; a single concave slab produces one connected region
// with narrow parts.
//

#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourCommon.h"
#include "Recast.h"
#include "catch2/catch_amalgamated.hpp"

#include <vector>
#include <cstring>

namespace
{
	// Walkable flag assigned to every generated polygon.
	const unsigned short WALKABLE_FLAG = 0x01;

	// Area ids used to drive traversal costs.
	const unsigned char AREA_DEFAULT = 0;
	const unsigned char AREA_EXPENSIVE = 1;

	// A flat slab described by its top-surface xz polygon (CCW when viewed from
	// above). The slab is extruded one unit downwards to give the rasterizer a
	// solid volume.
	using Loop = std::vector<float>; // (x, z) pairs

	void addSlab(const Loop& loop, std::vector<float>& verts, std::vector<int>& tris)
	{
		const int n = static_cast<int>(loop.size() / 2);
		const int base = static_cast<int>(verts.size() / 3);
		const float yTop = 0.0f;
		const float yBottom = -1.0f;

		for (int i = 0; i < n; ++i)
		{
			verts.push_back(loop[i * 2]);
			verts.push_back(yTop);
			verts.push_back(loop[i * 2 + 1]);
		}
		for (int i = 0; i < n; ++i)
		{
			verts.push_back(loop[i * 2]);
			verts.push_back(yBottom);
			verts.push_back(loop[i * 2 + 1]);
		}

		// Top fan.
		for (int i = 1; i < n - 1; ++i)
		{
			tris.push_back(base + 0);
			tris.push_back(base + i);
			tris.push_back(base + i + 1);
		}
		// Bottom fan (reversed winding).
		for (int i = 1; i < n - 1; ++i)
		{
			tris.push_back(base + n);
			tris.push_back(base + n + i + 1);
			tris.push_back(base + n + i);
		}
		// Side walls.
		for (int i = 0; i < n; ++i)
		{
			const int j = (i + 1) % n;
			tris.push_back(base + i);
			tris.push_back(base + n + i);
			tris.push_back(base + n + j);
			tris.push_back(base + i);
			tris.push_back(base + n + j);
			tris.push_back(base + j);
		}
	}

	// RAII holder for the Recast intermediates of one mesh build.
	struct RecastPipeline
	{
		rcHeightfield* heightfield = 0;
		rcCompactHeightfield* chf = 0;
		rcContourSet* cset = 0;
		rcPolyMesh* polyMesh = 0;
		rcPolyMeshDetail* detailMesh = 0;

		~RecastPipeline()
		{
			rcFreePolyMeshDetail(detailMesh);
			rcFreePolyMesh(polyMesh);
			rcFreeContourSet(cset);
			rcFreeCompactHeightfield(chf);
			rcFreeHeightField(heightfield);
		}
	};

	struct BuiltNavMesh
	{
		dtNavMesh* navMesh = 0;
		dtNavMeshQuery* query = 0;

		~BuiltNavMesh()
		{
			if (query)
				dtFreeNavMeshQuery(query);
			if (navMesh)
				dtFreeNavMesh(navMesh);
		}
	};

	// Builds a single-tile navigation mesh from the given slabs. When
	// expensiveArea is non-null it must point to four (x, y, z) vertices of a
	// convex polygon on the ground that is marked with AREA_EXPENSIVE.
	bool buildNavMesh(const std::vector<Loop>& slabs, const float* expensiveArea,
					  BuiltNavMesh& out)
	{
		rcContext ctx;

		std::vector<float> verts;
		std::vector<int> tris;
		for (size_t i = 0; i < slabs.size(); ++i)
			addSlab(slabs[i], verts, tris);

		const int numVerts = static_cast<int>(verts.size() / 3);
		const int numTris = static_cast<int>(tris.size() / 3);

		rcConfig config;
		memset(&config, 0, sizeof(config));
		config.cs = 0.3f;
		config.ch = 0.2f;
		config.walkableSlopeAngle = 45.0f;
		// The test slabs are only one unit thick. Keep the agent dimensions
		// small so ledge filtering does not remove every walkable span.
		config.walkableHeight = 2;
		config.walkableClimb = 1;
		config.walkableRadius = 0;
		config.maxEdgeLen = 12;
		config.maxSimplificationError = 1.3f;
		config.minRegionArea = 8;
		config.mergeRegionArea = 20;
		config.maxVertsPerPoly = 6;
		config.detailSampleDist = 6.0f;
		config.detailSampleMaxError = 1.0f;
		config.tileSize = 0;
		config.borderSize = 0;

		rcVcopy(config.bmin, verts.data());
		rcVcopy(config.bmax, verts.data());
		for (int i = 1; i < numVerts; ++i)
		{
			dtVmin(config.bmin, &verts[i * 3]);
			dtVmax(config.bmax, &verts[i * 3]);
		}
		// Pad the bounds so the heightfield covers the whole input geometry.
		config.bmin[0] -= config.cs;
		config.bmin[2] -= config.cs;
		config.bmax[0] += config.cs;
		config.bmax[2] += config.cs;
		rcCalcGridSize(config.bmin, config.bmax, config.cs, &config.width, &config.height);

		RecastPipeline pipe;
		pipe.heightfield = rcAllocHeightfield();
		if (!pipe.heightfield ||
			!rcCreateHeightfield(&ctx, *pipe.heightfield, config.width, config.height,
								 config.bmin, config.bmax, config.cs, config.ch))
			return false;

		std::vector<unsigned char> triAreas(numTris);
		rcMarkWalkableTriangles(&ctx, config.walkableSlopeAngle, verts.data(), numVerts,
								tris.data(), numTris, triAreas.data());
		if (!rcRasterizeTriangles(&ctx, verts.data(), numVerts, tris.data(), triAreas.data(),
								  numTris, *pipe.heightfield, config.walkableClimb))
			return false;

		rcFilterLedgeSpans(&ctx, config.walkableHeight, config.walkableClimb, *pipe.heightfield);
		rcFilterWalkableLowHeightSpans(&ctx, config.walkableHeight, *pipe.heightfield);

		pipe.chf = rcAllocCompactHeightfield();
		if (!pipe.chf ||
			!rcBuildCompactHeightfield(&ctx, config.walkableHeight, config.walkableClimb,
									   *pipe.heightfield, *pipe.chf))
			return false;
		if (!rcErodeWalkableArea(&ctx, config.walkableRadius, *pipe.chf))
			return false;

		if (expensiveArea)
		{
			rcMarkConvexPolyArea(&ctx, expensiveArea, 4,
								 config.bmin[1], config.bmax[1] + 1.0f,
								 AREA_EXPENSIVE, *pipe.chf);
		}

		if (!rcBuildRegionsMonotone(&ctx, *pipe.chf, 0, config.minRegionArea,
									config.mergeRegionArea))
			return false;

		pipe.cset = rcAllocContourSet();
		if (!pipe.cset ||
			!rcBuildContours(&ctx, *pipe.chf, config.maxSimplificationError, config.maxEdgeLen,
							 *pipe.cset))
			return false;

		pipe.polyMesh = rcAllocPolyMesh();
		if (!pipe.polyMesh ||
			!rcBuildPolyMesh(&ctx, *pipe.cset, config.maxVertsPerPoly, *pipe.polyMesh))
			return false;

		pipe.detailMesh = rcAllocPolyMeshDetail();
		if (!pipe.detailMesh ||
			!rcBuildPolyMeshDetail(&ctx, *pipe.polyMesh, *pipe.chf,
								   config.detailSampleDist, config.detailSampleMaxError,
								   *pipe.detailMesh))
			return false;

		for (int i = 0; i < pipe.polyMesh->npolys; ++i)
		{
			pipe.polyMesh->flags[i] = WALKABLE_FLAG;
			if (pipe.polyMesh->areas[i] == RC_WALKABLE_AREA)
				pipe.polyMesh->areas[i] = AREA_DEFAULT;
		}

		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.verts = pipe.polyMesh->verts;
		params.vertCount = pipe.polyMesh->nverts;
		params.polys = pipe.polyMesh->polys;
		params.polyAreas = pipe.polyMesh->areas;
		params.polyFlags = pipe.polyMesh->flags;
		params.polyCount = pipe.polyMesh->npolys;
		params.nvp = pipe.polyMesh->nvp;
		params.detailMeshes = pipe.detailMesh->meshes;
		params.detailVerts = pipe.detailMesh->verts;
		params.detailVertsCount = pipe.detailMesh->nverts;
		params.detailTris = pipe.detailMesh->tris;
		params.detailTriCount = pipe.detailMesh->ntris;
		params.walkableHeight = 2.0f;
		params.walkableRadius = 0.0f;
		params.walkableClimb = 0.2f;
		rcVcopy(params.bmin, pipe.polyMesh->bmin);
		rcVcopy(params.bmax, pipe.polyMesh->bmax);
		params.cs = config.cs;
		params.ch = config.ch;
		params.buildBvTree = true;

		unsigned char* navData = 0;
		int navDataSize = 0;
		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
			return false;

		out.navMesh = dtAllocNavMesh();
		if (!out.navMesh)
		{
			dtFree(navData);
			return false;
		}
		const dtStatus navStatus = out.navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
		if (dtStatusFailed(navStatus))
		{
			dtFree(navData);
			dtFreeNavMesh(out.navMesh);
			out.navMesh = 0;
			return false;
		}

		out.query = dtAllocNavMeshQuery();
		if (!out.query)
			return false;
		if (dtStatusFailed(out.query->init(out.navMesh, 4096)))
			return false;

		return true;
	}

	bool findPolyAt(const dtNavMeshQuery& query, const float* pos, dtPolyRef& ref,
					float* nearestPt = 0)
	{
		const float halfExtents[3] = {2.0f, 4.0f, 2.0f};
		dtQueryFilter filter;
		float pt[3];
		const dtStatus status = query.findNearestPoly(pos, halfExtents, &filter, &ref,
													   nearestPt ? nearestPt : pt);
		return dtStatusSucceed(status) && ref != 0;
	}
}

TEST_CASE("findNearestReachablePoly selects the cheapest of several reachable targets")
{
	// One long walkable strip along the x-axis. Targets sit at increasing
	// distances from the start, so the nearest one must win.
	const std::vector<Loop> slabs = {
		{-1.0f, -1.0f, 20.0f, -1.0f, 20.0f, 1.0f, -1.0f, 1.0f},
	};

	BuiltNavMesh built;
	REQUIRE(buildNavMesh(slabs, 0, built));
	REQUIRE(built.query != 0);

	dtQueryFilter filter;

	const float startPos[3] = {0.0f, 0.1f, 0.0f};
	dtPolyRef startRef = 0;
	REQUIRE(findPolyAt(*built.query, startPos, startRef));

	const float targetPositions[][3] = {
		{4.0f, 0.1f, 0.0f},
		{12.0f, 0.1f, 0.0f},
		{18.0f, 0.1f, 0.0f},
	};
	dtPolyRef targetRefs[3] = {0, 0, 0};
	for (int i = 0; i < 3; ++i)
		REQUIRE(findPolyAt(*built.query, targetPositions[i], targetRefs[i]));

	int nearestIndex = -2;
	dtPolyRef path[64];
	int pathCount = 0;
	float nearestCost = -1.0f;
	const dtStatus status = built.query->findNearestReachablePoly(
		startRef, startPos, targetRefs, &targetPositions[0][0], 3, &filter,
		&nearestIndex, path, &pathCount, 64, &nearestCost);

	REQUIRE(dtStatusSucceed(status));
	REQUIRE_FALSE(dtStatusDetail(status, DT_PARTIAL_RESULT));
	REQUIRE(nearestIndex == 0);
	REQUIRE(nearestCost >= 0.0f);
	REQUIRE(pathCount >= 2);
	REQUIRE(path[0] == startRef);
	REQUIRE(path[pathCount - 1] == targetRefs[0]);

	// The reported corridor must match a dedicated findPath to the same target.
	dtPolyRef singlePath[64];
	int singlePathCount = 0;
	const dtStatus singleStatus = built.query->findPath(
		startRef, targetRefs[0], startPos, targetPositions[0], &filter,
		singlePath, &singlePathCount, 64);
	REQUIRE(dtStatusSucceed(singleStatus));
	REQUIRE(singlePathCount == pathCount);
	for (int i = 0; i < pathCount; ++i)
		REQUIRE(path[i] == singlePath[i]);
}

TEST_CASE("findNearestReachablePoly reaches targets across separate regions and boundaries")
{
	// A single concave slab forms two wide regions joined by a narrow neck:
	// the target on the far region is reachable only through the neck
	// boundary polygons. A separate slab forms an isolated region whose
	// target cannot be reached.
	const std::vector<Loop> slabs = {
		// Connected slab: start region (x -1..4, z -2..2), neck (x 4..6,
		// z -0.5..0.5), far region (x 6..12, z -2..2).
		{-1.0f, -2.0f, 4.0f, -2.0f, 4.0f, -0.5f, 6.0f, -0.5f,
		 6.0f, -2.0f, 12.0f, -2.0f, 12.0f, 2.0f, 6.0f, 2.0f,
		 6.0f, 0.5f, 4.0f, 0.5f, 4.0f, 2.0f, -1.0f, 2.0f},
		// Disconnected region.
		{16.0f, -2.0f, 20.0f, -2.0f, 20.0f, 2.0f, 16.0f, 2.0f},
	};

	BuiltNavMesh built;
	REQUIRE(buildNavMesh(slabs, 0, built));
	REQUIRE(built.query != 0);

	dtQueryFilter filter;

	const float startPos[3] = {0.0f, 0.1f, 0.0f};
	dtPolyRef startRef = 0;
	REQUIRE(findPolyAt(*built.query, startPos, startRef));

	const float farPos[3] = {9.0f, 0.1f, 0.0f};
	const float isolatedPos[3] = {18.0f, 0.1f, 0.0f};
	dtPolyRef farRef = 0, isolatedRef = 0;
	REQUIRE(findPolyAt(*built.query, farPos, farRef));
	REQUIRE(findPolyAt(*built.query, isolatedPos, isolatedRef));
	REQUIRE(farRef != isolatedRef);

	// Sanity check: the two regions really are disconnected at the mesh level.
	dtPolyRef probePath[64];
	int probeCount = 0;
	const dtStatus probeStatus = built.query->findPath(
		startRef, isolatedRef, startPos, isolatedPos, &filter,
		probePath, &probeCount, 64);
	REQUIRE(dtStatusDetail(probeStatus, DT_PARTIAL_RESULT));

	// The isolated target is listed first: the search must cross the neck
	// boundary and still select the reachable target on the far region.
	const dtPolyRef targetRefs[2] = {isolatedRef, farRef};
	const float targetPositions[][3] = {
		{18.0f, 0.1f, 0.0f},
		{9.0f, 0.1f, 0.0f},
	};

	int nearestIndex = -2;
	dtPolyRef path[64];
	int pathCount = 0;
	float nearestCost = -1.0f;
	const dtStatus status = built.query->findNearestReachablePoly(
		startRef, startPos, targetRefs, &targetPositions[0][0], 2, &filter,
		&nearestIndex, path, &pathCount, 64, &nearestCost);

	REQUIRE(dtStatusSucceed(status));
	REQUIRE_FALSE(dtStatusDetail(status, DT_PARTIAL_RESULT));
	REQUIRE(nearestIndex == 1);
	REQUIRE(path[0] == startRef);
	REQUIRE(path[pathCount - 1] == farRef);
	REQUIRE(nearestCost > 0.0f);

	// The corridor must really leave the start region and enter the far region.
	bool entersFarRegion = false;
	for (int i = 0; i < pathCount; ++i)
		if (path[i] == farRef)
			entersFarRegion = true;
	REQUIRE(entersFarRegion);
}

TEST_CASE("findNearestReachablePoly honours area costs when ranking candidates")
{
	// Two targets at x = -8 and x = 8. The route to the right-hand target
	// crosses a high cost area, so inflating that area's cost must make the
	// left-hand target win regardless of the exact tessellation-derived path
	// lengths.
	const std::vector<Loop> slabs = {
		{-10.0f, -1.0f, 10.0f, -1.0f, 10.0f, 1.0f, -10.0f, 1.0f},
	};
	// Convex quad on the ground covering the route to the right-hand target.
	const float expensiveArea[12] = {
		1.0f, 0.0f, -1.0f,
		7.0f, 0.0f, -1.0f,
		7.0f, 0.0f, 1.0f,
		1.0f, 0.0f, 1.0f,
	};

	BuiltNavMesh built;
	REQUIRE(buildNavMesh(slabs, expensiveArea, built));
	REQUIRE(built.query != 0);

	const float startPos[3] = {0.0f, 0.1f, 0.0f};
	dtPolyRef startRef = 0;
	REQUIRE(findPolyAt(*built.query, startPos, startRef));

	const float leftPos[3] = {-8.0f, 0.1f, 0.0f};
	const float rightPos[3] = {8.0f, 0.1f, 0.0f};
	dtPolyRef leftRef = 0, rightRef = 0;
	REQUIRE(findPolyAt(*built.query, leftPos, leftRef));
	REQUIRE(findPolyAt(*built.query, rightPos, rightRef));

	// Sanity check: the right-hand corridor really traverses the expensive area.
	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	bool routeTouchesExpensiveArea = false;
	dtQueryFilter probeFilter;
	dtPolyRef probePath[64];
	int probeCount = 0;
	REQUIRE(dtStatusSucceed(built.query->findPath(
		startRef, rightRef, startPos, rightPos, &probeFilter,
		probePath, &probeCount, 64)));
	for (int i = 0; i < probeCount; ++i)
	{
		built.navMesh->getTileAndPolyByRef(probePath[i], &tile, &poly);
		if (poly->getArea() == AREA_EXPENSIVE)
			routeTouchesExpensiveArea = true;
	}
	REQUIRE(routeTouchesExpensiveArea);

	// Measure each target independently first, under both cost models.
	auto costOf = [&](const dtQueryFilter& f, dtPolyRef target, const float* targetPos) {
		int idx = -2;
		float cost = -1.0f;
		const dtStatus st = built.query->findNearestReachablePoly(
			startRef, startPos, &target, targetPos, 1, &f, &idx, 0, 0, 0, &cost);
		REQUIRE(dtStatusSucceed(st));
		REQUIRE(idx == 0);
		return cost;
	};

	dtQueryFilter uniformFilter;
	const float uniformLeft = costOf(uniformFilter, leftRef, leftPos);
	const float uniformRight = costOf(uniformFilter, rightRef, rightPos);

	dtQueryFilter expensiveFilter;
	expensiveFilter.setAreaCost(AREA_EXPENSIVE, 50.0f);
	const float expensiveLeft = costOf(expensiveFilter, leftRef, leftPos);
	const float expensiveRight = costOf(expensiveFilter, rightRef, rightPos);

	// The left route never enters the expensive area so its cost is unchanged;
	// the right route becomes substantially more expensive.
	REQUIRE(expensiveLeft == Catch::Approx(uniformLeft));
	REQUIRE(expensiveRight > uniformRight * 10.0f);

	// When both candidates are offered in one query with the expensive filter,
	// the left target must be selected even though it is listed second.
	const dtPolyRef targetRefs[2] = {rightRef, leftRef};
	const float targetPositions[][3] = {
		{8.0f, 0.1f, 0.0f},
		{-8.0f, 0.1f, 0.0f},
	};
	int nearestIndex = -2;
	float nearestCost = -1.0f;
	const dtStatus status = built.query->findNearestReachablePoly(
		startRef, startPos, targetRefs, &targetPositions[0][0], 2, &expensiveFilter,
		&nearestIndex, 0, 0, 0, &nearestCost);
	REQUIRE(dtStatusSucceed(status));
	REQUIRE(nearestIndex == 1);
	REQUIRE(nearestCost == Catch::Approx(expensiveLeft));
}

TEST_CASE("findNearestReachablePoly breaks equal-cost ties by input order")
{
	// Square slab. The two candidate positions at (+3, 0) and (-3, 0) resolve to
	// the same polygon as the start position and are symmetric around it, so the
	// traversal corridor and the end segment have identical lengths and the full
	// costs are exactly equal. This isolates the tie-break rule from any
	// tessellation-driven cost differences.
	const std::vector<Loop> slabs = {
		{-6.0f, -6.0f, 6.0f, -6.0f, 6.0f, 6.0f, -6.0f, 6.0f},
	};

	BuiltNavMesh built;
	REQUIRE(buildNavMesh(slabs, 0, built));
	REQUIRE(built.query != 0);

	dtQueryFilter filter;

	const float startPos[3] = {0.0f, 0.1f, 0.0f};
	dtPolyRef startRef = 0;
	REQUIRE(findPolyAt(*built.query, startPos, startRef));

	const float rightPos[3] = {3.0f, 0.1f, 0.0f};
	const float leftPos[3] = {-3.0f, 0.1f, 0.0f};
	dtPolyRef rightRef = 0, leftRef = 0;
	REQUIRE(findPolyAt(*built.query, rightPos, rightRef));
	REQUIRE(findPolyAt(*built.query, leftPos, leftRef));

	// The construction must really yield one shared target polygon, otherwise
	// the two candidates would not be guaranteed equal cost.
	REQUIRE(rightRef == leftRef);
	REQUIRE(rightRef == startRef);

	SECTION("Swapping input order flips the selected equal-cost target")
	{
		const dtPolyRef targetRefs[2] = {leftRef, rightRef};
		const float targetPositions[][3] = {
			{-3.0f, 0.1f, 0.0f},
			{3.0f, 0.1f, 0.0f},
		};

		int nearestIndex = -2;
		float costA = -1.0f, costB = -1.0f;
		const dtStatus status = built.query->findNearestReachablePoly(
			startRef, startPos, targetRefs, &targetPositions[0][0], 2, &filter,
			&nearestIndex, 0, 0, 0, &costA);
		REQUIRE(dtStatusSucceed(status));
		REQUIRE(nearestIndex == 0);

		const dtPolyRef swappedRefs[2] = {rightRef, leftRef};
		const float swappedPositions[][3] = {
			{3.0f, 0.1f, 0.0f},
			{-3.0f, 0.1f, 0.0f},
		};
		int swappedIndex = -2;
		const dtStatus swappedStatus = built.query->findNearestReachablePoly(
			startRef, startPos, swappedRefs, &swappedPositions[0][0], 2, &filter,
			&swappedIndex, 0, 0, 0, &costB);
		REQUIRE(dtStatusSucceed(swappedStatus));
		REQUIRE(swappedIndex == 0);
		REQUIRE(costA == Catch::Approx(costB));
	}

	SECTION("Duplicate references resolve to the first occurrence")
	{
		const dtPolyRef targetRefs[3] = {rightRef, leftRef, rightRef};
		const float targetPositions[][3] = {
			{3.0f, 0.1f, 0.0f},
			{-3.0f, 0.1f, 0.0f},
			{3.0f, 0.1f, 0.0f},
		};

		int nearestIndex = -2;
		const dtStatus status = built.query->findNearestReachablePoly(
			startRef, startPos, targetRefs, &targetPositions[0][0], 3, &filter,
			&nearestIndex, 0, 0, 0, 0);
		REQUIRE(dtStatusSucceed(status));
		REQUIRE(nearestIndex == 0);
	}
}

TEST_CASE("findNearestReachablePoly reports partial result when no target is reachable")
{
	// Two disconnected slabs: every candidate is on the second slab.
	const std::vector<Loop> slabs = {
		{-1.0f, -1.0f, 3.0f, -1.0f, 3.0f, 1.0f, -1.0f, 1.0f},
		{8.0f, -1.0f, 12.0f, -1.0f, 12.0f, 1.0f, 8.0f, 1.0f},
	};

	BuiltNavMesh built;
	REQUIRE(buildNavMesh(slabs, 0, built));
	REQUIRE(built.query != 0);

	dtQueryFilter filter;

	const float startPos[3] = {0.0f, 0.1f, 0.0f};
	dtPolyRef startRef = 0;
	REQUIRE(findPolyAt(*built.query, startPos, startRef));

	const float targetA[3] = {9.0f, 0.1f, 0.0f};
	const float targetB[3] = {11.0f, 0.1f, 0.0f};
	dtPolyRef targetRefs[2] = {0, 0};
	REQUIRE(findPolyAt(*built.query, targetA, targetRefs[0]));
	REQUIRE(findPolyAt(*built.query, targetB, targetRefs[1]));

	const float targetPositions[][3] = {
		{9.0f, 0.1f, 0.0f},
		{11.0f, 0.1f, 0.0f},
	};

	SECTION("Unreachable candidates return the partial-result flag")
	{
		int nearestIndex = -2;
		dtPolyRef path[16];
		int pathCount = 7;
		float nearestCost = 123.0f;
		const dtStatus status = built.query->findNearestReachablePoly(
			startRef, startPos, targetRefs, &targetPositions[0][0], 2, &filter,
			&nearestIndex, path, &pathCount, 16, &nearestCost);

		REQUIRE(dtStatusSucceed(status));
		REQUIRE(dtStatusDetail(status, DT_PARTIAL_RESULT));
		REQUIRE(nearestIndex == -1);
		REQUIRE(pathCount == 0);
		REQUIRE(nearestCost == 123.0f); // untouched when nothing is reachable
	}

	SECTION("Filtering out every target yields the same unreachable result")
	{
		dtQueryFilter blockedFilter;
		blockedFilter.setIncludeFlags(0);

		int nearestIndex = -2;
		const dtStatus status = built.query->findNearestReachablePoly(
			startRef, startPos, targetRefs, &targetPositions[0][0], 2, &blockedFilter,
			&nearestIndex, 0, 0, 0, 0);

		REQUIRE(dtStatusSucceed(status));
		REQUIRE(dtStatusDetail(status, DT_PARTIAL_RESULT));
		REQUIRE(nearestIndex == -1);
	}

	SECTION("Invalid parameters are rejected")
	{
		int nearestIndex = -2;
		REQUIRE(dtStatusFailed(built.query->findNearestReachablePoly(
			startRef, startPos, 0, &targetPositions[0][0], 2, &filter,
			&nearestIndex, 0, 0, 0, 0)));

		REQUIRE(dtStatusFailed(built.query->findNearestReachablePoly(
			startRef, startPos, targetRefs, &targetPositions[0][0], 0, &filter,
			&nearestIndex, 0, 0, 0, 0)));

		REQUIRE(dtStatusFailed(built.query->findNearestReachablePoly(
			startRef, startPos, targetRefs, &targetPositions[0][0], 2, &filter,
			0, 0, 0, 0, 0)));

		dtPolyRef smallPath[1];
		int pathCount = 0;
		REQUIRE(dtStatusFailed(built.query->findNearestReachablePoly(
			startRef, startPos, targetRefs, &targetPositions[0][0], 2, &filter,
			&nearestIndex, smallPath, &pathCount, 0, 0)));
	}
}

TEST_CASE("findNearestReachablePoly handles the start polygon and invalid candidate entries")
{
	const std::vector<Loop> slabs = {
		{-1.0f, -1.0f, 16.0f, -1.0f, 16.0f, 1.0f, -1.0f, 1.0f},
	};

	BuiltNavMesh built;
	REQUIRE(buildNavMesh(slabs, 0, built));
	REQUIRE(built.query != 0);

	dtQueryFilter filter;

	const float startPos[3] = {0.0f, 0.1f, 0.0f};
	dtPolyRef startRef = 0;
	REQUIRE(findPolyAt(*built.query, startPos, startRef));

	const float farPos[3] = {15.0f, 0.1f, 0.0f};
	dtPolyRef farRef = 0;
	REQUIRE(findPolyAt(*built.query, farPos, farRef));

	SECTION("The start polygon itself is a zero-cost reachable candidate")
	{
		// The far target is listed first; the start polygon must still win
		// because reaching it costs nothing.
		const dtPolyRef targetRefs[2] = {farRef, startRef};
		const float targetPositions[][3] = {
			{15.0f, 0.1f, 0.0f},
			{0.0f, 0.1f, 0.0f},
		};

		int nearestIndex = -2;
		dtPolyRef path[16];
		int pathCount = 0;
		float nearestCost = -1.0f;
		const dtStatus status = built.query->findNearestReachablePoly(
			startRef, startPos, targetRefs, &targetPositions[0][0], 2, &filter,
			&nearestIndex, path, &pathCount, 16, &nearestCost);

		REQUIRE(dtStatusSucceed(status));
		REQUIRE_FALSE(dtStatusDetail(status, DT_PARTIAL_RESULT));
		REQUIRE(nearestIndex == 1);
		REQUIRE(nearestCost == Catch::Approx(0.0f));
		REQUIRE(pathCount == 1);
		REQUIRE(path[0] == startRef);
	}

	SECTION("Invalid entries are skipped while valid ones are still evaluated")
	{
		// Entry 0 is a null reference and entry 2 repeats the far target; both
		// remaining real candidates must be evaluated normally and the nearer
		// one selected.
		const float nearPos[3] = {4.0f, 0.1f, 0.0f};
		dtPolyRef nearRef = 0;
		REQUIRE(findPolyAt(*built.query, nearPos, nearRef));

		const dtPolyRef targetRefs[4] = {0, farRef, nearRef, farRef};
		const float targetPositions[][3] = {
			{0.0f, 0.1f, 0.0f},   // unused, pairs with the null reference
			{15.0f, 0.1f, 0.0f},
			{4.0f, 0.1f, 0.0f},
			{15.0f, 0.1f, 0.0f},
		};

		int nearestIndex = -2;
		const dtStatus status = built.query->findNearestReachablePoly(
			startRef, startPos, targetRefs, &targetPositions[0][0], 4, &filter,
			&nearestIndex, 0, 0, 0, 0);

		REQUIRE(dtStatusSucceed(status));
		REQUIRE_FALSE(dtStatusDetail(status, DT_PARTIAL_RESULT));
		REQUIRE(nearestIndex == 2);
	}

	SECTION("A single surviving valid candidate is selected among bad entries")
	{
		const dtPolyRef targetRefs[3] = {0, farRef, 0};
		const float targetPositions[][3] = {
			{0.0f, 0.1f, 0.0f},
			{15.0f, 0.1f, 0.0f},
			{0.0f, 0.1f, 0.0f},
		};

		int nearestIndex = -2;
		dtPolyRef path[16];
		int pathCount = 0;
		const dtStatus status = built.query->findNearestReachablePoly(
			startRef, startPos, targetRefs, &targetPositions[0][0], 3, &filter,
			&nearestIndex, path, &pathCount, 16, 0);

		REQUIRE(dtStatusSucceed(status));
		REQUIRE(nearestIndex == 1);
		REQUIRE(path[pathCount - 1] == farRef);
	}
}

TEST_CASE("findNearestReachablePoly flags an undersized path buffer without losing the result")
{
	const std::vector<Loop> slabs = {
		{-1.0f, -1.0f, 16.0f, -1.0f, 16.0f, 1.0f, -1.0f, 1.0f},
	};

	BuiltNavMesh built;
	REQUIRE(buildNavMesh(slabs, 0, built));
	REQUIRE(built.query != 0);

	dtQueryFilter filter;

	const float startPos[3] = {0.0f, 0.1f, 0.0f};
	dtPolyRef startRef = 0;
	REQUIRE(findPolyAt(*built.query, startPos, startRef));

	const float targetPos[3] = {15.0f, 0.1f, 0.0f};
	dtPolyRef targetRef = 0;
	REQUIRE(findPolyAt(*built.query, targetPos, targetRef));

	int nearestIndex = -2;
	dtPolyRef path[2];
	int pathCount = 0;
	const dtStatus status = built.query->findNearestReachablePoly(
		startRef, startPos, &targetRef, targetPos, 1, &filter,
		&nearestIndex, path, &pathCount, 2, 0);

	REQUIRE(dtStatusSucceed(status));
	REQUIRE(dtStatusDetail(status, DT_BUFFER_TOO_SMALL));
	REQUIRE(nearestIndex == 0);
	REQUIRE(pathCount == 2);
	REQUIRE(path[0] == startRef);
}
