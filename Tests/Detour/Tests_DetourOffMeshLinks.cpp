// Tests for off-mesh connections spanning non-adjacent (remote) tiles.
//
// A flat world is split into 10x10 tiles. Two ground tiles are placed in
// cells which do NOT touch each other, and an off-mesh connection jumps
// from one cell to the other. The regular 8-neighbour hookup cannot create
// these links; the remote off-mesh connection code must.

#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourCommon.h"
#include "catch2/catch_amalgamated.hpp"

#include <vector>

namespace
{

const int TILE = 10;          // Tile size in world units.
const float AGENT_RADIUS = 0.5f;

struct FlatTile
{
	std::vector<unsigned char> data;
	int dataSize = 0;
};

// Builds tile data for one flat quad covering the cell (tx, ty).
// offMeshConVerts may be null; otherwise the first endpoint of every
// connection must lie inside this tile so the connection is stored here.
FlatTile buildFlatTile(int tx, int ty,
					   const float* offMeshConVerts, int offMeshConCount,
					   const unsigned char* offMeshConDir,
					   const float* offMeshConRad,
					   const unsigned int* offMeshConUserID,
					   int layer = 0)
{
	const float bmin[3] = { (float)tx * TILE, 0.0f, (float)ty * TILE };
	const float bmax[3] = { (float)(tx + 1) * TILE, 1.0f, (float)(ty + 1) * TILE };

	// Four corners of the quad, in Recast voxel coordinates relative to bmin.
	const unsigned short verts[4 * 3] =
	{
		0, 0, 0,
		TILE, 0, 0,
		TILE, 0, TILE,
		0, 0, TILE,
	};

	// One quad polygon. All four edges are hard borders (0xf).
	const int nvp = DT_VERTS_PER_POLYGON;
	unsigned short polys[2 * nvp];
	for (int i = 0; i < 2 * nvp; ++i) polys[i] = 0;
	polys[0] = 0; polys[1] = 1; polys[2] = 2; polys[3] = 3;
	for (int i = 0; i < 4; ++i)
		polys[nvp + i] = 0x8000 | 0xf; // Border edge.

	unsigned short polyFlags[1] = { 1 };
	unsigned char polyAreas[1] = { 0 };

	// The builder dereferences these arrays unconditionally when at least one
	// off-mesh connection is stored, so they must always be valid.
	const unsigned short conPolyFlags[1] = { 1 };
	const unsigned char conPolyAreas[1] = { 0 };
	const unsigned char localConDir[1] = { DT_OFFMESH_CON_BIDIR };
	const float localConRad[1] = { 1.0f };
	const unsigned int localConUserId[1] = { 0 };
	const unsigned char* dirs = offMeshConDir ? offMeshConDir : localConDir;
	const float* rads = offMeshConRad ? offMeshConRad : localConRad;
	const unsigned int* userIds = offMeshConUserID ? offMeshConUserID : localConUserId;

	dtNavMeshCreateParams params;
	memset(&params, 0, sizeof(params));
	params.verts = verts;
	params.vertCount = 4;
	params.polys = polys;
	params.polyFlags = polyFlags;
	params.polyAreas = polyAreas;
	params.polyCount = 1;
	params.nvp = nvp;
	params.userId = 0;
	params.tileX = tx;
	params.tileY = ty;
	params.tileLayer = layer;
	dtVcopy(params.bmin, bmin);
	dtVcopy(params.bmax, bmax);
	params.walkableHeight = 2.0f;
	params.walkableRadius = AGENT_RADIUS;
	params.walkableClimb = 0.5f;
	params.cs = 1.0f;
	params.ch = 1.0f;
	params.buildBvTree = true;

	params.offMeshConVerts = offMeshConVerts;
	params.offMeshConCount = offMeshConCount;
	params.offMeshConRad = rads;
	params.offMeshConDir = dirs;
	params.offMeshConFlags = conPolyFlags;
	params.offMeshConAreas = conPolyAreas;
	params.offMeshConUserID = userIds;

	unsigned char* outData = 0;
	int outSize = 0;
	REQUIRE(dtCreateNavMeshData(&params, &outData, &outSize));

	FlatTile tile;
	tile.data.assign(outData, outData + outSize);
	tile.dataSize = outSize;
	dtFree(outData);
	return tile;
}

dtNavMesh* createNavMesh()
{
	dtNavMesh* nav = dtAllocNavMesh();
	REQUIRE(nav);
	dtNavMeshParams params;
	memset(&params, 0, sizeof(params));
	params.orig[0] = 0.0f;
	params.orig[1] = 0.0f;
	params.orig[2] = 0.0f;
	params.tileWidth = (float)TILE;
	params.tileHeight = (float)TILE;
	params.maxTiles = 64;
	params.maxPolys = 16;
	REQUIRE(dtStatusSucceed(nav->init(&params)));
	return nav;
}

dtTileRef addFlatTile(dtNavMesh* nav, const FlatTile& tile)
{
	dtTileRef ref = 0;
	// The navmesh takes ownership and frees the copy via DT_TILE_FREE_DATA.
	unsigned char* copy = (unsigned char*)dtAlloc(tile.dataSize, DT_ALLOC_PERM);
	REQUIRE(copy);
	memcpy(copy, tile.data.data(), tile.dataSize);
	REQUIRE(dtStatusSucceed(nav->addTile(copy, tile.dataSize, DT_TILE_FREE_DATA, 0, &ref)));
	return ref;
}

// Adds tile data without transferring ownership: the navmesh references the
// FlatTile's storage directly, so removeTile() hands the same pointer back.
// This models streamed tiles which are reloaded from an external cache.
dtTileRef addBorrowedTile(dtNavMesh* nav, FlatTile& tile, dtTileRef lastRef = 0)
{
	dtTileRef ref = 0;
	REQUIRE(dtStatusSucceed(
		nav->addTile(tile.data.data(), tile.dataSize, 0, lastRef, &ref)));
	return ref;
}

// Counts links owned by this tile whose referenced polygon lives in the
// given other tile.
int countLinksToTile(const dtNavMesh* nav, const dtMeshTile* from, const dtMeshTile* to)
{
	const unsigned int targetTileNum =
		nav->decodePolyIdTile(nav->getTileRef(to));
	int count = 0;
	for (int i = 0; i < from->header->polyCount; ++i)
	{
		const dtPoly* poly = &from->polys[i];
		for (unsigned int k = poly->firstLink; k != DT_NULL_LINK; k = from->links[k].next)
		{
			if (nav->decodePolyIdTile(from->links[k].ref) == targetTileNum)
				++count;
		}
	}
	return count;
}

// Runs an A* path between the two world positions and reports the polygon
// path. Returns false when no path exists.
bool findPolyPath(dtNavMesh* nav, const float start[3], const float end[3],
				  std::vector<dtPolyRef>* path)
{
	dtNavMeshQuery* query = dtAllocNavMeshQuery();
	REQUIRE(query);
	REQUIRE(dtStatusSucceed(query->init(nav, 2048)));

	dtPolyRef startRef = 0, endRef = 0;
	float nearestStart[3], nearestEnd[3];
	const float ext[3] = { 2.0f, 4.0f, 2.0f };
	const dtQueryFilter defaultFilter;
	// A missing endpoint polygon (e.g. the landing tile was removed) means no
	// path exists; report failure instead of aborting the test case.
	if (dtStatusFailed(query->findNearestPoly(start, ext, &defaultFilter,
											  &startRef, nearestStart)) ||
		dtStatusFailed(query->findNearestPoly(end, ext, &defaultFilter,
											  &endRef, nearestEnd)) ||
		startRef == 0 || endRef == 0)
	{
		dtFreeNavMeshQuery(query);
		return false;
	}

	dtPolyRef rawPath[256];
	int pathCount = 0;
	const dtStatus status = query->findPath(startRef, endRef, nearestStart,
											nearestEnd, &defaultFilter, rawPath, &pathCount, 256);
	dtFreeNavMeshQuery(query);

	if (dtStatusFailed(status) || pathCount == 0 || rawPath[pathCount - 1] != endRef)
		return false;

	path->assign(rawPath, rawPath + pathCount);
	return true;
}

// Returns true when the polygon path traverses an off-mesh connection poly.
bool pathUsesOffMeshConnection(dtNavMesh* nav, const std::vector<dtPolyRef>& path)
{
	for (size_t i = 0; i < path.size(); ++i)
	{
		const dtMeshTile* tile = 0;
		const dtPoly* poly = 0;
		nav->getTileAndPolyByRef(path[i], &tile, &poly);
		if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
			return true;
	}
	return false;
}

} // namespace

TEST_CASE("Remote off-mesh connection between non-adjacent tiles")
{
	// Cell (0,0) and cell (3,0) are separated by two empty cells, so they
	// are not neighbours. The connection start lives in tile A, the landing
	// point in tile B.
	const int ax = 0, ay = 0;
	const int bx = 3, by = 0;

	const float startPos[3] = { 2.0f, 0.0f, 2.0f };  // Inside tile A.
	const float endPos[3]   = { 32.0f, 0.0f, 2.0f }; // Inside tile B (30..40).
	const float conVerts[6] =
	{
		startPos[0], startPos[1], startPos[2],
		endPos[0], endPos[1], endPos[2],
	};
	const float conRad[1] = { 1.0f };
	const unsigned char conDirBidir[1] = { DT_OFFMESH_CON_BIDIR };
	const unsigned char conDirOneWay[1] = { 0 };
	const unsigned int conUserId[1] = { 42 };

	dtNavMesh* nav = createNavMesh();

	SECTION("Forward connection is created once the remote landing tile exists")
	{
		// Add only tile A first: the connection has nowhere to land yet.
		FlatTile tileA = buildFlatTile(ax, ay, conVerts, 1, conDirOneWay,
									   conRad, conUserId);
		dtTileRef refA = addFlatTile(nav, tileA);
		const dtMeshTile* a = nav->getTileByRef(refA);
		REQUIRE(a->header->offMeshConCount == 1);

		// Before tile B exists there can be no link out of the connection.
		const dtPoly* conPoly = &a->polys[a->header->offMeshBase];
		bool hasExternalLanding = false;
		for (unsigned int k = conPoly->firstLink; k != DT_NULL_LINK; k = a->links[k].next)
		{
			if (a->links[k].edge == 1)
				hasExternalLanding = true;
		}
		REQUIRE_FALSE(hasExternalLanding);

		// Add the remote tile B; the dangling connection must now connect.
		FlatTile tileB = buildFlatTile(bx, by, 0, 0, 0, 0, 0);
		dtTileRef refB = addFlatTile(nav, tileB);
		const dtMeshTile* b = nav->getTileByRef(refB);

		REQUIRE(countLinksToTile(nav, a, b) >= 1);

		// A* must now find a path and it must traverse the off-mesh poly.
		std::vector<dtPolyRef> path;
		REQUIRE(findPolyPath(nav, startPos, endPos, &path));
		REQUIRE(pathUsesOffMeshConnection(nav, path));

		dtFreeNavMesh(nav);
	}

	SECTION("Bidirectional connection creates the reverse link as well")
	{
		FlatTile tileA = buildFlatTile(ax, ay, conVerts, 1, conDirBidir,
									   conRad, conUserId);
		FlatTile tileB = buildFlatTile(bx, by, 0, 0, 0, 0, 0);
		dtTileRef refA = addFlatTile(nav, tileA);
		dtTileRef refB = addFlatTile(nav, tileB);
		const dtMeshTile* a = nav->getTileByRef(refA);
		const dtMeshTile* b = nav->getTileByRef(refB);

		// Both directions: A owns the connection->B link, B owns the
		// landing-poly->connection link.
		REQUIRE(countLinksToTile(nav, a, b) >= 1);
		REQUIRE(countLinksToTile(nav, b, a) >= 1);

		// Forward pathfinding (A -> B).
		std::vector<dtPolyRef> forwardPath;
		REQUIRE(findPolyPath(nav, startPos, endPos, &forwardPath));
		REQUIRE(pathUsesOffMeshConnection(nav, forwardPath));

		// Reverse pathfinding (B -> A) must also use the same connection.
		std::vector<dtPolyRef> reversePath;
		REQUIRE(findPolyPath(nav, endPos, startPos, &reversePath));
		REQUIRE(pathUsesOffMeshConnection(nav, reversePath));

		dtFreeNavMesh(nav);
	}

	SECTION("One-way connection cannot be traversed in reverse")
	{
		FlatTile tileA = buildFlatTile(ax, ay, conVerts, 1, conDirOneWay,
									   conRad, conUserId);
		FlatTile tileB = buildFlatTile(bx, by, 0, 0, 0, 0, 0);
		addFlatTile(nav, tileA);
		addFlatTile(nav, tileB);

		// Forward works.
		std::vector<dtPolyRef> forwardPath;
		REQUIRE(findPolyPath(nav, startPos, endPos, &forwardPath));
		REQUIRE(pathUsesOffMeshConnection(nav, forwardPath));

		// Reverse must fail: there is no ground path between the cells.
		std::vector<dtPolyRef> reversePath;
		REQUIRE_FALSE(findPolyPath(nav, endPos, startPos, &reversePath));

		dtFreeNavMesh(nav);
	}

	SECTION("Reloading the remote landing tile restores the connection")
	{
		FlatTile tileA = buildFlatTile(ax, ay, conVerts, 1, conDirBidir,
									   conRad, conUserId);
		FlatTile tileB = buildFlatTile(bx, by, 0, 0, 0, 0, 0);
		// Borrowed storage models an external tile cache: removeTile() must
		// return the same pointer so the tile can be streamed back in.
		dtTileRef refA = addBorrowedTile(nav, tileA);
		dtTileRef refB = addBorrowedTile(nav, tileB);
		const dtMeshTile* a = nav->getTileByRef(refA);

		// Sanity: path works before reload.
		std::vector<dtPolyRef> pathBefore;
		REQUIRE(findPolyPath(nav, startPos, endPos, &pathBefore));

		// Remove tile B. The remote links must be pruned from tile A.
		unsigned char* removedData = 0;
		int removedSize = 0;
		REQUIRE(dtStatusSucceed(nav->removeTile(refB, &removedData, &removedSize)));
		REQUIRE(removedData == tileB.data.data());
		REQUIRE(removedSize == tileB.dataSize);

		// No reachable path while the landing tile is gone.
		std::vector<dtPolyRef> pathWhileGone;
		REQUIRE_FALSE(findPolyPath(nav, startPos, endPos, &pathWhileGone));

		// The connection poly must have lost its external landing link.
		const dtPoly* conPoly = &a->polys[a->header->offMeshBase];
		for (unsigned int k = conPoly->firstLink; k != DT_NULL_LINK; k = a->links[k].next)
			REQUIRE(a->links[k].edge != 1);

		// Reload tile B, reusing the same tile reference (typical streaming).
		dtTileRef refB2 = addBorrowedTile(nav, tileB, refB);
		REQUIRE(refB2 == refB);

		const dtMeshTile* a2 = nav->getTileByRef(refA);
		const dtMeshTile* b2 = nav->getTileByRef(refB2);
		REQUIRE(countLinksToTile(nav, a2, b2) >= 1);
		REQUIRE(countLinksToTile(nav, b2, a2) >= 1);

		// Both directions work again after reload.
		std::vector<dtPolyRef> pathAfter;
		REQUIRE(findPolyPath(nav, startPos, endPos, &pathAfter));
		REQUIRE(pathUsesOffMeshConnection(nav, pathAfter));

		std::vector<dtPolyRef> reverseAfter;
		REQUIRE(findPolyPath(nav, endPos, startPos, &reverseAfter));
		REQUIRE(pathUsesOffMeshConnection(nav, reverseAfter));

		dtFreeNavMesh(nav);
	}

	SECTION("Reloading the connection-owning tile reconnects to the remote tile")
	{
		FlatTile tileA = buildFlatTile(ax, ay, conVerts, 1, conDirBidir,
									   conRad, conUserId);
		FlatTile tileB = buildFlatTile(bx, by, 0, 0, 0, 0, 0);
		dtTileRef refA = addBorrowedTile(nav, tileA);
		dtTileRef refB = addBorrowedTile(nav, tileB);

		// Reload tile A (the owner of the off-mesh connection).
		unsigned char* removedData = 0;
		int removedSize = 0;
		REQUIRE(dtStatusSucceed(nav->removeTile(refA, &removedData, &removedSize)));
		REQUIRE(removedData == tileA.data.data());

		dtTileRef refA2 = addBorrowedTile(nav, tileA, refA);
		REQUIRE(refA2 == refA);

		const dtMeshTile* a2 = nav->getTileByRef(refA2);
		const dtMeshTile* b2 = nav->getTileByRef(refB);
		REQUIRE(countLinksToTile(nav, a2, b2) >= 1);
		REQUIRE(countLinksToTile(nav, b2, a2) >= 1);

		std::vector<dtPolyRef> path;
		REQUIRE(findPolyPath(nav, startPos, endPos, &path));
		REQUIRE(pathUsesOffMeshConnection(nav, path));

		dtFreeNavMesh(nav);
	}

	SECTION("Loading the landing tile later completes a previously dangling connection")
	{
		// Tile A (connection owner) exists while the remote landing cell is
		// still empty. Adding tile B afterwards must make the pre-existing
		// connection reachable. This exercises the existing-con -> new-tile
		// hookup path.
		FlatTile tileA = buildFlatTile(ax, ay, conVerts, 1, conDirBidir,
									   conRad, conUserId);
		dtTileRef refA = addFlatTile(nav, tileA);

		// No landing tile yet: no path.
		std::vector<dtPolyRef> pathBefore;
		REQUIRE_FALSE(findPolyPath(nav, startPos, endPos, &pathBefore));

		FlatTile tileB = buildFlatTile(bx, by, 0, 0, 0, 0, 0);
		dtTileRef refB = addFlatTile(nav, tileB);
		const dtMeshTile* a = nav->getTileByRef(refA);
		const dtMeshTile* b = nav->getTileByRef(refB);
		REQUIRE(countLinksToTile(nav, a, b) >= 1);
		REQUIRE(countLinksToTile(nav, b, a) >= 1);

		std::vector<dtPolyRef> pathAfter;
		REQUIRE(findPolyPath(nav, startPos, endPos, &pathAfter));
		REQUIRE(pathUsesOffMeshConnection(nav, pathAfter));

		dtFreeNavMesh(nav);
	}

	SECTION("Adding a second layer at the landing cell does not duplicate links")
	{
		// Layer 0 of the landing cell is connected first. Adding layer 1 at
		// the same cell must add one set of links for the new layer only and
		// must never touch the already established layer-0 links.
		FlatTile tileA = buildFlatTile(ax, ay, conVerts, 1, conDirBidir,
									   conRad, conUserId);
		FlatTile tileB0 = buildFlatTile(bx, by, 0, 0, 0, 0, 0, 0);
		dtTileRef refA = addFlatTile(nav, tileA);
		dtTileRef refB0 = addFlatTile(nav, tileB0);

		const dtMeshTile* aBefore = nav->getTileByRef(refA);
		const dtMeshTile* b0Before = nav->getTileByRef(refB0);
		const int aToB0Before = countLinksToTile(nav, aBefore, b0Before);
		const int b0ToABefore = countLinksToTile(nav, b0Before, aBefore);
		REQUIRE(aToB0Before >= 1);
		REQUIRE(b0ToABefore >= 1);

		// Add a second layer at the same remote cell.
		FlatTile tileB1 = buildFlatTile(bx, by, 0, 0, 0, 0, 0, 1);
		dtTileRef refB1 = addFlatTile(nav, tileB1);
		const dtMeshTile* aAfter = nav->getTileByRef(refA);
		const dtMeshTile* b0After = nav->getTileByRef(refB0);
		const dtMeshTile* b1After = nav->getTileByRef(refB1);

		// The links towards the original layer must be unchanged (no dupes).
		REQUIRE(countLinksToTile(nav, aAfter, b0After) == aToB0Before);
		REQUIRE(countLinksToTile(nav, b0After, aAfter) == b0ToABefore);
		// And the new layer got its own set of links.
		REQUIRE(countLinksToTile(nav, aAfter, b1After) >= 1);
		REQUIRE(countLinksToTile(nav, b1After, aAfter) >= 1);

		dtFreeNavMesh(nav);
	}
}
