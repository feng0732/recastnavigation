#include "DetourCrowd.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourCommon.h"
#include "catch2/catch_amalgamated.hpp"

#include <string.h>
#include <vector>

namespace
{
	static const unsigned short WALKABLE = 0x01;
	static const unsigned short MESH_NULL_IDX_LOCAL = 0xffff;
	/// Description of one off-mesh connection used by the test mesh builder.
	struct TestOffMeshCon
	{
		float start[3];
		float end[3];
		unsigned short flags;	///< Carries the crowd connection type bits (or zero for legacy).
		unsigned char dir;		///< 0 = A->B only, DT_OFFMESH_CON_BIDIR = bidirectional.
		unsigned char area;
		unsigned int userId;
	};

	/// One serialized tile plus the owning navmesh, plus a query and crowd.
	/// The tile buffers are kept alive for the lifetime of the fixture so the
	/// navmesh can stream tiles out and back in.
	struct OffMeshFixture
	{
		static const int GROUND_POLYS = 2;
		static const int NVP = 6;

		struct Tile
		{
			int x;
			int y;
			unsigned char* data;	// Owned by the fixture via dtAlloc, never by the navmesh.
			int dataSize;
			dtTileRef ref;
			Tile() : x(0), y(0), data(0), dataSize(0), ref(0) {}
		};

		dtNavMesh* nav;
		dtNavMeshQuery* query;
		dtCrowd* crowd;
		std::vector<Tile> tiles;

		OffMeshFixture() : nav(0), query(0), crowd(0) {}

		~OffMeshFixture()
		{
			if (crowd)
				dtFreeCrowd(crowd);
			if (query)
				dtFreeNavMeshQuery(query);
			// Destroy the navmesh first (it does not own the tile buffers),
			// then release the fixture-owned buffers.
			if (nav)
				dtFreeNavMesh(nav);
			for (size_t i = 0; i < tiles.size(); ++i)
				dtFree(tiles[i].data);
		}

		/// Builds a flat two-polygon ground tile at tile grid coordinate (x, 0).
		/// Poly 0 covers x in [2x, 2x+1], poly 1 covers x in [2x+1, 2x+2],
		/// both spanning z in [0,1]. When @p joinedGround is false the shared
		/// edge is left as a border, so the two polygons can only be reached
		/// through an off-mesh connection (the common test configuration).
		/// On success *outData points to a dtAlloc-owned buffer the caller must free.
		void buildTileData(const int x, const std::vector<TestOffMeshCon>& cons,
						   unsigned char** outData, int* outSize,
						   const bool joinedGround = false) const
		{
			const float cs = 1.0f;
			const float ch = 1.0f;
			const int base = x * 2;

			// Voxel vertices are LOCAL to the tile origin; dtCreateNavMeshData
			// maps them to world space as bmin + iv*cs. The tile origin is set
			// via bmin below, so these indices must not include the tile offset.
			unsigned short verts[6*3];
			// v0..v5, all at height 0. Counter-clockwise winding when viewed from above.
			const unsigned short v[6][3] =
			{
				{0, 0, 0},
				{0, 0, 1},
				{1, 0, 1},
				{1, 0, 0},
				{2, 0, 1},
				{2, 0, 0}
			};
			for (int i = 0; i < 6*3; ++i)
				verts[i] = (&v[0][0])[i];

			// Two quads, CCW winding, geometrically sharing the edge at x = base+1.
			// Recast layout: each polygon block is 2*NVP entries, vertices then neighbours.
			unsigned short polys[GROUND_POLYS * NVP * 2];
			memset(polys, 0xff, sizeof(polys));
			// Poly 0: verts 0,1,2,3. Shared edge (2->3) is local edge 2.
			polys[0*2*NVP + 0] = 0;
			polys[0*2*NVP + 1] = 1;
			polys[0*2*NVP + 2] = 2;
			polys[0*2*NVP + 3] = 3;
			polys[0*2*NVP + NVP + 0] = MESH_NULL_IDX_LOCAL; // border
			polys[0*2*NVP + NVP + 1] = MESH_NULL_IDX_LOCAL;
			polys[0*2*NVP + NVP + 2] = joinedGround ? 1 : MESH_NULL_IDX_LOCAL;
			polys[0*2*NVP + NVP + 3] = MESH_NULL_IDX_LOCAL;
			// Poly 1: verts 3,4,5,1 (base+1,0)->(base+2,1)->(base+2,0)->(base+1,1).
			// Shared edge (1->3) is local edge 3.
			polys[1*2*NVP + 0] = 3;
			polys[1*2*NVP + 1] = 4;
			polys[1*2*NVP + 2] = 5;
			polys[1*2*NVP + 3] = 1;
			polys[1*2*NVP + NVP + 0] = MESH_NULL_IDX_LOCAL;
			polys[1*2*NVP + NVP + 1] = MESH_NULL_IDX_LOCAL;
			polys[1*2*NVP + NVP + 2] = MESH_NULL_IDX_LOCAL;
			polys[1*2*NVP + NVP + 3] = joinedGround ? 0 : MESH_NULL_IDX_LOCAL;

			unsigned short polyFlags[GROUND_POLYS] = { WALKABLE, WALKABLE };
			unsigned char polyAreas[GROUND_POLYS] = { 0, 0 };

			std::vector<float> conVerts(cons.size() * 6);
			std::vector<float> conRad(cons.size());
			std::vector<unsigned short> conFlags(cons.size());
			std::vector<unsigned char> conAreas(cons.size());
			std::vector<unsigned char> conDir(cons.size());
			std::vector<unsigned int> conUserId(cons.size());
			for (size_t i = 0; i < cons.size(); ++i)
			{
				dtVcopy(&conVerts[i*6+0], cons[i].start);
				dtVcopy(&conVerts[i*6+3], cons[i].end);
				// Generous radius so cross-tile landing endpoints resolve reliably.
				conRad[i] = 1.0f;
				conFlags[i] = cons[i].flags;
				conAreas[i] = cons[i].area;
				conDir[i] = cons[i].dir;
				conUserId[i] = cons[i].userId;
			}

			dtNavMeshCreateParams params;
			memset(&params, 0, sizeof(params));
			params.verts = verts;
			params.vertCount = 6;
			params.polys = polys;
			params.polyFlags = polyFlags;
			params.polyAreas = polyAreas;
			params.polyCount = GROUND_POLYS;
			params.nvp = NVP;
			params.detailMeshes = 0;
			params.detailVerts = 0;
			params.detailVertsCount = 0;
			params.detailTris = 0;
			params.detailTriCount = 0;
			params.offMeshConVerts = cons.empty() ? 0 : conVerts.data();
			params.offMeshConRad = cons.empty() ? 0 : conRad.data();
			params.offMeshConFlags = cons.empty() ? 0 : conFlags.data();
			params.offMeshConAreas = cons.empty() ? 0 : conAreas.data();
			params.offMeshConDir = cons.empty() ? 0 : conDir.data();
			params.offMeshConUserID = cons.empty() ? 0 : conUserId.data();
			params.offMeshConCount = (int)cons.size();
			params.walkableHeight = 2.0f;
			params.walkableRadius = 0.5f;
			params.walkableClimb = 0.9f;
			params.tileX = x;
			params.tileY = 0;
			params.tileLayer = 0;
			// Ground vertices have voxel y=0, so bmin.y must be 0 for the ground
			// plane to land at world y=0. The height cull range is widened by
			// walkableClimb and still contains the off-mesh endpoints at y=0.
			params.bmin[0] = (float)base;	params.bmin[1] = 0.0f;	params.bmin[2] = 0.0f;
			params.bmax[0] = (float)(base+2); params.bmax[1] = 1.0f;	params.bmax[2] = 1.0f;
			params.cs = cs;
			params.ch = ch;
			// A BV tree is required for reliable nearest-poly and overlap
			// queries against the zero-thickness flat ground polygons.
			params.buildBvTree = true;

			unsigned char* newData = 0;
			int newSize = 0;
			bool ok = dtCreateNavMeshData(&params, &newData, &newSize);
			REQUIRE(ok);
			REQUIRE(newData != 0);
			*outData = newData;
			*outSize = newSize;
		}

		void init(const int maxTiles, const std::vector<int>& tileXs,
				  const std::vector<std::vector<TestOffMeshCon> >& conPerTile,
				  const int maxAgents = 8,
				  const bool joinedGround = false)
		{
			dtNavMeshParams navParams;
			memset(&navParams, 0, sizeof(navParams));
			dtVset(navParams.orig, 0,0,0);
			navParams.tileWidth = 2.0f;
			navParams.tileHeight = 1.0f;
			navParams.maxTiles = maxTiles;
			navParams.maxPolys = 16;

			nav = dtAllocNavMesh();
			REQUIRE(nav);
			REQUIRE(dtStatusSucceed(nav->init(&navParams)));

			for (size_t t = 0; t < tileXs.size(); ++t)
			{
				Tile tile;
				tile.x = tileXs[t];
				tile.y = 0;
				buildTileData(tile.x, conPerTile[t], &tile.data, &tile.dataSize, joinedGround);
				tile.ref = 0;
				// Flag 0: the navmesh references the buffer but the fixture owns it.
				dtStatus status = nav->addTile(tile.data, tile.dataSize, 0, 0, &tile.ref);
				REQUIRE(dtStatusSucceed(status));
				tiles.push_back(tile);
			}

			query = dtAllocNavMeshQuery();
			REQUIRE(query);
			REQUIRE(dtStatusSucceed(query->init(nav, 1024)));

			crowd = dtAllocCrowd();
			REQUIRE(crowd);
			REQUIRE(crowd->init(maxAgents, 0.5f, nav));
		}

		dtPolyRef polyAt(const float* pos) const
		{
			dtPolyRef ref = 0;
			float nearest[3];
			const dtQueryFilter* filter = crowd->getFilter(0);
			query->findNearestPoly(pos, crowd->getQueryHalfExtents(), filter, &ref, nearest);
			return ref;
		}

		dtCrowdAgentParams defaultAgentParams() const
		{
			dtCrowdAgentParams p;
			memset(&p, 0, sizeof(p));
			p.radius = 0.3f;
			p.height = 1.8f;
			p.maxAcceleration = 8.0f;
			p.maxSpeed = 2.0f;
			p.collisionQueryRange = 3.0f;
			p.pathOptimizationRange = 3.0f;
			p.separationWeight = 1.0f;
			p.updateFlags = DT_CROWD_ANTICIPATE_TURNS;
			p.obstacleAvoidanceType = 0;
			p.queryFilterType = 0;
			p.allowedOffMeshTypes = 0; // all typed connections allowed
			p.userData = 0;
			return p;
		}

		int addAgentAt(const float* pos)
		{
			dtCrowdAgentParams p = defaultAgentParams();
			return crowd->addAgent(pos, &p);
		}

		int addAgentAt(const float* pos, const float maxSpeed)
		{
			dtCrowdAgentParams p = defaultAgentParams();
			p.maxSpeed = maxSpeed;
			return crowd->addAgent(pos, &p);
		}

		/// Plans a corridor through the given polygons and assigns it directly,
		/// bypassing the asynchronous path queue so tests stay deterministic.
		void setCorridor(const int agentIdx, const dtPolyRef* path, const int npath,
						 const float* targetPos)
		{
			dtCrowdAgent* ag = crowd->getEditableAgent(agentIdx);
			ag->corridor.setCorridor(targetPos, path, npath);
			ag->targetRef = path[npath-1];
			dtVcopy(ag->targetPos, targetPos);
			ag->targetState = DT_CROWDAGENT_TARGET_VALID;
			ag->targetPathqRef = DT_PATHQ_INVALID;
			ag->targetReplan = false;
		}
	};

	struct ArrivalRecord
	{
		int agentIdx;
		dtPolyRef conRef;
		unsigned char type;
	};

	void arrivalCollector(const int agentIdx, const dtPolyRef conRef,
						  const unsigned char type, void* userData)
	{
		std::vector<ArrivalRecord>* records = static_cast<std::vector<ArrivalRecord>*>(userData);
		records->push_back(ArrivalRecord{ agentIdx, conRef, type });
	}
}

// ---------------------------------------------------------------------------
// A typed forward traversal completes end to end and returns to walking.
// ---------------------------------------------------------------------------
TEST_CASE("Crowd off-mesh: typed forward traversal completes", "[crowd][offmesh]")
{
	OffMeshFixture f;
	const float start[3] = { 0.5f, 0.0f, 0.5f };
	const float end[3]   = { 1.5f, 0.0f, 0.5f };
	const TestOffMeshCon con = { {0.5f,0,0.5f}, {1.5f,0,0.5f},
								 (WALKABLE | DT_CROWD_OFFMESH_TYPE_JUMP), 0, 0, 42 };
	f.init(4, std::vector<int>{0},
		   std::vector<std::vector<TestOffMeshCon> >{ { con } });

	const dtPolyRef startRef = f.polyAt(start);
	const dtPolyRef endRef = f.polyAt(end);
	REQUIRE(startRef);
	REQUIRE(endRef);
	REQUIRE(startRef != endRef);

	const dtOffMeshConnection* navCon = 0;
	// Locate the connection poly via a path query instead of guessing refs.
	dtPolyRef path[8];
	int npath = 0;
	REQUIRE(dtStatusSucceed(f.query->findPath(startRef, endRef, start, end,
											  f.crowd->getFilter(0), path, &npath, 8)));
	REQUIRE(npath >= 3);
	dtPolyRef conRef = 0;
	for (int i = 0; i < npath; ++i)
	{
		const dtOffMeshConnection* c = f.nav->getOffMeshConnectionByRef(path[i]);
		if (c)
		{
			conRef = path[i];
			navCon = c;
		}
	}
	REQUIRE(conRef);
	REQUIRE(navCon != 0);
	REQUIRE((navCon->flags & DT_OFFMESH_CON_BIDIR) == 0); // unidirectional

	std::vector<ArrivalRecord> arrivals;
	f.crowd->setOffMeshConnectionArrivalCallback(arrivalCollector, &arrivals);

	const int idx = f.addAgentAt(start);
	REQUIRE(idx >= 0);
	f.setCorridor(idx, path, npath, end);

	// Drive updates until the agent enters the connection.
	bool entered = false;
	for (int frame = 0; frame < 200 && !entered; ++frame)
	{
		f.crowd->update(0.1f, 0);
		if (f.crowd->getAgent(idx)->state == DT_CROWDAGENT_STATE_OFFMESH)
			entered = true;
	}
	REQUIRE(entered);
	REQUIRE(f.crowd->getAgentTraversalState(idx) == DT_CROWD_TRAVERSAL_ACTIVE);
	REQUIRE(f.crowd->getAgentOffMeshConnection(idx) == conRef);
	REQUIRE(f.crowd->getOffMeshConnectionOccupant(conRef) == idx);

	// While traversing, the agent must not be steered as an ordinary walker.
	REQUIRE(f.crowd->getAgent(idx)->state == DT_CROWDAGENT_STATE_OFFMESH);

	// Complete the traversal.
	bool completed = false;
	for (int frame = 0; frame < 200 && !completed; ++frame)
	{
		f.crowd->update(0.1f, 0);
		if (f.crowd->getAgent(idx)->state == DT_CROWDAGENT_STATE_WALKING)
			completed = true;
	}
	REQUIRE(completed);
	REQUIRE(f.crowd->getAgentTraversalState(idx) == DT_CROWD_TRAVERSAL_NONE);
	REQUIRE(f.crowd->getAgentOffMeshConnection(idx) == 0);
	REQUIRE(f.crowd->getOffMeshConnectionOccupant(conRef) == -1);

	REQUIRE(arrivals.size() == 1);
	CHECK(arrivals[0].agentIdx == idx);
	CHECK(arrivals[0].conRef == conRef);
	CHECK(arrivals[0].type == DT_CROWD_OFFMESH_TYPE_JUMP);

	// The agent landed on the far side polygon.
	const dtCrowdAgent* ag = f.crowd->getAgent(idx);
	CHECK(ag->corridor.getFirstPoly() == endRef);
}

// ---------------------------------------------------------------------------
// A second agent cannot occupy a typed connection already in use.
// ---------------------------------------------------------------------------
TEST_CASE("Crowd off-mesh: occupancy is exclusive and deterministic", "[crowd][offmesh]")
{
	OffMeshFixture f;
	const float start[3] = { 0.5f, 0.0f, 0.5f };
	const float end[3]   = { 1.5f, 0.0f, 0.5f };
	const TestOffMeshCon con = { {0.5f,0,0.5f}, {1.5f,0,0.5f},
								 (WALKABLE | DT_CROWD_OFFMESH_TYPE_DOOR), 0, 0, 7 };
	f.init(4, std::vector<int>{0},
		   std::vector<std::vector<TestOffMeshCon> >{ { con } });

	const dtPolyRef startRef = f.polyAt(start);
	const dtPolyRef endRef = f.polyAt(end);

	dtPolyRef path[8];
	int npath = 0;
	REQUIRE(dtStatusSucceed(f.query->findPath(startRef, endRef, start, end,
											  f.crowd->getFilter(0), path, &npath, 8)));
	dtPolyRef conRef = 0;
	for (int i = 0; i < npath; ++i)
		if (f.nav->getOffMeshConnectionByRef(path[i]))
			conRef = path[i];
	REQUIRE(conRef);

	// Slow agents make the traversal take several frames so the exclusive
	// ownership can be observed while it is actually in progress.
	const int a = f.addAgentAt(start, 0.2f);
	const int b = f.addAgentAt(start, 0.2f);
	REQUIRE(a >= 0);
	REQUIRE(b >= 0);
	REQUIRE(a != b);
	f.setCorridor(a, path, npath, end);
	f.setCorridor(b, path, npath, end);

	// First agent enters.
	bool entered = false;
	for (int frame = 0; frame < 200 && !entered; ++frame)
	{
		f.crowd->update(0.1f, 0);
		if (f.crowd->getAgent(a)->state == DT_CROWDAGENT_STATE_OFFMESH)
			entered = true;
	}
	REQUIRE(entered);
	REQUIRE(f.crowd->getOffMeshConnectionOccupant(conRef) == a);

	// The second agent must never take over the same connection while agent a
	// is traversing it. Deterministic: owner a keeps it for every frame.
	for (int frame = 0; frame < 5; ++frame)
	{
		f.crowd->update(0.1f, 0);
		CHECK(f.crowd->getOffMeshConnectionOccupant(conRef) == a);
		CHECK(f.crowd->getAgent(b)->state != DT_CROWDAGENT_STATE_OFFMESH);
	}

	// Once agent a leaves, the slot is released and agent b may eventually use it.
	bool aDone = false;
	for (int frame = 0; frame < 200 && !aDone; ++frame)
	{
		f.crowd->update(0.1f, 0);
		if (f.crowd->getAgent(a)->state == DT_CROWDAGENT_STATE_WALKING)
			aDone = true;
	}
	REQUIRE(aDone);
	REQUIRE(f.crowd->getOffMeshConnectionOccupant(conRef) == -1);
}

// ---------------------------------------------------------------------------
// A unidirectional connection cannot be traversed in reverse; the rejected
// attempt must not masquerade as normal reachability.
// ---------------------------------------------------------------------------
TEST_CASE("Crowd off-mesh: unidirectional connection rejects reverse entry", "[crowd][offmesh]")
{
	OffMeshFixture f;
	// Unidirectional A->B connection (poly 0 -> poly 1), plus a bidirectional
	// connection elsewhere so the mesh still has a legal return route.
	const TestOffMeshCon oneWay = { {0.5f,0,0.2f}, {1.5f,0,0.2f},
									(WALKABLE | DT_CROWD_OFFMESH_TYPE_JUMP), 0, 0, 1 };
	const TestOffMeshCon twoWay = { {1.5f,0,0.8f}, {0.5f,0,0.8f},
									(WALKABLE | DT_CROWD_OFFMESH_TYPE_CLIMB), DT_OFFMESH_CON_BIDIR, 0, 2 };
	f.init(4, std::vector<int>{0},
		   std::vector<std::vector<TestOffMeshCon> >{ { oneWay, twoWay } });

	const float aPos[3] = { 0.5f, 0.0f, 0.2f };
	const float bPos[3] = { 1.5f, 0.0f, 0.2f };
	const dtPolyRef polyA = f.polyAt(aPos);
	const dtPolyRef polyB = f.polyAt(bPos);
	REQUIRE(polyA != 0);
	REQUIRE(polyB != 0);
	REQUIRE(polyA != polyB);

	// Find the unidirectional connection reference.
	dtPolyRef oneWayRef = 0;
	const dtMeshTile* tile = f.nav->getTileAt(0,0,0);
	REQUIRE(tile != 0);
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		const dtPolyRef ref = f.nav->getPolyRefBase(tile) | (dtPolyRef)i;
		const dtOffMeshConnection* c = f.nav->getOffMeshConnectionByRef(ref);
		if (c && c->userId == 1)
			oneWayRef = ref;
	}
	REQUIRE(oneWayRef);

	const int idx = f.addAgentAt(bPos);
	REQUIRE(idx >= 0);

	// Forge a corridor that would traverse the one-way connection backwards:
	// current poly B, then the off-mesh connection poly, then poly A.
	const dtPolyRef reversePath[3] = { polyB, oneWayRef, polyA };
	const float target[3] = { 0.5f, 0.0f, 0.2f };
	f.setCorridor(idx, reversePath, 3, target);

	// Move the agent onto the connection's B endpoint so the trigger fires.
	dtCrowdAgent* ag = f.crowd->getEditableAgent(idx);
	dtVset(ag->npos, 1.5f, 0.0f, 0.2f);
	ag->corridor.movePosition(ag->npos, f.query, f.crowd->getFilter(0));

	// Seed a corner pointing at the off-mesh connection so overOffmeshConnection fires.
	for (int frame = 0; frame < 50; ++frame)
	{
		f.crowd->update(0.1f, 0);
		if (f.crowd->getAgent(idx)->state == DT_CROWDAGENT_STATE_OFFMESH)
			break;
	}

	// The reverse attempt must never enter the traversal.
	CHECK(f.crowd->getAgent(idx)->state != DT_CROWDAGENT_STATE_OFFMESH);
	CHECK(f.crowd->getAgentTraversalState(idx) != DT_CROWD_TRAVERSAL_ACTIVE);
	CHECK(f.crowd->getOffMeshConnectionOccupant(oneWayRef) == -1);

	// A forward traversal of the same connection still works.
	const int fwd = f.addAgentAt(aPos);
	dtPolyRef fwdPath[8];
	int nfwd = 0;
	REQUIRE(dtStatusSucceed(f.query->findPath(polyA, polyB, aPos, bPos,
											  f.crowd->getFilter(0), fwdPath, &nfwd, 8)));
	f.setCorridor(fwd, fwdPath, nfwd, bPos);
	bool entered = false;
	for (int frame = 0; frame < 200 && !entered; ++frame)
	{
		f.crowd->update(0.1f, 0);
		if (f.crowd->getAgent(fwd)->state == DT_CROWDAGENT_STATE_OFFMESH)
			entered = true;
	}
	CHECK(entered);
}

// ---------------------------------------------------------------------------
// A bidirectional connection can be traversed in reverse and records the
// reversed direction semantics.
// ---------------------------------------------------------------------------
TEST_CASE("Crowd off-mesh: bidirectional connection supports reverse traversal", "[crowd][offmesh]")
{
	OffMeshFixture f;
	const TestOffMeshCon twoWay = { {0.5f,0,0.5f}, {1.5f,0,0.5f},
									(WALKABLE | DT_CROWD_OFFMESH_TYPE_ELEVATOR), DT_OFFMESH_CON_BIDIR, 0, 5 };
	f.init(4, std::vector<int>{0},
		   std::vector<std::vector<TestOffMeshCon> >{ { twoWay } });

	const float aPos[3] = { 0.5f, 0.0f, 0.5f };
	const float bPos[3] = { 1.5f, 0.0f, 0.5f };
	const dtPolyRef polyA = f.polyAt(aPos);
	const dtPolyRef polyB = f.polyAt(bPos);

	dtPolyRef conRef = 0;
	const dtMeshTile* tile = f.nav->getTileAt(0,0,0);
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		const dtPolyRef ref = f.nav->getPolyRefBase(tile) | (dtPolyRef)i;
		if (f.nav->getOffMeshConnectionByRef(ref))
			conRef = ref;
	}
	REQUIRE(conRef);

	// Reverse path: B -> connection -> A.
	const int idx = f.addAgentAt(bPos);
	const dtPolyRef reversePath[3] = { polyB, conRef, polyA };
	const float target[3] = { 0.5f, 0.0f, 0.5f };
	f.setCorridor(idx, reversePath, 3, target);

	bool entered = false;
	for (int frame = 0; frame < 200 && !entered; ++frame)
	{
		f.crowd->update(0.1f, 0);
		if (f.crowd->getAgent(idx)->state == DT_CROWDAGENT_STATE_OFFMESH)
			entered = true;
	}
	REQUIRE(entered);
	REQUIRE(f.crowd->getAgentTraversalState(idx) == DT_CROWD_TRAVERSAL_ACTIVE);

	bool completed = false;
	for (int frame = 0; frame < 200 && !completed; ++frame)
	{
		f.crowd->update(0.1f, 0);
		if (f.crowd->getAgent(idx)->state == DT_CROWDAGENT_STATE_WALKING)
			completed = true;
	}
	REQUIRE(completed);
	// Reverse traversal lands on poly A.
	CHECK(f.crowd->getAgent(idx)->corridor.getFirstPoly() == polyA);
}

// ---------------------------------------------------------------------------
// An agent whose allowed type mask excludes a connection cannot enter it.
// ---------------------------------------------------------------------------
TEST_CASE("Crowd off-mesh: agent type filter blocks disallowed connections", "[crowd][offmesh]")
{
	OffMeshFixture f;
	const TestOffMeshCon con = { {0.5f,0,0.5f}, {1.5f,0,0.5f},
								 (WALKABLE | DT_CROWD_OFFMESH_TYPE_CLIMB), 0, 0, 9 };
	f.init(4, std::vector<int>{0},
		   std::vector<std::vector<TestOffMeshCon> >{ { con } });

	const float start[3] = { 0.5f, 0.0f, 0.5f };
	const float end[3]   = { 1.5f, 0.0f, 0.5f };
	const dtPolyRef startRef = f.polyAt(start);
	const dtPolyRef endRef = f.polyAt(end);

	dtPolyRef path[8];
	int npath = 0;
	REQUIRE(dtStatusSucceed(f.query->findPath(startRef, endRef, start, end,
											  f.crowd->getFilter(0), path, &npath, 8)));
	dtPolyRef conRef = 0;
	for (int i = 0; i < npath; ++i)
		if (f.nav->getOffMeshConnectionByRef(path[i]))
			conRef = path[i];
	REQUIRE(conRef);

	dtCrowdAgentParams p = f.defaultAgentParams();
	p.allowedOffMeshTypes = DT_CROWD_OFFMESH_TYPE_DOOR; // cannot climb
	const int idx = f.crowd->addAgent(start, &p);
	REQUIRE(idx >= 0);
	f.setCorridor(idx, path, npath, end);

	for (int frame = 0; frame < 50; ++frame)
	{
		f.crowd->update(0.1f, 0);
		CHECK(f.crowd->getAgent(idx)->state != DT_CROWDAGENT_STATE_OFFMESH);
	}
	CHECK(f.crowd->getOffMeshConnectionOccupant(conRef) == -1);
}

// ---------------------------------------------------------------------------
// Legacy, untyped connections keep the original interpolation behavior.
// ---------------------------------------------------------------------------
TEST_CASE("Crowd off-mesh: untyped legacy connection keeps old behavior", "[crowd][offmesh]")
{
	OffMeshFixture f;
	const TestOffMeshCon con = { {0.5f,0,0.5f}, {1.5f,0,0.5f},
								 WALKABLE, 0, 0, 11 }; // walkable but no type bits -> legacy
	f.init(4, std::vector<int>{0},
		   std::vector<std::vector<TestOffMeshCon> >{ { con } });

	const float start[3] = { 0.5f, 0.0f, 0.5f };
	const float end[3]   = { 1.5f, 0.0f, 0.5f };
	const dtPolyRef startRef = f.polyAt(start);
	const dtPolyRef endRef = f.polyAt(end);

	dtPolyRef path[8];
	int npath = 0;
	REQUIRE(dtStatusSucceed(f.query->findPath(startRef, endRef, start, end,
											  f.crowd->getFilter(0), path, &npath, 8)));
	dtPolyRef conRef = 0;
	for (int i = 0; i < npath; ++i)
		if (f.nav->getOffMeshConnectionByRef(path[i]))
			conRef = path[i];
	REQUIRE(conRef);

	const int idx = f.addAgentAt(start);
	f.setCorridor(idx, path, npath, end);

	bool entered = false;
	for (int frame = 0; frame < 200 && !entered; ++frame)
	{
		f.crowd->update(0.1f, 0);
		if (f.crowd->getAgent(idx)->state == DT_CROWDAGENT_STATE_OFFMESH)
			entered = true;
	}
	REQUIRE(entered);
	CHECK(f.crowd->getAgentTraversalState(idx) == DT_CROWD_TRAVERSAL_LEGACY);
	// Legacy connections are never occupied exclusively.
	CHECK(f.crowd->getOffMeshConnectionOccupant(conRef) == -1);

	bool completed = false;
	for (int frame = 0; frame < 200 && !completed; ++frame)
	{
		f.crowd->update(0.1f, 0);
		if (f.crowd->getAgent(idx)->state == DT_CROWDAGENT_STATE_WALKING)
			completed = true;
	}
	REQUIRE(completed);
}

// ---------------------------------------------------------------------------
// Streaming out the tile mid-traversal hands the agent to a valid polygon on
// another tile and releases the stale connection, with no stranded occupancy.
// ---------------------------------------------------------------------------
TEST_CASE("Crowd off-mesh: tile stream-out during traversal recovers agent", "[crowd][offmesh]")
{
	OffMeshFixture f;
	// Tile 0 holds a forward connection whose far endpoint lands in tile 1.
	const TestOffMeshCon con = { {0.5f,0,0.5f}, {2.5f,0,0.5f},
								 (WALKABLE | DT_CROWD_OFFMESH_TYPE_JUMP), 0, 0, 21 };
	f.init(8, std::vector<int>{0, 1},
		   std::vector<std::vector<TestOffMeshCon> >{ { con }, { } });

	const float start[3] = { 0.5f, 0.0f, 0.5f };
	const float end[3]   = { 2.5f, 0.0f, 0.5f };
	const dtPolyRef startRef = f.polyAt(start);
	const dtPolyRef endRef = f.polyAt(end);
	REQUIRE(startRef != 0);
	REQUIRE(endRef != 0);

	dtPolyRef path[16];
	int npath = 0;
	REQUIRE(dtStatusSucceed(f.query->findPath(startRef, endRef, start, end,
											  f.crowd->getFilter(0), path, &npath, 16)));
	dtPolyRef conRef = 0;
	for (int i = 0; i < npath; ++i)
		if (f.nav->getOffMeshConnectionByRef(path[i]))
			conRef = path[i];
	REQUIRE(conRef);

	const int idx = f.addAgentAt(start);
	f.setCorridor(idx, path, npath, end);

	bool entered = false;
	for (int frame = 0; frame < 200 && !entered; ++frame)
	{
		f.crowd->update(0.1f, 0);
		if (f.crowd->getAgent(idx)->state == DT_CROWDAGENT_STATE_OFFMESH)
			entered = true;
	}
	REQUIRE(entered);
	REQUIRE(f.crowd->getOffMeshConnectionOccupant(conRef) == idx);

	// Stream out the tile that owns the connection while the agent is mid-air.
	dtTileRef tile0Ref = f.tiles[0].ref;
	unsigned char* removedData = 0;
	int removedSize = 0;
	REQUIRE(dtStatusSucceed(f.nav->removeTile(tile0Ref, &removedData, &removedSize)));
	REQUIRE(removedData);
	// The buffer stays owned by the fixture; do not free it here.

	// The next update must recover the agent onto a surviving polygon.
	f.crowd->update(0.1f, 0);
	const dtCrowdAgent* ag = f.crowd->getAgent(idx);
	CHECK(ag->state == DT_CROWDAGENT_STATE_WALKING);
	CHECK(f.crowd->getAgentTraversalState(idx) == DT_CROWD_TRAVERSAL_NONE);
	CHECK(f.crowd->getAgentOffMeshConnection(idx) == 0);
	CHECK(f.crowd->getOffMeshConnectionOccupant(conRef) == -1);
	// The recovered corridor starts on a still-valid polygon (tile 1).
	CHECK(f.nav->isValidPolyRef(ag->corridor.getFirstPoly()));
}

// ---------------------------------------------------------------------------
// Removing every reachable tile mid-traversal yields the recognizable stalled
// state instead of a silently dropped agent.
// ---------------------------------------------------------------------------
TEST_CASE("Crowd off-mesh: unrecoverable traversal reports stalled state", "[crowd][offmesh]")
{
	OffMeshFixture f;
	const TestOffMeshCon con = { {0.5f,0,0.5f}, {1.5f,0,0.5f},
								 (WALKABLE | DT_CROWD_OFFMESH_TYPE_JUMP), 0, 0, 31 };
	f.init(4, std::vector<int>{0},
		   std::vector<std::vector<TestOffMeshCon> >{ { con } });

	const float start[3] = { 0.5f, 0.0f, 0.5f };
	const float end[3]   = { 1.5f, 0.0f, 0.5f };
	const dtPolyRef startRef = f.polyAt(start);
	const dtPolyRef endRef = f.polyAt(end);

	dtPolyRef path[8];
	int npath = 0;
	REQUIRE(dtStatusSucceed(f.query->findPath(startRef, endRef, start, end,
											  f.crowd->getFilter(0), path, &npath, 8)));
	dtPolyRef conRef = 0;
	for (int i = 0; i < npath; ++i)
		if (f.nav->getOffMeshConnectionByRef(path[i]))
			conRef = path[i];
	REQUIRE(conRef);

	const int idx = f.addAgentAt(start);
	f.setCorridor(idx, path, npath, end);

	bool entered = false;
	for (int frame = 0; frame < 200 && !entered; ++frame)
	{
		f.crowd->update(0.1f, 0);
		if (f.crowd->getAgent(idx)->state == DT_CROWDAGENT_STATE_OFFMESH)
			entered = true;
	}
	REQUIRE(entered);

	// Remove the only tile; no valid polygon remains to take over.
	unsigned char* removedData = 0;
	int removedSize = 0;
	REQUIRE(dtStatusSucceed(f.nav->removeTile(f.tiles[0].ref, &removedData, &removedSize)));
	REQUIRE(removedData);
	// The buffer stays owned by the fixture; do not free it here.

	f.crowd->update(0.1f, 0);
	const dtCrowdAgent* ag = f.crowd->getAgent(idx);
	CHECK(ag->state == DT_CROWDAGENT_STATE_OFFMESH_STALLED);
	CHECK(f.crowd->getAgentTraversalState(idx) == DT_CROWD_TRAVERSAL_STALLED);
	// No stale occupancy or connection reference survives.
	CHECK(f.crowd->getAgentOffMeshConnection(idx) == 0);
	CHECK(f.crowd->getOffMeshConnectionOccupant(conRef) == -1);
}

// ---------------------------------------------------------------------------
// Removing an agent mid-traversal releases its occupancy slot.
// ---------------------------------------------------------------------------
TEST_CASE("Crowd off-mesh: removing an agent releases occupancy", "[crowd][offmesh]")
{
	OffMeshFixture f;
	const TestOffMeshCon con = { {0.5f,0,0.5f}, {1.5f,0,0.5f},
								 (WALKABLE | DT_CROWD_OFFMESH_TYPE_DOOR), 0, 0, 41 };
	f.init(4, std::vector<int>{0},
		   std::vector<std::vector<TestOffMeshCon> >{ { con } });

	const float start[3] = { 0.5f, 0.0f, 0.5f };
	const float end[3]   = { 1.5f, 0.0f, 0.5f };
	const dtPolyRef startRef = f.polyAt(start);
	const dtPolyRef endRef = f.polyAt(end);

	dtPolyRef path[8];
	int npath = 0;
	REQUIRE(dtStatusSucceed(f.query->findPath(startRef, endRef, start, end,
											  f.crowd->getFilter(0), path, &npath, 8)));
	dtPolyRef conRef = 0;
	for (int i = 0; i < npath; ++i)
		if (f.nav->getOffMeshConnectionByRef(path[i]))
			conRef = path[i];
	REQUIRE(conRef);

	const int idx = f.addAgentAt(start);
	f.setCorridor(idx, path, npath, end);

	bool entered = false;
	for (int frame = 0; frame < 200 && !entered; ++frame)
	{
		f.crowd->update(0.1f, 0);
		if (f.crowd->getAgent(idx)->state == DT_CROWDAGENT_STATE_OFFMESH)
			entered = true;
	}
	REQUIRE(entered);
	REQUIRE(f.crowd->getOffMeshConnectionOccupant(conRef) == idx);

	f.crowd->removeAgent(idx);
	CHECK(f.crowd->getOffMeshConnectionOccupant(conRef) == -1);
}
