//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <limits.h>
#include <stddef.h>

namespace
{
/// Allocates and constructs an object of the given type, returning a pointer.
/// @param[in]		allocLifetime	Allocation lifetime hint
template<typename T>
T* rcNew(const rcAllocHint allocLifetime)
{
	T* ptr = (T*)rcAlloc(sizeof(T), allocLifetime);
	::new(rcNewTag(), (void*)ptr) T();
	return ptr;
}

/// Destroys and frees an object allocated with rcNew.
/// @param[in]     ptr    The object pointer to delete.
template<typename T>
void rcDelete(T* ptr)
{
	if (ptr)
	{
		ptr->~T();
		rcFree((void*)ptr);
	}
}
} // anonymous namespace

float rcSqrt(float x)
{
	return sqrtf(x);
}

void rcContext::log(const rcLogCategory category, const char* format, ...)
{
	if (!m_logEnabled)
	{
		return;
	}
	static const int MSG_SIZE = 512;
	char msg[MSG_SIZE];
	va_list argList;
	va_start(argList, format);
	int len = vsnprintf(msg, MSG_SIZE, format, argList);
	if (len >= MSG_SIZE)
	{
		len = MSG_SIZE - 1;
		msg[MSG_SIZE - 1] = '\0';

		const char* errorMessage = "Log message was truncated";
		doLog(RC_LOG_ERROR, errorMessage, (int)strlen(errorMessage));
	}
	va_end(argList);
	doLog(category, msg, len);
}

void rcContext::doResetLog()
{
	// Defined out of line to fix the weak v-tables warning
}

rcHeightfield* rcAllocHeightfield()
{
	return rcNew<rcHeightfield>(RC_ALLOC_PERM);
}

void rcFreeHeightField(rcHeightfield* heightfield)
{
	rcDelete(heightfield);
}

rcHeightfield::rcHeightfield()
: width()
, height()
, bmin()
, bmax()
, cs()
, ch()
, spans()
, pools()
, freelist()
{
}

rcHeightfield::~rcHeightfield()
{
	// Delete span array.
	rcFree(spans);
	// Delete span pools.
	while (pools)
	{
		rcSpanPool* next = pools->next;
		rcFree(pools);
		pools = next;
	}
}

rcCompactHeightfield* rcAllocCompactHeightfield()
{
	return rcNew<rcCompactHeightfield>(RC_ALLOC_PERM);
}

void rcFreeCompactHeightfield(rcCompactHeightfield* compactHeightfield)
{
	rcDelete(compactHeightfield);
}

rcCompactHeightfield::rcCompactHeightfield()
: width()
, height()
, spanCount()
, walkableHeight()
, walkableClimb()
, borderSize()
, maxDistance()
, maxRegions()
, bmin()
, bmax()
, cs()
, ch()
, cells()
, spans()
, dist()
, areas()
{
}

rcCompactHeightfield::~rcCompactHeightfield()
{
	rcFree(cells);
	rcFree(spans);
	rcFree(dist);
	rcFree(areas);
}

rcHeightfieldLayerSet* rcAllocHeightfieldLayerSet()
{
	return rcNew<rcHeightfieldLayerSet>(RC_ALLOC_PERM);
}

void rcFreeHeightfieldLayerSet(rcHeightfieldLayerSet* layerSet)
{
	rcDelete(layerSet);
}

rcHeightfieldLayerSet::rcHeightfieldLayerSet()
: layers()
, nlayers()
{
}

rcHeightfieldLayerSet::~rcHeightfieldLayerSet()
{
	for (int i = 0; i < nlayers; ++i)
	{
		rcFree(layers[i].heights);
		rcFree(layers[i].areas);
		rcFree(layers[i].cons);
	}
	rcFree(layers);
}


rcContourSet* rcAllocContourSet()
{
	return rcNew<rcContourSet>(RC_ALLOC_PERM);
}

void rcFreeContourSet(rcContourSet* contourSet)
{
	rcDelete(contourSet);
}

rcContourSet::rcContourSet()
: conts()
, nconts()
, bmin()
, bmax()
, cs()
, ch()
, width()
, height()
, borderSize()
, maxError()
{
}

rcContourSet::~rcContourSet()
{
	for (int i = 0; i < nconts; ++i)
	{
		rcFree(conts[i].verts);
		rcFree(conts[i].rverts);
	}
	rcFree(conts);
}

rcPolyMesh* rcAllocPolyMesh()
{
	return rcNew<rcPolyMesh>(RC_ALLOC_PERM);
}

void rcFreePolyMesh(rcPolyMesh* polyMesh)
{
	rcDelete(polyMesh);
}

rcPolyMesh::rcPolyMesh()
: verts()
, polys()
, regs()
, flags()
, areas()
, nverts()
, npolys()
, maxpolys()
, nvp()
, bmin()
, bmax()
, cs()
, ch()
, borderSize()
, maxEdgeError()
{
}

rcPolyMesh::~rcPolyMesh()
{
	rcFree(verts);
	rcFree(polys);
	rcFree(regs);
	rcFree(flags);
	rcFree(areas);
}

rcPolyMeshDetail* rcAllocPolyMeshDetail()
{
	return rcNew<rcPolyMeshDetail>(RC_ALLOC_PERM);
}

void rcFreePolyMeshDetail(rcPolyMeshDetail* detailMesh)
{
	if (detailMesh == NULL)
	{
		return;
	}
	rcFree(detailMesh->meshes);
	rcFree(detailMesh->verts);
	rcFree(detailMesh->tris);
	rcFree(detailMesh);
}

rcPolyMeshDetail::rcPolyMeshDetail()
: meshes()
, verts()
, tris()
, nmeshes()
, nverts()
, ntris()
{
}

void rcCalcBounds(const float* verts, int numVerts, float* minBounds, float* maxBounds)
{
	if (!verts || numVerts <= 0)
	{
		minBounds[0] = minBounds[1] = minBounds[2] = 0.0f;
		maxBounds[0] = maxBounds[1] = maxBounds[2] = 0.0f;
		return;
	}

	// Non-finite vertices carry no meaningful bounds information. Skipping
	// them keeps a single bad vertex from poisoning the whole build with NaN
	// or infinite extents (which would later turn into an invalid grid size).
	bool initialized = false;
	for (int i = 0; i < numVerts; ++i)
	{
		const float* v = &verts[i * 3];
		if (!rcVisfinite(v))
		{
			continue;
		}

		if (!initialized)
		{
			rcVcopy(minBounds, v);
			rcVcopy(maxBounds, v);
			initialized = true;
			continue;
		}

		rcVmin(minBounds, v);
		rcVmax(maxBounds, v);
	}

	if (!initialized)
	{
		minBounds[0] = minBounds[1] = minBounds[2] = 0.0f;
		maxBounds[0] = maxBounds[1] = maxBounds[2] = 0.0f;
	}
}

void rcCalcGridSize(const float* minBounds, const float* maxBounds, const float cellSize, int* sizeX, int* sizeZ)
{
	// A non-positive cell size or non-finite bounds cannot describe a grid.
	// Returning a zero sized grid keeps the failure visible to the caller
	// instead of overflowing the integer conversion.
	if (!(cellSize > 0.0f) || !rcVisfinite(minBounds) || !rcVisfinite(maxBounds))
	{
		*sizeX = 0;
		*sizeZ = 0;
		return;
	}

	// Keep the original float arithmetic so the result is bit-for-bit identical
	// for well formed input. The clamp only affects huge (but finite) extents
	// which would otherwise overflow the integer conversion.
	const float sizeXf = (maxBounds[0] - minBounds[0]) / cellSize + 0.5f;
	const float sizeZf = (maxBounds[2] - minBounds[2]) / cellSize + 0.5f;

	// Widening to double is exact and makes the comparison against INT_MAX well
	// defined for every float value.
	double dx = (double)sizeXf;
	double dz = (double)sizeZf;
	if (dx < 0.0) dx = 0.0;
	if (dz < 0.0) dz = 0.0;
	if (dx > (double)INT_MAX) dx = (double)INT_MAX;
	if (dz > (double)INT_MAX) dz = (double)INT_MAX;
	*sizeX = (int)dx;
	*sizeZ = (int)dz;
}

bool rcCreateHeightfield(rcContext* context, rcHeightfield& heightfield, int sizeX, int sizeZ,
                         const float* minBounds, const float* maxBounds,
                         float cellSize, float cellHeight)
{
	// Reject configurations that cannot describe a valid heightfield. Without
	// this a bad input (negative size, zero cell size or NaN bounds) would
	// either allocate a nonsensical amount of memory or produce spans whose
	// indices are undefined.
	if (sizeX < 0 || sizeZ < 0 ||
		!(cellSize > 0.0f) || !(cellHeight > 0.0f) ||
		!rcVisfinite(minBounds) || !rcVisfinite(maxBounds))
	{
		if (context)
		{
			context->log(RC_LOG_ERROR, "rcCreateHeightfield: Invalid heightfield parameters.");
		}
		return false;
	}

	// Guard against the multiplication below overflowing a size_t / int.
	if ((size_t)sizeX > (size_t)(-1) / sizeof(rcSpan*) / (size_t)(sizeZ > 0 ? sizeZ : 1))
	{
		if (context)
		{
			context->log(RC_LOG_ERROR, "rcCreateHeightfield: Heightfield is too large.");
		}
		return false;
	}

	heightfield.width = sizeX;
	heightfield.height = sizeZ;
	rcVcopy(heightfield.bmin, minBounds);
	rcVcopy(heightfield.bmax, maxBounds);
	heightfield.cs = cellSize;
	heightfield.ch = cellHeight;

	const int numColumns = heightfield.width * heightfield.height;
	if (numColumns == 0)
	{
		// A heightfield with no columns is valid but empty.
		heightfield.spans = NULL;
		return true;
	}

	heightfield.spans = (rcSpan**)rcAlloc(sizeof(rcSpan*) * numColumns, RC_ALLOC_PERM);
	if (!heightfield.spans)
	{
		return false;
	}
	memset(heightfield.spans, 0, sizeof(rcSpan*) * numColumns);
	return true;
}

static void calcTriNormal(const float* v0, const float* v1, const float* v2, float* faceNormal)
{
	float e0[3], e1[3];
	rcVsub(e0, v1, v0);
	rcVsub(e1, v2, v0);
	rcVcross(faceNormal, e0, e1);
	rcVnormalize(faceNormal);
}

rcTriangleClassification rcClassifyTriangle(const float* v0, const float* v1, const float* v2)
{
	// A triangle whose shortest edge is below this fraction of its longest edge
	// cannot define a plane reliably; the resulting normal would be dominated
	// by floating point noise.
	static const float MIN_EDGE_RATIO = 1e-6f;
	// A triangle whose area is below this fraction of its squared longest edge
	// is numerically collinear (coincident or collinear vertices and slivers).
	static const float MIN_AREA_RATIO = 1e-6f;

	if (v0 == NULL || v1 == NULL || v2 == NULL)
	{
		return RC_TRIANGLE_NON_FINITE;
	}

	// NaN and infinite coordinates cannot be mapped to a voxel position.
	if (!rcVisfinite(v0) || !rcVisfinite(v1) || !rcVisfinite(v2))
	{
		return RC_TRIANGLE_NON_FINITE;
	}

	float e0[3], e1[3], e2[3];
	rcVsub(e0, v1, v0);
	rcVsub(e1, v2, v0);
	rcVsub(e2, v2, v1);

	// The difference of two finite but extremely large coordinates can still
	// overflow to an infinity. Such a triangle has no usable extent.
	if (!rcVisfinite(e0) || !rcVisfinite(e1) || !rcVisfinite(e2))
	{
		return RC_TRIANGLE_NON_FINITE;
	}

	float cross[3];
	rcVcross(cross, e0, e1);
	if (!rcVisfinite(cross))
	{
		return RC_TRIANGLE_NON_FINITE;
	}

	// Squared edge lengths are used so the classification is independent of the
	// triangle's position and orientation. It only depends on the triangle
	// itself, which keeps the result stable across tiles and repeated builds.
	const float len0Sq = rcVdot(e0, e0);
	const float len1Sq = rcVdot(e1, e1);
	const float len2Sq = rcVdot(e2, e2);

	// Values large enough to overflow the squared lengths cannot be compared
	// reliably and cannot be voxelized with meaningful precision.
	if (!rcMathIsfinite(len0Sq) || !rcMathIsfinite(len1Sq) || !rcMathIsfinite(len2Sq))
	{
		return RC_TRIANGLE_NON_FINITE;
	}

	const float maxEdgeSq = rcMax(len0Sq, rcMax(len1Sq, len2Sq));
	const float minEdgeSq = rcMin(len0Sq, rcMin(len1Sq, len2Sq));

	// All vertices coincide (or the edges underflow to zero).
	if (!(maxEdgeSq > 0.0f))
	{
		return RC_TRIANGLE_DEGENERATE;
	}

	// An extremely short edge makes the resulting normal (and therefore the
	// slope classification) numerically meaningless.
	if (!(minEdgeSq > maxEdgeSq * (MIN_EDGE_RATIO * MIN_EDGE_RATIO)))
	{
		return RC_TRIANGLE_DEGENERATE;
	}

	// Zero or near-zero area: coincident and collinear vertices.
	const float crossLenSq = rcVdot(cross, cross);
	if (!rcMathIsfinite(crossLenSq))
	{
		return RC_TRIANGLE_NON_FINITE;
	}
	const float crossLen = rcSqrt(crossLenSq);
	if (!(crossLen > maxEdgeSq * MIN_AREA_RATIO))
	{
		return RC_TRIANGLE_DEGENERATE;
	}

	return RC_TRIANGLE_VALID;
}

void rcMarkWalkableTriangles(rcContext* context, const float walkableSlopeAngle,
                             const float* verts, const int numVerts,
                             const int* tris, const int numTris,
                             unsigned char* triAreaIDs)
{
	rcIgnoreUnused(context);
	rcIgnoreUnused(numVerts);

	const float walkableThr = cosf(walkableSlopeAngle / 180.0f * RC_PI);

	float norm[3];

	for (int i = 0; i < numTris; ++i)
	{
		const int* tri = &tris[i * 3];
		if (numVerts > 0 && (tri[0] < 0 || tri[0] >= numVerts ||
		                     tri[1] < 0 || tri[1] >= numVerts ||
		                     tri[2] < 0 || tri[2] >= numVerts))
		{
			// Out of range indices cannot describe a triangle. Leave the area id
			// untouched rather than indexing outside of the vertex array.
			continue;
		}

		const float* v0 = &verts[tri[0] * 3];
		const float* v1 = &verts[tri[1] * 3];
		const float* v2 = &verts[tri[2] * 3];

		// Degenerate triangles have no reliable normal, so they are never
		// classified as walkable. Their area ids are left untouched, matching
		// the documented behavior for unwalkable triangles. They are skipped by
		// the rasterizer as well, so no span is ever created for them.
		if (rcClassifyTriangle(v0, v1, v2) != RC_TRIANGLE_VALID)
		{
			continue;
		}

		calcTriNormal(v0, v1, v2, norm);
		// Check if the face is walkable.
		if (norm[1] > walkableThr)
		{
			triAreaIDs[i] = RC_WALKABLE_AREA;
		}
	}
}

void rcClearUnwalkableTriangles(rcContext* context, const float walkableSlopeAngle,
                                const float* verts, int numVerts,
                                const int* tris, int numTris,
                                unsigned char* triAreaIDs)
{
	rcIgnoreUnused(context);
	rcIgnoreUnused(numVerts);

	// The minimum Y value for a face normal of a triangle with a walkable slope.
	const float walkableLimitY = cosf(walkableSlopeAngle / 180.0f * RC_PI);

	float faceNormal[3];
	for (int i = 0; i < numTris; ++i)
	{
		const int* tri = &tris[i * 3];
		if (numVerts > 0 && (tri[0] < 0 || tri[0] >= numVerts ||
		                     tri[1] < 0 || tri[1] >= numVerts ||
		                     tri[2] < 0 || tri[2] >= numVerts))
		{
			continue;
		}

		const float* v0 = &verts[tri[0] * 3];
		const float* v1 = &verts[tri[1] * 3];
		const float* v2 = &verts[tri[2] * 3];

		// Degenerate triangles cannot be walked on. Make sure they end up with
		// a null area instead of keeping a stale walkable id (their normals are
		// undefined and would otherwise fail both slope comparisons).
		if (rcClassifyTriangle(v0, v1, v2) != RC_TRIANGLE_VALID)
		{
			triAreaIDs[i] = RC_NULL_AREA;
			continue;
		}

		calcTriNormal(v0, v1, v2, faceNormal);
		// Check if the face is walkable.
		if (faceNormal[1] <= walkableLimitY)
		{
			triAreaIDs[i] = RC_NULL_AREA;
		}
	}
}

int rcGetHeightFieldSpanCount(rcContext* context, const rcHeightfield& heightfield)
{
	rcIgnoreUnused(context);

	const int numCols = heightfield.width * heightfield.height;
	int spanCount = 0;
	for (int columnIndex = 0; columnIndex < numCols; ++columnIndex)
	{
		for (rcSpan* span = heightfield.spans[columnIndex]; span != NULL; span = span->next)
		{
			if (span->area != RC_NULL_AREA)
			{
				spanCount++;
			}
		}
	}
	return spanCount;
}

bool rcBuildCompactHeightfield(rcContext* context, const int walkableHeight, const int walkableClimb,
                               const rcHeightfield& heightfield, rcCompactHeightfield& compactHeightfield)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_BUILD_COMPACTHEIGHTFIELD);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;
	const int spanCount = rcGetHeightFieldSpanCount(context, heightfield);

	// Fill in header.
	compactHeightfield.width = xSize;
	compactHeightfield.height = zSize;
	compactHeightfield.spanCount = spanCount;
	compactHeightfield.walkableHeight = walkableHeight;
	compactHeightfield.walkableClimb = walkableClimb;
	compactHeightfield.maxRegions = 0;
	rcVcopy(compactHeightfield.bmin, heightfield.bmin);
	rcVcopy(compactHeightfield.bmax, heightfield.bmax);
	compactHeightfield.bmax[1] += walkableHeight * heightfield.ch;
	compactHeightfield.cs = heightfield.cs;
	compactHeightfield.ch = heightfield.ch;

	// An empty grid has no cells or spans to build. Leave the arrays empty so
	// the rest of the pipeline can handle it without allocating zero sized
	// buffers or dereferencing a null span array.
	if (xSize <= 0 || zSize <= 0 || !heightfield.spans)
	{
		compactHeightfield.cells = NULL;
		compactHeightfield.spans = NULL;
		compactHeightfield.areas = NULL;
		return true;
	}

	compactHeightfield.cells = (rcCompactCell*)rcAlloc(sizeof(rcCompactCell) * xSize * zSize, RC_ALLOC_PERM);
	if (!compactHeightfield.cells)
	{
		context->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Out of memory 'chf.cells' (%d)", xSize * zSize);
		return false;
	}
	memset(compactHeightfield.cells, 0, sizeof(rcCompactCell) * xSize * zSize);
	compactHeightfield.spans = (rcCompactSpan*)rcAlloc(sizeof(rcCompactSpan) * spanCount, RC_ALLOC_PERM);
	if (!compactHeightfield.spans)
	{
		context->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Out of memory 'chf.spans' (%d)", spanCount);
		return false;
	}
	memset(compactHeightfield.spans, 0, sizeof(rcCompactSpan) * spanCount);
	compactHeightfield.areas = (unsigned char*)rcAlloc(sizeof(unsigned char) * spanCount, RC_ALLOC_PERM);
	if (!compactHeightfield.areas)
	{
		context->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Out of memory 'chf.areas' (%d)", spanCount);
		return false;
	}
	memset(compactHeightfield.areas, RC_NULL_AREA, sizeof(unsigned char) * spanCount);

	const int MAX_HEIGHT = 0xffff;

	// Fill in cells and spans.
	int currentCellIndex = 0;
	const int numColumns = xSize * zSize;
	for (int columnIndex = 0; columnIndex < numColumns; ++columnIndex)
	{
		const rcSpan* span = heightfield.spans[columnIndex];
			
		// If there are no spans at this cell, just leave the data to index=0, count=0.
		if (span == NULL)
		{
			continue;
		}
			
		rcCompactCell& cell = compactHeightfield.cells[columnIndex];
		cell.index = currentCellIndex;
		cell.count = 0;

		for (; span != NULL; span = span->next)
		{
			if (span->area != RC_NULL_AREA)
			{
				const int bot = (int)span->smax;
				const int top = span->next ? (int)span->next->smin : MAX_HEIGHT;
				compactHeightfield.spans[currentCellIndex].y = (unsigned short)rcClamp(bot, 0, 0xffff);
				compactHeightfield.spans[currentCellIndex].h = (unsigned char)rcClamp(top - bot, 0, 0xff);
				compactHeightfield.areas[currentCellIndex] = span->area;
				currentCellIndex++;
				cell.count++;
			}
		}
	}
	
	// Find neighbour connections.
	const int MAX_LAYERS = RC_NOT_CONNECTED - 1;
	int maxLayerIndex = 0;
	const int zStride = xSize; // for readability
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			for (int i = (int)cell.index, ni = (int)(cell.index + cell.count); i < ni; ++i)
			{
				rcCompactSpan& span = compactHeightfield.spans[i];

				for (int dir = 0; dir < 4; ++dir)
				{
					rcSetCon(span, dir, RC_NOT_CONNECTED);
					const int neighborX = x + rcGetDirOffsetX(dir);
					const int neighborZ = z + rcGetDirOffsetY(dir);
					// First check that the neighbour cell is in bounds.
					if (neighborX < 0 || neighborZ < 0 || neighborX >= xSize || neighborZ >= zSize)
					{
						continue;
					}

					// Iterate over all neighbour spans and check if any of the is
					// accessible from current cell.
					const rcCompactCell& neighborCell = compactHeightfield.cells[neighborX + neighborZ * zStride];
					for (int k = (int)neighborCell.index, nk = (int)(neighborCell.index + neighborCell.count); k < nk; ++k)
					{
						const rcCompactSpan& neighborSpan = compactHeightfield.spans[k];
						const int bot = rcMax(span.y, neighborSpan.y);
						const int top = rcMin(span.y + span.h, neighborSpan.y + neighborSpan.h);

						// Check that the gap between the spans is walkable,
						// and that the climb height between the gaps is not too high.
						if ((top - bot) >= walkableHeight && rcAbs((int)neighborSpan.y - (int)span.y) <= walkableClimb)
						{
							// Mark direction as walkable.
							const int layerIndex = k - (int)neighborCell.index;
							if (layerIndex < 0 || layerIndex > MAX_LAYERS)
							{
								maxLayerIndex = rcMax(maxLayerIndex, layerIndex);
								continue;
							}
							rcSetCon(span, dir, layerIndex);
							break;
						}
					}
				}
			}
		}
	}

	if (maxLayerIndex > MAX_LAYERS)
	{
		context->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Heightfield has too many layers %d (max: %d)",
		         maxLayerIndex, MAX_LAYERS);
	}

	return true;
}
