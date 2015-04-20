
/*
    pbrt source code Copyright(c) 1998-2012 Matt Pharr and Greg Humphreys.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */


// accelerators/bvh.cpp*
#include "stdafx.h"
#include "accelerators/bvh.h"
#include "probes.h"
#include "paramset.h"

#include <unordered_map>

// BVHAccel Local Declarations
struct BVHPrimitiveInfo {
    BVHPrimitiveInfo() { }
    BVHPrimitiveInfo(int pn, const BBox &b)
        : primitiveNumber(pn), bounds(b) {
        centroid = .5f * b.pMin + .5f * b.pMax;
    }
    int primitiveNumber;
    Point centroid;
    BBox bounds;
};


struct BVHBuildNode {
    // BVHBuildNode Public Methods
    BVHBuildNode() { children[0] = children[1] = NULL; }
    void InitLeaf(uint32_t first, uint32_t n, const BBox &b) {
        firstPrimOffset = first;
        nPrimitives = n;
        bounds = b;
    }
    void InitInterior(uint32_t axis, BVHBuildNode *c0, BVHBuildNode *c1) {
        children[0] = c0;
        children[1] = c1;
        bounds = Union(c0->bounds, c1->bounds);
        splitAxis = axis;
        nPrimitives = 0;
    }
    BBox bounds;
    BVHBuildNode *children[2];
    uint32_t splitAxis, firstPrimOffset, nPrimitives;
};


struct CompareToMid {
    CompareToMid(int d, float m) { dim = d; mid = m; }
    int dim;
    float mid;
    bool operator()(const BVHPrimitiveInfo &a) const {
        return a.centroid[dim] < mid;
    }
};


struct ComparePoints {
    ComparePoints(int d) { dim = d; }
    int dim;
    bool operator()(const BVHPrimitiveInfo &a,
                    const BVHPrimitiveInfo &b) const {
        return a.centroid[dim] < b.centroid[dim];
    }
};


struct CompareToBucket {
    CompareToBucket(int split, int num, int d, const BBox &b)
        : centroidBounds(b)
    { splitBucket = split; nBuckets = num; dim = d; }
    bool operator()(const BVHPrimitiveInfo &p) const;

    int splitBucket, nBuckets, dim;
    const BBox &centroidBounds;
};


bool CompareToBucket::operator()(const BVHPrimitiveInfo &p) const {
    int b = nBuckets * ((p.centroid[dim] - centroidBounds.pMin[dim]) /
            (centroidBounds.pMax[dim] - centroidBounds.pMin[dim]));
    if (b == nBuckets) b = nBuckets-1;
    Assert(b >= 0 && b < nBuckets);
    return b <= splitBucket;
}


struct LinearBVHNode {
    BBox bounds;
    union {
        uint32_t primitivesOffset;    // leaf
        uint32_t secondChildOffset;   // interior
    };

    uint8_t nPrimitives;  // 0 -> interior node
    uint8_t axis;         // interior node: xyz
    uint8_t pad[2];       // ensure 32 byte total size
};


static inline bool IntersectP(const BBox &bounds, const Ray &ray,
        const Vector &invDir, const uint32_t dirIsNeg[3]) {
    // Check for ray intersection against $x$ and $y$ slabs
    float tmin =  (bounds[  dirIsNeg[0]].x - ray.o.x) * invDir.x;
    float tmax =  (bounds[1-dirIsNeg[0]].x - ray.o.x) * invDir.x;
    float tymin = (bounds[  dirIsNeg[1]].y - ray.o.y) * invDir.y;
    float tymax = (bounds[1-dirIsNeg[1]].y - ray.o.y) * invDir.y;
    if ((tmin > tymax) || (tymin > tmax))
        return false;
    if (tymin > tmin) tmin = tymin;
    if (tymax < tmax) tmax = tymax;

    // Check for ray intersection against $z$ slab
    float tzmin = (bounds[  dirIsNeg[2]].z - ray.o.z) * invDir.z;
    float tzmax = (bounds[1-dirIsNeg[2]].z - ray.o.z) * invDir.z;
    if ((tmin > tzmax) || (tzmin > tmax))
        return false;
    if (tzmin > tmin)
        tmin = tzmin;
    if (tzmax < tmax)
        tmax = tzmax;
    return (tmin < ray.maxt) && (tmax > ray.mint);
}



// BVHAccel Method Definitions
BVHAccel::BVHAccel(const vector<Reference<Primitive> > &p,
                   uint32_t mp, const string &sm) {
    maxPrimsInNode = min(255u, mp);
    for (uint32_t i = 0; i < p.size(); ++i)
        p[i]->FullyRefine(primitives);
    if (sm == "sah")         splitMethod = SPLIT_SAH;
    else if (sm == "middle") splitMethod = SPLIT_MIDDLE;
    else if (sm == "equal")  splitMethod = SPLIT_EQUAL_COUNTS;
    else if (sm == "aac")    splitMethod = SPLIT_AAC;
    else {
        Warning("BVH split method \"%s\" unknown.  Using \"sah\".",
                sm.c_str());
        splitMethod = SPLIT_SAH;
    }

    if (primitives.size() == 0) {
        nodes = NULL;
        return;
    }
    // Build BVH from _primitives_
    PBRT_BVH_STARTED_CONSTRUCTION(this, primitives.size());

    // Initialize _buildData_ array for primitives
    vector<BVHPrimitiveInfo> buildData;
    buildData.reserve(primitives.size());
    for (uint32_t i = 0; i < primitives.size(); ++i) {
        BBox bbox = primitives[i]->WorldBound();
        buildData.push_back(BVHPrimitiveInfo(i, bbox));
    }

    // Recursively build BVH tree for primitives
    MemoryArena buildArena;
    uint32_t totalNodes = 0;
    vector<Reference<Primitive> > orderedPrims;
    orderedPrims.reserve(primitives.size());
    
    BVHBuildNode *root;
    if (splitMethod == SPLIT_AAC) {
        root = aacBuild(buildArena, buildData, &totalNodes, orderedPrims);
    } else {
        root = recursiveBuild(buildArena, buildData, 0,
            primitives.size(), &totalNodes,
            orderedPrims);
    }

    primitives.swap(orderedPrims);
        Info("BVH created with %d nodes for %d primitives (%.2f MB)", totalNodes,
             (int)primitives.size(), float(totalNodes * sizeof(LinearBVHNode))/(1024.f*1024.f));

    // Compute representation of depth-first traversal of BVH tree
    nodes = AllocAligned<LinearBVHNode>(totalNodes);
    for (uint32_t i = 0; i < totalNodes; ++i)
        new (&nodes[i]) LinearBVHNode;
    uint32_t offset = 0;
    flattenBVHTree(root, &offset);
    Assert(offset == totalNodes);
    PBRT_BVH_FINISHED_CONSTRUCTION(this);
}


BBox BVHAccel::WorldBound() const {
    return nodes ? nodes[0].bounds : BBox();
}


BVHBuildNode *BVHAccel::recursiveBuild(MemoryArena &buildArena,
        vector<BVHPrimitiveInfo> &buildData, uint32_t start,
        uint32_t end, uint32_t *totalNodes,
        vector<Reference<Primitive> > &orderedPrims) {
    Assert(start != end);
    (*totalNodes)++;
    BVHBuildNode *node = buildArena.Alloc<BVHBuildNode>();
    // Compute bounds of all primitives in BVH node
    BBox bbox;
    for (uint32_t i = start; i < end; ++i)
        bbox = Union(bbox, buildData[i].bounds);
    uint32_t nPrimitives = end - start;
    if (nPrimitives == 1) {
        // Create leaf _BVHBuildNode_
        uint32_t firstPrimOffset = orderedPrims.size();
        for (uint32_t i = start; i < end; ++i) {
            uint32_t primNum = buildData[i].primitiveNumber;
            orderedPrims.push_back(primitives[primNum]);
        }
        node->InitLeaf(firstPrimOffset, nPrimitives, bbox);
    }
    else {
        // Compute bound of primitive centroids, choose split dimension _dim_
        BBox centroidBounds;
        for (uint32_t i = start; i < end; ++i)
            centroidBounds = Union(centroidBounds, buildData[i].centroid);
        int dim = centroidBounds.MaximumExtent();

        // Partition primitives into two sets and build children
        uint32_t mid = (start + end) / 2;
        if (centroidBounds.pMax[dim] == centroidBounds.pMin[dim]) {
            // If nPrimitives is no greater than maxPrimsInNode,
            // then all the nodes can be stored in a compact bvh node.
            if (nPrimitives <= maxPrimsInNode) {
                // Create leaf _BVHBuildNode_
                uint32_t firstPrimOffset = orderedPrims.size();
                for (uint32_t i = start; i < end; ++i) {
                    uint32_t primNum = buildData[i].primitiveNumber;
                    orderedPrims.push_back(primitives[primNum]);
                }
                node->InitLeaf(firstPrimOffset, nPrimitives, bbox);
                return node;
            }
            else {
                // else if nPrimitives is greater than maxPrimsInNode, we
                // need to split it further to guarantee each node contains
                // no more than maxPrimsInNode primitives.
                node->InitInterior(dim,
                                   recursiveBuild(buildArena, buildData, start, mid,
                                                  totalNodes, orderedPrims),
                                   recursiveBuild(buildArena, buildData, mid, end,
                                                  totalNodes, orderedPrims));
                return node;
            }
        }

        // Partition primitives based on _splitMethod_
        switch (splitMethod) {
        case SPLIT_MIDDLE: {
            // Partition primitives through node's midpoint
            float pmid = .5f * (centroidBounds.pMin[dim] + centroidBounds.pMax[dim]);
            BVHPrimitiveInfo *midPtr = std::partition(&buildData[start],
                                                      &buildData[end-1]+1,
                                                      CompareToMid(dim, pmid));
            mid = midPtr - &buildData[0];
            if (mid != start && mid != end)
                // for lots of prims with large overlapping bounding boxes, this
                // may fail to partition; in that case don't break and fall through
                // to SPLIT_EQUAL_COUNTS
                break;
        }
        case SPLIT_EQUAL_COUNTS: {
            // Partition primitives into equally-sized subsets
            mid = (start + end) / 2;
            std::nth_element(&buildData[start], &buildData[mid],
                             &buildData[end-1]+1, ComparePoints(dim));
            break;
        }
        case SPLIT_SAH: default: {
            // Partition primitives using approximate SAH
            if (nPrimitives <= 4) {
                // Partition primitives into equally-sized subsets
                mid = (start + end) / 2;
                std::nth_element(&buildData[start], &buildData[mid],
                                 &buildData[end-1]+1, ComparePoints(dim));
            }
            else {
                // Allocate _BucketInfo_ for SAH partition buckets
                const int nBuckets = 12;
                struct BucketInfo {
                    BucketInfo() { count = 0; }
                    int count;
                    BBox bounds;
                };
                BucketInfo buckets[nBuckets];

                // Initialize _BucketInfo_ for SAH partition buckets
                for (uint32_t i = start; i < end; ++i) {
                    int b = nBuckets *
                        ((buildData[i].centroid[dim] - centroidBounds.pMin[dim]) /
                         (centroidBounds.pMax[dim] - centroidBounds.pMin[dim]));
                    if (b == nBuckets) b = nBuckets-1;
                    Assert(b >= 0 && b < nBuckets);
                    buckets[b].count++;
                    buckets[b].bounds = Union(buckets[b].bounds, buildData[i].bounds);
                }

                // Compute costs for splitting after each bucket
                float cost[nBuckets-1];
                for (int i = 0; i < nBuckets-1; ++i) {
                    BBox b0, b1;
                    int count0 = 0, count1 = 0;
                    for (int j = 0; j <= i; ++j) {
                        b0 = Union(b0, buckets[j].bounds);
                        count0 += buckets[j].count;
                    }
                    for (int j = i+1; j < nBuckets; ++j) {
                        b1 = Union(b1, buckets[j].bounds);
                        count1 += buckets[j].count;
                    }
                    cost[i] = .125f + (count0*b0.SurfaceArea() + count1*b1.SurfaceArea()) /
                              bbox.SurfaceArea();
                }

                // Find bucket to split at that minimizes SAH metric
                float minCost = cost[0];
                uint32_t minCostSplit = 0;
                for (int i = 1; i < nBuckets-1; ++i) {
                    if (cost[i] < minCost) {
                        minCost = cost[i];
                        minCostSplit = i;
                    }
                }

                // Either create leaf or split primitives at selected SAH bucket
                if (nPrimitives > maxPrimsInNode ||
                    minCost < nPrimitives) {
                    BVHPrimitiveInfo *pmid = std::partition(&buildData[start],
                        &buildData[end-1]+1,
                        CompareToBucket(minCostSplit, nBuckets, dim, centroidBounds));
                    mid = pmid - &buildData[0];
                }
                
                else {
                    // Create leaf _BVHBuildNode_
                    uint32_t firstPrimOffset = orderedPrims.size();
                    for (uint32_t i = start; i < end; ++i) {
                        uint32_t primNum = buildData[i].primitiveNumber;
                        orderedPrims.push_back(primitives[primNum]);
                    }
                    node->InitLeaf(firstPrimOffset, nPrimitives, bbox);
                    return node;
                }
            }
            break;
        }
        }
        node->InitInterior(dim,
                           recursiveBuild(buildArena, buildData, start, mid,
                                          totalNodes, orderedPrims),
                           recursiveBuild(buildArena, buildData, mid, end,
                                          totalNodes, orderedPrims));
    }
    return node;
}



inline uint32_t mortonCode(uint32_t x, uint32_t y, uint32_t z) {
    // compute morton code from quantized coordinates. code from:
    // http://stackoverflow.com/questions/1024754/how-to-compute-a-3d-morton-number-interleave-the-bits-of-3-ints
    x = (x | (x << 16)) & 0x030000FF;
    x = (x | (x << 8)) & 0x0300F00F;
    x = (x | (x << 4)) & 0x030C30C3;
    x = (x | (x << 2)) & 0x09249249;

    y = (y | (y << 16)) & 0x030000FF;
    y = (y | (y << 8)) & 0x0300F00F;
    y = (y | (y << 4)) & 0x030C30C3;
    y = (y | (y << 2)) & 0x09249249;

    z = (z | (z << 16)) & 0x030000FF;
    z = (z | (z << 8)) & 0x0300F00F;
    z = (z | (z << 4)) & 0x030C30C3;
    z = (z | (z << 2)) & 0x09249249;

    uint32_t mortonCode = x | (y << 1) | (z << 2);
    assert((mortonCode & 0xC0000000) == 0);

    return mortonCode;
}

// AAC algorithm parameters
#define AAC_DELTA 4
#define AAC_EPSILON 0.2f

inline uint32_t maxClustersFunction(uint32_t x) {
    float c = 0.5f * pow(AAC_DELTA, 0.5f + AAC_EPSILON);
    float alpha = 0.5f - AAC_EPSILON;
    return Round2Int(c * pow(x, alpha));
}


struct BVHPrimitiveInfoAac {
    BVHPrimitiveInfoAac() { }
    void init(BVHPrimitiveInfo &primitiveInfo, uint32_t morton) {
        primitiveNumber = primitiveInfo.primitiveNumber;
        centroid = primitiveInfo.centroid;
        bounds = primitiveInfo.bounds;
        mortonCode = morton;
    }
    int primitiveNumber;
    Point centroid;
    BBox bounds;
    uint32_t mortonCode;
};

struct PrimitiveMorton {
    int primitiveNumber;
    uint32_t mortonCode;
};

// MSD radix sort based on
// http://rosettacode.org/wiki/Sorting_algorithms/Radix_sort#C.2B.2B
class MortonBitTest {
    const int bit;
public:
    MortonBitTest(int bit) : bit(bit) {}
    bool operator()(PrimitiveMorton& p) {       // passed to std::partition
        return !(p.mortonCode & (1 << bit));
    }
    bool operator()(BVHPrimitiveInfoAac& p) {   // passed to std::find_if
        return (p.mortonCode & (1 << bit));
    }
};
void msd_radix_sort(PrimitiveMorton *start, PrimitiveMorton *end, int msb = 29) {
    
    if (start != end && msb >= 0) {
        PrimitiveMorton *mid = std::partition(start, end, MortonBitTest(msb));
        msb--;
        msd_radix_sort(start, mid, msb);
        msd_radix_sort(mid, end, msb);
    }
}

BVHBuildNode *BVHAccel::aacBuild(MemoryArena &buildArena, 
    vector<BVHPrimitiveInfo> &buildData, uint32_t *totalNodes,
    vector<Reference<Primitive> > &orderedPrims) {

    // Find bbox of all primitive centroids
    BBox centroidBounds(buildData[0].centroid);
    for (uint32_t i = 1; i < buildData.size(); i++) {
        centroidBounds = Union(centroidBounds, buildData[i].centroid);
    }

    vector<PrimitiveMorton> mortonData(buildData.size());
    mortonData.begin();

    Vector centroidRange = centroidBounds.pMax - centroidBounds.pMin;
    for (uint32_t i = 0; i < buildData.size(); i++) {
        mortonData[i].primitiveNumber = buildData[i].primitiveNumber;

        // find quantized coordinates for the primitive centroid
        Point &c = buildData[i].centroid;
        int x = floor(1024.f * (c.x - centroidBounds.pMin.x) / centroidRange.x);
        if (x == 1024) x = 1023;
        int y = floor(1024.f * (c.y - centroidBounds.pMin.y) / centroidRange.y);
        if (y == 1024) y = 1023;
        int z = floor(1024.f * (c.z - centroidBounds.pMin.z) / centroidRange.z);
        if (z == 1024) z = 1023;
        assert(0 <= x && x < 1024);
        assert(0 <= y && y < 1024);
        assert(0 <= z && z < 1024);

        // compute morton code from quantized coordinates
        mortonData[i].mortonCode = mortonCode(x, y, z);;
    }

    // radix sort primitives by morton code
    msd_radix_sort(&mortonData.front(), &mortonData.back());

    vector<BVHPrimitiveInfoAac> buildDataAac(buildData.size());
    for (uint32_t i = 0; i < buildData.size(); i++) {
        buildDataAac[i].init(buildData[mortonData[i].primitiveNumber],
                             mortonData[i].mortonCode);
    }
    
    std::unordered_set<BVHBuildNode*> clusters = recursiveAacBuild(
        buildArena, buildDataAac, 0, buildDataAac.size(), 29, totalNodes,
        orderedPrims);

    combineClusters(buildArena, clusters, 1, totalNodes);

    return *(clusters.cbegin());
}



std::unordered_set<BVHBuildNode*> BVHAccel::recursiveAacBuild(
    MemoryArena &buildArena, vector<BVHPrimitiveInfoAac> &buildDataAac,
    uint32_t start, uint32_t end, int mortonSplitBit,
    uint32_t *totalNodes, vector<Reference<Primitive> > &orderedPrims) {

    uint32_t nPrimitives = end - start;
    if (nPrimitives < AAC_DELTA) {
        // intialize clusters with primitives (one cluster per primitive)
        std::unordered_set<BVHBuildNode*> clusters;
        for (uint32_t i = start; i < end; i++) {
            BVHBuildNode *node = buildArena.Alloc<BVHBuildNode>();

            // orderedPrims will be in Morton code sorted order.
            // It doesn't matter how primitives are ordered in orderedPrims;
            // each leaf in the final BVH will have only one primitive, so as
            // long as the BVHBuildNode.primitiveNumber points to the index in
            // orderedPrims that its primitve resides at, it will work.
            uint32_t firstPrimOffset = orderedPrims.size();
            orderedPrims.push_back(primitives[buildDataAac[i].primitiveNumber]);
            node->InitLeaf(firstPrimOffset, 1, buildDataAac[i].bounds);
            clusters.insert(node);
        }
        (*totalNodes) += nPrimitives;

        combineClusters(buildArena, clusters, maxClustersFunction(AAC_DELTA), totalNodes);
        return clusters;
    }

    // split primitives into two partitions at current morton code bit position
    // all primitives have same bit value at current position, move one bit right.
    // if all bits have been exhausted, just split in half
    uint32_t mid;
    switch (mortonSplitBit >= 0) {
    case true: {
        do {
            BVHPrimitiveInfoAac *pmid = std::find_if(&buildDataAac[start], &buildDataAac[end],
                MortonBitTest(mortonSplitBit));
            mid = pmid - &buildDataAac[0];
            mortonSplitBit--;
        } while ((mid == 0 || mid == end) && mortonSplitBit >= 0);
        // if we exhaust all morton code bits, fall through to the bits-exhausted case
        if (mortonSplitBit >= 0) {
            break;
        }
    }
    default: {
        mid = (start + end) / 2;
    }
    }

    // calculate clusters for left and right partions of primitives and find their union
    std::unordered_set<BVHBuildNode*> clusters = recursiveAacBuild(buildArena, buildDataAac,
        start, mid, mortonSplitBit, totalNodes, orderedPrims);
    std::unordered_set<BVHBuildNode*> rightClusters = recursiveAacBuild(buildArena, buildDataAac,
        mid, end, mortonSplitBit, totalNodes, orderedPrims);
    clusters.insert(rightClusters.begin(), rightClusters.end());

    combineClusters(buildArena, clusters, maxClustersFunction(nPrimitives), totalNodes);
    return clusters;
}

BVHBuildNode *findClosestCluster(const std::unordered_set<BVHBuildNode*> &clusters,
    BVHBuildNode *target, float *closestDistance) {

    float minDistance = INFINITY;
    BVHBuildNode *minDistanceCluster = NULL;
    for (auto it = clusters.cbegin(); it != clusters.cend(); it++) {
        BVHBuildNode *cluster = *it;
        if (cluster == target) continue;
        BBox unionBox = Union(target->bounds, cluster->bounds);
        float distance = unionBox.SurfaceArea();
        if (distance < minDistance) {
            minDistance = distance;
            minDistanceCluster = cluster;
        }
    }
    *closestDistance = minDistance;
    return minDistanceCluster;
}

void BVHAccel::combineClusters(MemoryArena &buildArena,
    std::unordered_set<BVHBuildNode*> &clusters,
    uint32_t maxClusters, uint32_t *totalNodes) {

    std::unordered_map<BVHBuildNode*, std::pair<BVHBuildNode*, float>> closestMap;
    
    float bestDistance = INFINITY;
    BVHBuildNode *left = NULL;
    BVHBuildNode *right = NULL;

    // for each cluster, find the cluster that's closest to it and its distance away
    for (auto it = clusters.cbegin(); it != clusters.cend(); it++) {
        BVHBuildNode *cluster = *it;
        float minDistance;
        BVHBuildNode* closestCluster = findClosestCluster(clusters, cluster, &minDistance);

        closestMap[cluster].first = closestCluster;
        closestMap[cluster].second = minDistance;

        // find the closest pair of clusters (will be first pair of clusters to combine)
        if (minDistance < bestDistance) {
            bestDistance = minDistance;
            left = cluster;
            right = closestCluster;
        }
    }

    for (uint32_t i = 0; i < clusters.size() - maxClusters; i++) {

        // combine pair of closest clusters
        (*totalNodes)++;
        BVHBuildNode *newCluster = buildArena.Alloc<BVHBuildNode>();
        newCluster->InitInterior(0, left, right);   // splitAxis set to 0

        // remove left, right clusters from list
        clusters.erase(left);
        clusters.erase(right);
        closestMap.erase(left);
        closestMap.erase(right);
        
        // find cluster closest to newly created cluster
        float minDistanceToNew;
        BVHBuildNode* closestClusterToNew = findClosestCluster(clusters, newCluster, &minDistanceToNew);


        // Iterate clusters list and simultaneously do the following:
        // Update closestMap for clusters whose closest was one of the two clusters that go combined;
        // Find the new closest pair of clusters
        bestDistance = minDistanceToNew;
        left = newCluster;
        right = closestClusterToNew;
        for (auto it = clusters.cbegin(); it != clusters.cend(); it++) {
            BVHBuildNode* cluster = *it;
            BVHBuildNode* closestCluster = closestMap[cluster].first;
            float minDistance;
            if (closestCluster == left || closestCluster == right) {
                closestCluster = findClosestCluster(clusters, cluster, &minDistance);
                closestMap[cluster].first = closestCluster;
                closestMap[cluster].second = minDistance;
            } else {
                minDistance = closestMap[cluster].second;
            }

            if (minDistance < bestDistance) {
                bestDistance = minDistance;
                left = cluster;
                right = closestCluster;
            }
        }

        // insert new cluster into list
        clusters.insert(newCluster);
        closestMap[newCluster].first = closestClusterToNew;
        closestMap[newCluster].second = minDistanceToNew;
    }
}



uint32_t BVHAccel::flattenBVHTree(BVHBuildNode *node, uint32_t *offset) {
    LinearBVHNode *linearNode = &nodes[*offset];
    linearNode->bounds = node->bounds;
    uint32_t myOffset = (*offset)++;
    if (node->nPrimitives > 0) {
        Assert(!node->children[0] && !node->children[1]);
        linearNode->primitivesOffset = node->firstPrimOffset;
        linearNode->nPrimitives = node->nPrimitives;
    }
    else {
        // Creater interior flattened BVH node
        linearNode->axis = node->splitAxis;
        linearNode->nPrimitives = 0;
        flattenBVHTree(node->children[0], offset);
        linearNode->secondChildOffset = flattenBVHTree(node->children[1],
                                                       offset);
    }
    return myOffset;
}


BVHAccel::~BVHAccel() {
    FreeAligned(nodes);
}


bool BVHAccel::Intersect(const Ray &ray, Intersection *isect) const {
    if (!nodes) return false;
    PBRT_BVH_INTERSECTION_STARTED(const_cast<BVHAccel *>(this), const_cast<Ray *>(&ray));
    bool hit = false;
    Vector invDir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
    uint32_t dirIsNeg[3] = { invDir.x < 0, invDir.y < 0, invDir.z < 0 };
    // Follow ray through BVH nodes to find primitive intersections
    uint32_t todoOffset = 0, nodeNum = 0;
    uint32_t todo[64];
    while (true) {
        const LinearBVHNode *node = &nodes[nodeNum];
        // Check ray against BVH node
        if (::IntersectP(node->bounds, ray, invDir, dirIsNeg)) {
            if (node->nPrimitives > 0) {
                // Intersect ray with primitives in leaf BVH node
                PBRT_BVH_INTERSECTION_TRAVERSED_LEAF_NODE(const_cast<LinearBVHNode *>(node));
                for (uint32_t i = 0; i < node->nPrimitives; ++i)
                {
                    PBRT_BVH_INTERSECTION_PRIMITIVE_TEST(const_cast<Primitive *>(primitives[node->primitivesOffset+i].GetPtr()));
                    if (primitives[node->primitivesOffset+i]->Intersect(ray, isect))
                    {
                        PBRT_BVH_INTERSECTION_PRIMITIVE_HIT(const_cast<Primitive *>(primitives[node->primitivesOffset+i].GetPtr()));
                        hit = true;
                    }
                    else {
                        PBRT_BVH_INTERSECTION_PRIMITIVE_MISSED(const_cast<Primitive *>(primitives[node->primitivesOffset+i].GetPtr()));
                   }
                }
                if (todoOffset == 0) break;
                nodeNum = todo[--todoOffset];
            }
            else {
                // Put far BVH node on _todo_ stack, advance to near node
                PBRT_BVH_INTERSECTION_TRAVERSED_INTERIOR_NODE(const_cast<LinearBVHNode *>(node));
                if (dirIsNeg[node->axis]) {
                   todo[todoOffset++] = nodeNum + 1;
                   nodeNum = node->secondChildOffset;
                }
                else {
                   todo[todoOffset++] = node->secondChildOffset;
                   nodeNum = nodeNum + 1;
                }
            }
        }
        else {
            if (todoOffset == 0) break;
            nodeNum = todo[--todoOffset];
        }
    }
    PBRT_BVH_INTERSECTION_FINISHED();
    return hit;
}


bool BVHAccel::IntersectP(const Ray &ray) const {
    if (!nodes) return false;
    PBRT_BVH_INTERSECTIONP_STARTED(const_cast<BVHAccel *>(this), const_cast<Ray *>(&ray));
    Vector invDir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
    uint32_t dirIsNeg[3] = { invDir.x < 0, invDir.y < 0, invDir.z < 0 };
    uint32_t todo[64];
    uint32_t todoOffset = 0, nodeNum = 0;
    while (true) {
        const LinearBVHNode *node = &nodes[nodeNum];
        if (::IntersectP(node->bounds, ray, invDir, dirIsNeg)) {
            // Process BVH node _node_ for traversal
            if (node->nPrimitives > 0) {
                PBRT_BVH_INTERSECTIONP_TRAVERSED_LEAF_NODE(const_cast<LinearBVHNode *>(node));
                  for (uint32_t i = 0; i < node->nPrimitives; ++i) {
                    PBRT_BVH_INTERSECTIONP_PRIMITIVE_TEST(const_cast<Primitive *>(primitives[node->primitivesOffset + i].GetPtr()));
                    if (primitives[node->primitivesOffset+i]->IntersectP(ray)) {
                        PBRT_BVH_INTERSECTIONP_PRIMITIVE_HIT(const_cast<Primitive *>(primitives[node->primitivesOffset+i].GetPtr()));
                        return true;
                    }
                else {
                        PBRT_BVH_INTERSECTIONP_PRIMITIVE_MISSED(const_cast<Primitive *>(primitives[node->primitivesOffset + i].GetPtr()));
                    }
                }
                if (todoOffset == 0) break;
                nodeNum = todo[--todoOffset];
            }
            else {
                PBRT_BVH_INTERSECTIONP_TRAVERSED_INTERIOR_NODE(const_cast<LinearBVHNode *>(node));
                if (dirIsNeg[node->axis]) {
                   /// second child first
                   todo[todoOffset++] = nodeNum + 1;
                   nodeNum = node->secondChildOffset;
                }
                else {
                   todo[todoOffset++] = node->secondChildOffset;
                   nodeNum = nodeNum + 1;
                }
            }
        }
        else {
            if (todoOffset == 0) break;
            nodeNum = todo[--todoOffset];
        }
    }
    PBRT_BVH_INTERSECTIONP_FINISHED();
    return false;
}


BVHAccel *CreateBVHAccelerator(const vector<Reference<Primitive> > &prims,
        const ParamSet &ps) {
    string splitMethod = ps.FindOneString("splitmethod", "sah");
    uint32_t maxPrimsInNode = ps.FindOneInt("maxnodeprims", 4);
    return new BVHAccel(prims, maxPrimsInNode, splitMethod);
}


