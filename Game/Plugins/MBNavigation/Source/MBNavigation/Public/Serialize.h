#pragma once

#include "NavMeshTypes.h"


MBNAVIGATION_API void SerializeNavMesh(FNavMesh& NavMesh, FGuid& ID);
MBNAVIGATION_API bool DeserializeNavMesh(FNavMesh& OutNavMesh, FGuid& OutID);

MBNAVIGATION_API FArchive& operator<<(FArchive& Ar, F3DVector16& Vector16);
MBNAVIGATION_API FArchive& operator<<(FArchive& Ar, F3DVector32& Vector32);
MBNAVIGATION_API FArchive& operator<<(FArchive& Ar, FOctreeNeighbours& OctreeNeighbours);
MBNAVIGATION_API FArchive& operator<<(FArchive& Ar, FOctreeNode& OctreeNode);
MBNAVIGATION_API FArchive& operator<<(FArchive& Ar, FNodesMap& NodesMap);
MBNAVIGATION_API FArchive& operator<<(FArchive& Ar, TSharedPtr<FOctree>& Octree);
MBNAVIGATION_API FArchive& operator<<(FArchive& Ar, FChunk& Chunk);
MBNAVIGATION_API FArchive& operator<<(FArchive& Ar, FNavMesh& NavMesh);

template<typename Archive, typename KeyType, typename ValueType>
MBNAVIGATION_API Archive& SerializeMap(Archive& Ar, ankerl::unordered_dense::map<KeyType, ValueType>& Map);