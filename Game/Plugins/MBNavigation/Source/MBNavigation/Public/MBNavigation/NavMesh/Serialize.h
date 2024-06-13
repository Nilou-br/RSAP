// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "MBNavigation/Types/NavMesh.h"
#include "MBNavigation/Types/Math.h"

// todo: move to NavMesh folder

MBNAVIGATION_API void SerializeNavMesh(FNavMesh& NavMesh, FGuid& ID);
MBNAVIGATION_API bool DeserializeNavMesh(FNavMesh& OutNavMesh, FGuid& OutID);

MBNAVIGATION_API FArchive& operator<<(FArchive& Ar, FGlobalVector& GlobalVector);
MBNAVIGATION_API FArchive& operator<<(FArchive& Ar, FNodeRelations& Relations);
MBNAVIGATION_API FArchive& operator<<(FArchive& Ar, FNode& Node);
MBNAVIGATION_API FArchive& operator<<(FArchive& Ar, FOctreeLayer& Layer);
MBNAVIGATION_API FArchive& operator<<(FArchive& Ar, TSharedPtr<FOctree>& Octree);
MBNAVIGATION_API FArchive& operator<<(FArchive& Ar, FChunk& Chunk);
MBNAVIGATION_API FArchive& operator<<(FArchive& Ar, FNavMesh& NavMesh);