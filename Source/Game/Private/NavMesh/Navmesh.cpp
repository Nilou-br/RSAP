// Copyright Melvin Brink 2023. All Rights Reserved.

#include "Rsap/NavMesh/Navmesh.h"
#include "Rsap/NavMesh/Processing/Generator.h"

void FRsapNavmesh::Generate(const UWorld* World, const FActorMap& ActorMap)
{
	// todo: use RsapEditorWorld directly, and have boolean on it that determines if world is ready, then get actors from it.
	if(!World) return;
	
	Chunks.clear();
	FRsapGenerator::Generate(World, *this, ActorMap);
}
