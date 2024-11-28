// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "Navmesh.h"



/**
 * Responsible for updating the navmesh asynchronously.
 * Stores a reference to the navmesh you would like to be updated.
 *
 * The updater will run after calling ::Start
 * You can pause the updater by calling ::Pause which will finish the current node being processed.
 *
 * Pause the updater when you want to use the navmesh to avoid race conditions.
 */
class RSAPGAME_API FRsapNavmeshUpdater
{
	FRsapNavmesh& Navmesh;
	FRsapDirtyNavmesh DirtyNavmesh;
	
	Rsap::Map::flat_map<FRsapCollisionComponentPtr, std::vector<FRsapFlatChunk>> StagedComponentEntries;

public:
	explicit FRsapNavmeshUpdater(FRsapNavmesh& InNavmesh) : Navmesh(InNavmesh){}

	void StageComponent(const FRsapCollisionComponentPtr& ComponentPtr, const std::vector<FRsapFlatChunk>& IntersectedChunks)
	{
		
	}
};