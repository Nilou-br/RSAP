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
class RSAPSHARED_API FRsapNavmeshOldUpdater
{
	FRsapNavmeshOld& Navmesh;
	FRsapDirtyNavmesh DirtyNavmesh;
	
	std::unordered_set<std::shared_ptr<FRsapCollisionComponent>> StagedComponents;

public:
	explicit FRsapNavmeshOldUpdater(FRsapNavmeshOld& InNavmesh) : Navmesh(InNavmesh){}

	void StageComponent(const std::shared_ptr<FRsapCollisionComponent>& Component)
	{
		// todo: mutex?
		StagedComponents.insert(Component);
		std::weak_ptr ComponentPtr = Component;

		Component->ForEachDirtyNode([&DirtyNavmesh = DirtyNavmesh, &ComponentPtr = ComponentPtr](const chunk_morton ChunkMC, const node_morton NodeMC, const layer_idx LayerIdx)
		{
			FRsapDirtyChunk* DirtyChunk = DirtyNavmesh.FindChunk(ChunkMC);
			if(!DirtyChunk) DirtyChunk = &DirtyNavmesh.InitChunk(ChunkMC);

			bool bWasInserted;
			FRsapDirtyNode& DirtyNode = DirtyChunk->TryInitNode(bWasInserted, NodeMC, LayerIdx);
			if(bWasInserted) DirtyChunk->InitNodeParents(NodeMC, LayerIdx);

			DirtyNode.Components.insert(ComponentPtr);
		});
	}
};