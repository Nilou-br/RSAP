// Copyright Melvin Brink 2023. All Rights Reserved.

#include "MBNavigation/NavMesh/Updater.h"
#include <set>
#include "MBNavigation/Types/NavMesh.h"

DEFINE_LOG_CATEGORY(LogNavMeshUpdater)



uint8 GetStartingLayer(const TBoundsPair<F3DVector32>& BoundsPair)
{
	uint8 StartingLayer = FNavMeshStatic::StaticDepth;

	// We only actually need one of the bounds in this pair, so just use current.
	const TBounds<F3DVector32> Bounds = BoundsPair.Current;

	// Get its largest side.
	const int32 MaxSide = Bounds.GetLengths().GetMax();

	// Get the first layer where the node fits at-least 3 times in the object.
	for (uint8 LayerIndex = 0; LayerIndex<FNavMeshStatic::StaticDepth; ++LayerIndex)
	{
		if(MaxSide / FNavMeshStatic::NodeSizes[LayerIndex] <= 1) continue;
		StartingLayer = LayerIndex;
		break;
	}
	
	return StartingLayer;
}

void FNavMeshUpdater::UpdateStatic(const TArray<TBoundsPair<F3DVector32>>& BeforeAfterBoundsPairs)
{
	
#if WITH_EDITOR
	FlushPersistentDebugLines(World);
	const auto StartTime = std::chrono::high_resolution_clock::now();
#endif
	
	for (const auto BoundsPair : BeforeAfterBoundsPairs)
	{
		// Get the layer-index used as the starting point for the overlap checks.
		const uint8 StartingLayer = GetStartingLayer(BoundsPair);

		// Get the chunks these bounds are inside of.
		for (const FChunk* Chunk : GetChunksFromBoundsPair(BoundsPair))
		{
			TBounds<F3DVector32> ChunkBounds = Chunk->GetBounds();
			
			// These bounds are the remainder of an intersection between the bounds and the chunk.
			const TBounds<F3DVector32> PrevBounds = BoundsPair.Previous.Overlaps(ChunkBounds)
				? BoundsPair.Previous.GetIntersection(ChunkBounds) : TBounds<F3DVector32>();
			const TBounds<F3DVector32> CurrBounds = BoundsPair.Current.Overlaps(ChunkBounds)
				? BoundsPair.Current.GetIntersection(ChunkBounds) : TBounds<F3DVector32>();

			// Convert them from global to morton-space, and round them to the nearest multiple of the node-size of this layer.
			const TBounds<F3DVector10> PrevRoundedBounds = PrevBounds.ToMortonSpace(Chunk->Location).Round(StartingLayer);
			const TBounds<F3DVector10> CurrRoundedBounds = CurrBounds.ToMortonSpace(Chunk->Location).Round(StartingLayer);

			// Debug draw
			if(CurrBounds.IsValid()) CurrRoundedBounds.Draw(World, Chunk->Location, FColor::Green);

			// Keep track of all the parents of affected nodes.
			std::unordered_set<uint_fast32_t> ParentMortonCodes;
			
			const auto PrevResult = HandlePrevBounds(Chunk, PrevRoundedBounds, CurrRoundedBounds, StartingLayer);
			ParentMortonCodes.insert(PrevResult.begin(), PrevResult.end());
			const auto CurrResult = HandleCurrentBounds(Chunk, CurrRoundedBounds, StartingLayer);
			ParentMortonCodes.insert(CurrResult.begin(), CurrResult.end());
			
			// check every parent that have had their children affected.
			RecursiveClearParents(Chunk, ParentMortonCodes, StartingLayer);
		}
	}

#if WITH_EDITOR
	const float DurationSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::high_resolution_clock::now() - StartTime).count() / 1000.0f;
	// UE_LOG(LogNavMeshUpdater, Log, TEXT("Update took : '%f' seconds"), DurationSeconds);
#endif
}

TArray<FChunk*> FNavMeshUpdater::GetChunksFromBoundsPair(const TBoundsPair<F3DVector32>& BoundsPair) const
{
	const TBounds<F3DVector32> TotalBounds = BoundsPair.GetTotalBounds();
	const F3DVector32 ChunkMin = TotalBounds.Min & FNavMeshStatic::ChunkMask;
	const F3DVector32 ChunkMax = TotalBounds.Max & FNavMeshStatic::ChunkMask;
	
	TArray<FChunk*> Chunks;
	for (int32 X = ChunkMin.X; X <= ChunkMax.X; X+=FNavMeshStatic::ChunkSize)
	{
		for (int32 Y = ChunkMin.Y; Y <= ChunkMax.Y; Y+=FNavMeshStatic::ChunkSize)
		{
			for (int32 Z = ChunkMin.Z; Z <= ChunkMax.Z; Z+=FNavMeshStatic::ChunkSize)
			{
				const F3DVector32 ChunkLocation = F3DVector32(X, Y, Z);
				
				auto ChunkIterator = NavMeshPtr->find(ChunkLocation.ToKey());
				if(ChunkIterator == NavMeshPtr->end())
				{
					// Init new one if it does not exists yet.
					std::tie(ChunkIterator, std::ignore) = NavMeshPtr->emplace(ChunkLocation.ToKey(), FChunk(ChunkLocation));
				}
				
				FChunk* Chunk = &ChunkIterator->second;
				Chunks.Add(Chunk);
			}
		}
	}
	return Chunks;
}

std::unordered_set<uint_fast32_t> FNavMeshUpdater::HandlePrevBounds(const FChunk* Chunk, const TBounds<F3DVector10> PrevBounds, const TBounds<F3DVector10> CurrBounds, const uint8 LayerIndex)
{
	if(!PrevBounds.IsValid()) return{};
	
	// Lambda for checking and updating these bounds.
	const auto CheckBounds = [&](const TBounds<F3DVector10> Bounds)
	{
		Bounds.Draw(World, Chunk->Location, FColor::Red);
		std::unordered_set<uint_fast32_t> ParentMortonCodes;
		// First check if these bounds overlap with anything in the world.
		if(const TBounds<F3DVector32> GlobalBounds = Bounds.ToGlobalSpace(Chunk->Location); !GlobalBounds.HasOverlap(World))
		{
			// There is no overlap, so we can clear all nodes inside at once.
			Bounds.ForEachPoint(FNavMeshStatic::MortonOffsets[LayerIndex], [&, LayerIndex](const F3DVector10 MortonLocation) -> void
			{
				if(!Chunk->Octrees[0].IsValid())
				{
					UE_LOG(LogNavMeshUpdater, Log, TEXT("Problem"));
				}
				const auto NodeIterator = Chunk->Octrees[0]->Layers[LayerIndex].find(MortonLocation.ToMortonCode());
				if(NodeIterator == Chunk->Octrees[0]->Layers[LayerIndex].end()) return;
				FOctreeNode& Node = NodeIterator->second;
				
				Node.SetOccluded(false);
				ParentMortonCodes.insert(Node.GetParentMortonCode(LayerIndex));
				
				// Clear the children on this node if it has any.
				if(LayerIndex < FNavMeshStatic::StaticDepth && Node.IsFilled())
				{
					RecursiveClearAllChildren(Chunk, Node, LayerIndex);
					Node.SetFilled(false);
				}
			});
		}
		else
		{
			// There is an overlap, so each node should be checked manually.
			Bounds.ForEachPoint(FNavMeshStatic::MortonOffsets[LayerIndex], [&](const F3DVector10 MortonLocation) -> void
			{
				if(!Chunk->Octrees[0].IsValid())
				{
					UE_LOG(LogNavMeshUpdater, Log, TEXT("Problem"));
				}
				const auto NodeIterator = Chunk->Octrees[0]->Layers[LayerIndex].find(MortonLocation.ToMortonCode());
				if(NodeIterator == Chunk->Octrees[0]->Layers[LayerIndex].end()) return;
				FOctreeNode& Node = NodeIterator->second;

				if(!Node.HasOverlap(World, Chunk->Location, LayerIndex))
				{
					Node.SetOccluded(false);
					ParentMortonCodes.insert(Node.GetParentMortonCode(LayerIndex));

					// Clear the children on this node if it has any.
					if(LayerIndex < FNavMeshStatic::StaticDepth && Node.IsFilled())
					{
						RecursiveClearAllChildren(Chunk, Node, LayerIndex);
						Node.SetFilled(false);
					}
					return;
				}

				RecursiveClearUnoccludedChildren(Chunk, Node, LayerIndex);
			});
		}
		return ParentMortonCodes;
	};
	
	// If current-bounds are not valid, then we don't need to get any remainder.
	if(!CurrBounds.IsValid())
	{
		return CheckBounds(PrevBounds);
	}

	// Get remainder of intersection between previous and current bounds.
	for (const auto RemainingBounds : PrevBounds.GetRemainder(CurrBounds))
	{
		return CheckBounds(RemainingBounds);
	}

	return {};
}

std::unordered_set<uint_fast32_t> FNavMeshUpdater::HandleCurrentBounds(const FChunk* Chunk, const TBounds<F3DVector10> CurrBounds, const uint8 LayerIndex)
{
	if(!CurrBounds.IsValid()) return{};
	return {};
}

void FNavMeshUpdater::RecursiveClearUnoccludedChildren(const FChunk* Chunk, const FOctreeNode& Node, const uint8 LayerIndex)
{
	Chunk->ForEachChildOfNode(Node, LayerIndex, [&](FOctreeNode* ChildNode)
	{
		const uint8 ChildLayerIndex = LayerIndex+1;
		if(ChildNode->HasOverlap(World, Chunk->Location, ChildLayerIndex))
		{
			// Keep searching for unoccluded nodes.
			RecursiveClearUnoccludedChildren(Chunk, *ChildNode, ChildLayerIndex+1);
			return;
		}

		// No overlap, so update node and clear all its children.
		ChildNode->SetFilled(false);
		ChildNode->SetOccluded(false);
		RecursiveClearAllChildren(Chunk, *ChildNode, ChildLayerIndex+1);
	});
}

void FNavMeshUpdater::RecursiveClearAllChildren(const FChunk* Chunk, const FOctreeNode& Node, const uint8 LayerIndex)
{
	Chunk->ForEachChildOfNode(Node, LayerIndex, [&](FOctreeNode* ChildNode)
	{
		const uint8 ChildLayerIndex = LayerIndex+1;
		if(ChildLayerIndex < FNavMeshStatic::StaticDepth && ChildNode->IsFilled())
		{
			RecursiveClearAllChildren(Chunk, *ChildNode, ChildLayerIndex+1);
		}
		Chunk->Octrees[0]->Layers[ChildLayerIndex].erase(ChildNode->GetMortonCode());
	});
}

// OPTIMIZE
void FNavMeshUpdater::RecursiveClearParents(const FChunk* Chunk, const std::unordered_set<uint_fast32_t>& ParentMortonCodes, const uint8 ChildLayerIndex)
{
	if(ChildLayerIndex == 0) return;
	const uint8 ParentLayerIndex = ChildLayerIndex-1;

	std::unordered_set<uint_fast32_t> GrandParentMortonCodes;
	for (auto ParentMortonCode : ParentMortonCodes)
	{
		const auto NodeIterator = Chunk->Octrees[0]->Layers[ParentLayerIndex].find(ParentMortonCode);
		FOctreeNode& ParentNode = NodeIterator->second;
				
		bool bDeleteChildren = true;
		TArray<uint_fast32_t> ChildMortonCodes;
		Chunk->ForEachChildOfNode(ParentNode, ParentLayerIndex, [&](const FOctreeNode* ChildNode) -> void
		{
			ChildMortonCodes.Add(ChildNode->GetMortonCode());
			if(bDeleteChildren && ChildNode->IsOccluded()) bDeleteChildren = false;
		});
		if(!bDeleteChildren) continue;

		ParentNode.SetFilled(false);
		ParentNode.SetOccluded(false);
		for (auto ChildMortonCode : ChildMortonCodes)
		{
			Chunk->Octrees[0]->Layers[ChildLayerIndex].erase(ChildMortonCode);
		}

		GrandParentMortonCodes.insert(ParentNode.GetParentMortonCode(ParentLayerIndex));
	}

	if(!GrandParentMortonCodes.empty()) RecursiveClearParents(Chunk, GrandParentMortonCodes, ParentLayerIndex);
}