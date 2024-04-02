// Copyright Melvin Brink 2023. All Rights Reserved.

#include "MBNavigation/NavMesh/Updater.h"
#include <set>

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

	// For-each chunk that these bounds are inside of.
	const auto ForEachChunk = [&](const TBounds<F3DVector32>& Bounds, auto Callback)
	{
		const F3DVector32 ChunkMin = Bounds.Min & FNavMeshStatic::ChunkMask;
		const F3DVector32 ChunkMax = Bounds.Max & FNavMeshStatic::ChunkMask;
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
					Callback(&ChunkIterator->second);
				}
			}
		}
	};
	
	for (const auto BoundsPair : BeforeAfterBoundsPairs)
	{
		// Get the layer-index used as the starting point for the overlap checks.
		const uint8 StartingLayer = GetStartingLayer(BoundsPair);
		
		ForEachChunk(BoundsPair.GetTotalBounds(), [&](FChunk* Chunk)
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
			
			// Check every parent that have had their children affected.
			RecursiveClearParents(Chunk, ParentMortonCodes, StartingLayer);
		});
	}

#if WITH_EDITOR
	const float DurationSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::high_resolution_clock::now() - StartTime).count() / 1000.0f;
	// UE_LOG(LogNavMeshUpdater, Log, TEXT("Update took : '%f' seconds"), DurationSeconds);
#endif
}

template <typename Func>
void FNavMeshUpdater::ForEachNodeInBounds(const TBounds<F3DVector10> Bounds, const FChunk* Chunk, const uint8 LayerIndex, Func Callback)
{
	for (uint_fast16_t X = Bounds.Min.X; X < Bounds.Max.X; X+=FNavMeshStatic::MortonOffsets[LayerIndex]) {
		for (uint_fast16_t Y = Bounds.Min.Y; Y < Bounds.Max.Y; Y+=FNavMeshStatic::MortonOffsets[LayerIndex]) {
			for (uint_fast16_t Z = Bounds.Min.Z; Z < Bounds.Max.Z; Z+=FNavMeshStatic::MortonOffsets[LayerIndex]) {
				const auto NodeIterator = Chunk->Octrees[0]->Layers[LayerIndex].find(F3DVector10(X, Y, Z).ToMortonCode());
				if(NodeIterator == Chunk->Octrees[0]->Layers[LayerIndex].end()) return;
				FOctreeNode& Node = NodeIterator->second;
				Callback(Node);
			}
		}
	}
}

std::unordered_set<uint_fast32_t> FNavMeshUpdater::HandlePrevBounds(const FChunk* Chunk, const TBounds<F3DVector10> PrevBounds, const TBounds<F3DVector10> CurrBounds, const uint8 LayerIndex)
{
	if(!PrevBounds.IsValid()) return{};
	
	// If current-bounds are not valid, then we don't need to get any remainder.
	if(!CurrBounds.IsValid())
	{
		return ClearUnoccludedBounds(Chunk, PrevBounds, LayerIndex);
	}

	// Get remainder of intersection between previous and current bounds.
	for (const auto RemainingBounds : PrevBounds.GetRemainder(CurrBounds))
	{
		return ClearUnoccludedBounds(Chunk, RemainingBounds, LayerIndex);
	}

	return {};
}

std::unordered_set<uint_fast32_t> FNavMeshUpdater::ClearUnoccludedBounds(const FChunk* Chunk, const TBounds<F3DVector10> Bounds, const uint8 Layer)
{
	Bounds.Draw(World, Chunk->Location, FColor::Red);
	std::unordered_set<uint_fast32_t> ParentMortonCodes;
	
	// First check if these bounds overlap with anything in the world.
	if(const TBounds<F3DVector32> GlobalBounds = Bounds.ToGlobalSpace(Chunk->Location); !GlobalBounds.HasOverlap(World))
	{
		// There is no overlap, so we can clear all nodes inside at once.
		for (uint_fast16_t X = Bounds.Min.X; X < Bounds.Max.X; X+=FNavMeshStatic::MortonOffsets[Layer])
		{
			for (uint_fast16_t Y = Bounds.Min.Y; Y < Bounds.Max.Y; Y+=FNavMeshStatic::MortonOffsets[Layer])
			{
				for (uint_fast16_t Z = Bounds.Min.Z; Z < Bounds.Max.Z; Z+=FNavMeshStatic::MortonOffsets[Layer])
				{
					const auto NodeIterator = Chunk->Octrees[0]->Layers[Layer].find(F3DVector10(X, Y, Z).ToMortonCode());
					if(NodeIterator == Chunk->Octrees[0]->Layers[Layer].end()) continue;
					FOctreeNode& Node = NodeIterator->second;

					Node.SetOccluded(false);
					ParentMortonCodes.insert(Node.GetParentMortonCode(Layer));
			
					// Clear the children on this node if it has any.
					if(Layer < FNavMeshStatic::StaticDepth && Node.IsFilled())
					{
						RecursiveClearAllChildren(Chunk, Node, Layer);
						Node.SetFilled(false);
					}
				}
			}
		}
		return ParentMortonCodes;
	}
	
	// There is an overlap, so each node should be checked manually.
	Bounds.ForEachPoint(FNavMeshStatic::MortonOffsets[Layer], [&](const F3DVector10 MortonLocation) -> void
	{
		const auto NodeIterator = Chunk->Octrees[0]->Layers[Layer].find(MortonLocation.ToMortonCode());
		if(NodeIterator == Chunk->Octrees[0]->Layers[Layer].end()) return;
		FOctreeNode& Node = NodeIterator->second;

		if(!Node.HasOverlap(World, Chunk->Location, Layer))
		{
			Node.SetOccluded(false);
			ParentMortonCodes.insert(Node.GetParentMortonCode(Layer));

			// Clear the children on this node if it has any.
			if(Layer < FNavMeshStatic::StaticDepth && Node.IsFilled())
			{
				RecursiveClearAllChildren(Chunk, Node, Layer);
				Node.SetFilled(false);
			}
			return;
		}
		RecursiveClearUnoccludedChildren(Chunk, Node, Layer);
	});
	
	return ParentMortonCodes;
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
		const uint8 ChildLayer = LayerIndex+1;
		if(ChildNode->HasOverlap(World, Chunk->Location, ChildLayer))
		{
			// Keep searching for unoccluded nodes.
			RecursiveClearUnoccludedChildren(Chunk, *ChildNode, ChildLayer+1);
			return;
		}

		// No overlap, so update node and clear all its children.
		ChildNode->SetFilled(false);
		ChildNode->SetOccluded(false);
		RecursiveClearAllChildren(Chunk, *ChildNode, ChildLayer+1);
	});
}

void FNavMeshUpdater::RecursiveClearAllChildren(const FChunk* Chunk, const FOctreeNode& Node, const uint8 LayerIndex)
{
	Chunk->ForEachChildOfNode(Node, LayerIndex, [&](FOctreeNode* ChildNode)
	{
		const uint8 ChildLayer = LayerIndex+1;
		if(ChildLayer < FNavMeshStatic::StaticDepth && ChildNode->IsFilled())
		{
			RecursiveClearAllChildren(Chunk, *ChildNode, ChildLayer+1);
		}
		Chunk->Octrees[0]->Layers[ChildLayer].erase(ChildNode->GetMortonCode());
	});
}

// OPTIMIZE
void FNavMeshUpdater::RecursiveClearParents(const FChunk* Chunk, const std::unordered_set<uint_fast32_t>& ParentMortonCodes, const uint8 ChildLayerIndex)
{
	if(ChildLayerIndex == 0) return;
	const uint8 ParentLayer = ChildLayerIndex-1;

	std::unordered_set<uint_fast32_t> GrandParentMortonCodes;
	for (auto ParentMortonCode : ParentMortonCodes)
	{
		const auto NodeIterator = Chunk->Octrees[0]->Layers[ParentLayer].find(ParentMortonCode);
		FOctreeNode& ParentNode = NodeIterator->second;
				
		bool bDeleteChildren = true;
		TArray<uint_fast32_t> ChildMortonCodes;
		Chunk->ForEachChildOfNode(ParentNode, ParentLayer, [&](const FOctreeNode* ChildNode) -> void
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

		GrandParentMortonCodes.insert(ParentNode.GetParentMortonCode(ParentLayer));
	}

	if(!GrandParentMortonCodes.empty()) RecursiveClearParents(Chunk, GrandParentMortonCodes, ParentLayer);
}