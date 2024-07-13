// Copyright Melvin Brink 2023. All Rights Reserved.

#include "MBNavigation/NavMesh/Types/Node.h"
#include "MBNavigation/NavMesh/Math/Vectors.h"
#include "MBNavigation/NavMesh/Types/Static.h"


FNode::FNode(const uint8 ChildIdx, const DirectionType ParentChunkBorder)
{
	if (ParentChunkBorder)
	{
		ChunkBorder |= ChildIdx & 1 ? Direction::X_Positive : Direction::X_Negative;
		ChunkBorder |= ChildIdx & 2 ? Direction::Y_Positive : Direction::Y_Negative;
		ChunkBorder |= ChildIdx & 4 ? Direction::Z_Positive : Direction::Z_Negative;
		ChunkBorder &= ParentChunkBorder; // Can only be against the same border(s) as the parent.
	}
}

// // For the given node, set its children's relation in the Direction to the given LayerIdxToSet. Only the children against the same border in this direction will be updated.
// static void UpdateChildRelations(const FChunk* Chunk, const FNodePair& NodePair, const LayerIdxType LayerIdx, const LayerIdxType LayerIdxToSet, const DirectionType Direction)
// {
// 	if(!NodePair.second.HasChildren()) return;
// 	
// 	const FMortonVector ParentLocation = FMortonVector::FromMortonCode(NodePair.first);
// 	const LayerIdxType ChildLayerIdx = LayerIdx+1;
// 	const uint16 MortonOffset = FNavMeshStatic::MortonOffsets[ChildLayerIdx];
//
// 	// Get the morton-codes of the children facing the direction. ( Children against the border of their parent in this direction. )
// 	std::array<NodeMortonType, 4> ChildMortonCodes;
// 	switch (Direction)
// 	{
// 		case Direction::X_Negative:
// 			ChildMortonCodes[0] = (ParentLocation+FMortonVector()).ToMortonCode();																// 1st child
// 			ChildMortonCodes[1] = (ParentLocation+FMortonVector(0,					MortonOffset,	0)).ToMortonCode();				// 3rd child
// 			ChildMortonCodes[2] = (ParentLocation+FMortonVector(0,					0,				MortonOffset)).ToMortonCode();	// 5th child
// 			ChildMortonCodes[3] = (ParentLocation+FMortonVector(0,					MortonOffset,	MortonOffset)).ToMortonCode();	// 7th child
// 			break;
// 		case Direction::Y_Negative:
// 			ChildMortonCodes[0] = (ParentLocation+FMortonVector()).ToMortonCode();																// 1st child
// 			ChildMortonCodes[1] = (ParentLocation+FMortonVector(MortonOffset,		0,	0)).ToMortonCode();							// 2nd child
// 			ChildMortonCodes[2] = (ParentLocation+FMortonVector(0,					0,	MortonOffset)).ToMortonCode();				// 5th child
// 			ChildMortonCodes[3] = (ParentLocation+FMortonVector(MortonOffset,		0,	MortonOffset)).ToMortonCode();				// 6th child
// 			break;
// 		case Direction::Z_Negative:
// 			ChildMortonCodes[0] = (ParentLocation+FMortonVector()).ToMortonCode();																// 1st child
// 			ChildMortonCodes[1] = (ParentLocation+FMortonVector(MortonOffset,		0,				0)).ToMortonCode();				// 2nd child
// 			ChildMortonCodes[2] = (ParentLocation+FMortonVector(0,					MortonOffset,	0)).ToMortonCode();				// 3rd child
// 			ChildMortonCodes[3] = (ParentLocation+FMortonVector(MortonOffset,		MortonOffset,	0)).ToMortonCode();				// 4th child
// 			break;
// 		case Direction::X_Positive:
// 			ChildMortonCodes[0] = (ParentLocation+FMortonVector(MortonOffset,		0,				0)).ToMortonCode();				// 2nd child
// 			ChildMortonCodes[1] = (ParentLocation+FMortonVector(MortonOffset,		MortonOffset,	0)).ToMortonCode();				// 4th child
// 			ChildMortonCodes[2] = (ParentLocation+FMortonVector(MortonOffset,		0,				MortonOffset)).ToMortonCode();	// 6th child
// 			ChildMortonCodes[3] = (ParentLocation+FMortonVector(MortonOffset,		MortonOffset,	MortonOffset)).ToMortonCode();	// 8th child
// 			break;
// 		case Direction::Y_Positive:
// 			ChildMortonCodes[0] = (ParentLocation+FMortonVector(0,					MortonOffset,	0)).ToMortonCode();				// 3rd child
// 			ChildMortonCodes[1] = (ParentLocation+FMortonVector(MortonOffset,		MortonOffset,	0)).ToMortonCode();				// 4th child
// 			ChildMortonCodes[2] = (ParentLocation+FMortonVector(0,					MortonOffset,	MortonOffset)).ToMortonCode();	// 7th child
// 			ChildMortonCodes[3] = (ParentLocation+FMortonVector(MortonOffset,		MortonOffset,	MortonOffset)).ToMortonCode();	// 8th child
// 			break;
// 		case Direction::Z_Positive:
// 			ChildMortonCodes[0] = (ParentLocation+FMortonVector(0,					0,				MortonOffset)).ToMortonCode();	// 5th child
// 			ChildMortonCodes[1] = (ParentLocation+FMortonVector(MortonOffset,		0,				MortonOffset)).ToMortonCode();	// 6th child
// 			ChildMortonCodes[2] = (ParentLocation+FMortonVector(0,					MortonOffset,	MortonOffset)).ToMortonCode();	// 7th child
// 			ChildMortonCodes[3] = (ParentLocation+FMortonVector(MortonOffset,		MortonOffset,	MortonOffset)).ToMortonCode();	// 8th child
// 			break;
// 		default: break;
// 	}
//
// 	// Update each child's relation in this direction to the LayerIdxToSet. Recursively do the same for their children in this direction.
// 	for (auto ChildMortonCode : ChildMortonCodes)
// 	{
// 		const auto ChildNodeIterator = Chunk->Octrees[0]->Layers[ChildLayerIdx]->find(ChildMortonCode);
// 		ChildNodeIterator->second.Relations.SetFromDirection(LayerIdxToSet, Direction);
// 		UpdateChildRelations(Chunk, *ChildNodeIterator, ChildLayerIdx, LayerIdxToSet, Direction);
// 	}
// }

// Updates the relations for the given Node, but only the relations specified in the given RelationsToUpdate.
// Will also update the neighbours, including their children (against the node), to point to this node.
// void FNode::UpdateRelations(const FNavMeshPtr& NavMeshPtr, const FChunk& Chunk, const NodeMortonType MortonCode, const LayerIdxType LayerIdx, const DirectionType RelationsToUpdate)
// {
// 	TRACE_CPUPROFILER_EVENT_SCOPE_STR("UpdateRelations");
// 	
// 	// Iterate over each direction, from -X to +Z.
//     for (const DirectionType Direction : FNavMeshStatic::Directions)
// 	{
// 		// Return if the current direction does not need to be updated.
// 		if (!RelationsToUpdate & Direction) continue;
//
//     	// Get the chunk the neighbour is in, which is a different chunk if the current direction goes outside of the chunk.
//     	const FChunk* NeighbourChunk = &Chunk;
//     	if(ChunkBorder & Direction)
//     	{
//     		const auto Iterator = NavMeshPtr->find(Chunk.GetNeighbour(Direction));
//     		if (Iterator == NavMeshPtr->end())
//     		{
//     			Relations.SetFromDirection(Layer_Idx_Invalid, Direction);
//     			// todo: set children of this node to Layer_Idx_Invalid.
//     			continue; // Chunk does not exist.
//     		}
//     		NeighbourChunk = &Iterator->second;
//     	}
// 	
//     	// Get the morton-code of the neighbour in this direction, in the same layer.
//     	NodeMortonType NeighbourMortonCode = FNodeMortonUtils::Move(MortonCode, LayerIdx, Direction);
//     	FNodePair* NeighbourNodePair;
//     	LayerIdxType NeighbourLayerIdx = LayerIdx;
//
//     	// Will find the neighbour by checking each layer one by one upwards, starting from the same layer as this node. Will guarantee a neighbour being found.
// 		while (true)
// 		{
// 			const auto NeighbourIterator = NeighbourChunk->Octrees[0]->Layers[NeighbourLayerIdx]->find(NeighbourMortonCode);
// 			if(NeighbourIterator != NeighbourChunk->Octrees[0]->Layers[NeighbourLayerIdx]->end())
// 			{
// 				NeighbourNodePair = &*NeighbourIterator;
// 				break;
// 			}
//
// 			// Neighbour does not exist, so get the parent and try again.
// 			NeighbourMortonCode = FNodeMortonUtils::GetParent(NeighbourMortonCode, NeighbourLayerIdx);
// 			--NeighbourLayerIdx;
// 		}
//
//     	FNode& Neighbour = NeighbourNodePair->second;
//     	
// 		// Set the NeighbourLayerIdx on the relation for this direction.
// 		// We set the same layer-idx on the neighbour-node because a relation cannot be in a deeper layer, only the same or above as a node.
// 		switch (Direction)
// 		{
// 			case Direction::X_Negative:
// 				Relations.X_Negative_LayerIdx = NeighbourLayerIdx;
// 				Neighbour.Relations.X_Positive_LayerIdx = NeighbourLayerIdx;
// 				//UpdateChildRelations(NeighbourChunk, *NeighbourNodePair, NeighbourLayerIdx, LayerIdx, Direction::X_Positive);
// 				break;
// 			case Direction::Y_Negative:
// 				Relations.Y_Negative_LayerIdx = NeighbourLayerIdx;
// 				Neighbour.Relations.Y_Positive_LayerIdx = NeighbourLayerIdx;
// 				//UpdateChildRelations(NeighbourChunk, *NeighbourNodePair, NeighbourLayerIdx, LayerIdx, Direction::Y_Positive);
// 				break;
// 			case Direction::Z_Negative:
// 				Relations.Z_Negative_LayerIdx = NeighbourLayerIdx;
// 				Neighbour.Relations.Z_Positive_LayerIdx = NeighbourLayerIdx;
// 				//UpdateChildRelations(NeighbourChunk, *NeighbourNodePair, NeighbourLayerIdx, LayerIdx, Direction::Z_Positive);
// 				break;
// 			case Direction::X_Positive:
// 				Relations.X_Positive_LayerIdx = NeighbourLayerIdx;
// 				Neighbour.Relations.X_Negative_LayerIdx = NeighbourLayerIdx;
// 				//UpdateChildRelations(NeighbourChunk, *NeighbourNodePair, NeighbourLayerIdx, LayerIdx, Direction::X_Negative);
// 				break;
// 			case Direction::Y_Positive:
// 				Relations.Y_Positive_LayerIdx = NeighbourLayerIdx;
// 				Neighbour.Relations.Y_Negative_LayerIdx = NeighbourLayerIdx;
// 				//UpdateChildRelations(NeighbourChunk, *NeighbourNodePair, NeighbourLayerIdx, LayerIdx, Direction::Y_Negative);
// 				break;
// 			case Direction::Z_Positive:
// 				Relations.Z_Positive_LayerIdx = NeighbourLayerIdx;
// 				Neighbour.Relations.Z_Negative_LayerIdx = NeighbourLayerIdx;
// 				//UpdateChildRelations(NeighbourChunk, *NeighbourNodePair, NeighbourLayerIdx, LayerIdx, Direction::Z_Negative);
// 				break;
// 			default: break;
// 		}
// 		
// 		// Relation is now updated.
// 	}
// }

bool FNode::HasOverlap(const UWorld* World, const FGlobalVector& ChunkLocation, const NodeMortonType MortonCode, const LayerIdxType LayerIdx) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("Node Has-World-Overlap");
	return FPhysicsInterface::GeomOverlapBlockingTest(
		World,
		FNavMeshStatic::CollisionBoxes[LayerIdx],
		GetGlobalLocation(ChunkLocation, MortonCode).ToVector() + FNavMeshStatic::NodeHalveSizes[LayerIdx],
		FQuat::Identity,
		ECollisionChannel::ECC_WorldStatic,
		FCollisionQueryParams::DefaultQueryParam,
		FCollisionResponseParams::DefaultResponseParam
	);
}

// bool FNode::HasGeomOverlap(const FBodyInstance* BodyInstance, const FGlobalVector& CenterLocation, const LayerIdxType LayerIdx)
// {
// 	TRACE_CPUPROFILER_EVENT_SCOPE_STR("Node Has-Geom-Overlap");
// 	return true;
// 	//return FPhysInterface_Chaos::Overlap_Geom(BodyInstance, FNavMeshStatic::CollisionBoxes[LayerIdx], FQuat::Identity, FTransform(FQuat::Identity, CenterLocation.ToVector()));
// }

void FNode::Draw(const UWorld* World, const FGlobalVector& ChunkLocation, const NodeMortonType MortonCode, const LayerIdxType LayerIndex, const FColor Color, const uint32 Thickness) const
{
	const float NodeHalveSize = FNavMeshStatic::NodeHalveSizes[LayerIndex];
	const FVector GlobalCenter = GetGlobalLocation(ChunkLocation, MortonCode).ToVector() + NodeHalveSize;
	const FVector Extent(NodeHalveSize);
	DrawDebugBox(World, GlobalCenter, Extent, Color, true, -1, 0, Thickness);
}