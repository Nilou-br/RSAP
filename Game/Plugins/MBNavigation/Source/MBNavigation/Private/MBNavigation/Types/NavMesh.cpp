// Copyright Melvin Brink 2023. All Rights Reserved.

#include "MBNavigation/Types/NavMesh.h"
#include "MBNavigation/NavMesh/Shared.h"
// #include "Physics/Experimental/PhysInterface_Chaos.h"


FNode::FNode(const uint8 ChildIdx, const NavmeshDirection ParentChunkBorder)
{
	if (ParentChunkBorder)
	{
		ChunkBorder |= ChildIdx & 1 ? DIRECTION_X_POSITIVE : DIRECTION_X_NEGATIVE;
		ChunkBorder |= ChildIdx & 2 ? DIRECTION_Y_POSITIVE : DIRECTION_Y_NEGATIVE;
		ChunkBorder |= ChildIdx & 4 ? DIRECTION_Z_POSITIVE : DIRECTION_Z_NEGATIVE;
		ChunkBorder &= ParentChunkBorder; // Can only be against the same border(s) as the parent.
	}
}

std::array<uint8, 6> FNode::GetNeighbourLayerIndexes() const
{
    std::array<uint8, 6> NeighbourLayerIndexes;

    LayerIdxType LayerIdx = 0;
    for (NavmeshDirection Direction = 0b100000; Direction >= 0b000001; Direction>>=1, ++LayerIdx)
    {
        switch (Direction) {
            case DIRECTION_X_NEGATIVE: NeighbourLayerIndexes[LayerIdx] = Relations.X_Negative_Layer; break;
            case DIRECTION_Y_NEGATIVE: NeighbourLayerIndexes[LayerIdx] = Relations.X_Positive_Layer; break;
            case DIRECTION_Z_NEGATIVE: NeighbourLayerIndexes[LayerIdx] = Relations.Y_Negative_Layer; break;
            case DIRECTION_X_POSITIVE: NeighbourLayerIndexes[LayerIdx] = Relations.Y_Positive_Layer; break;
            case DIRECTION_Y_POSITIVE: NeighbourLayerIndexes[LayerIdx] = Relations.Z_Negative_Layer; break;
            case DIRECTION_Z_POSITIVE: NeighbourLayerIndexes[LayerIdx] = Relations.Z_Positive_Layer; break;
            default:break;
        }
    }

    return NeighbourLayerIndexes;
}

std::array<FNodeLookupData, 6> FNode::GetNeighboursLookupData(const FGlobalVector& ChunkLocation, const MortonCodeType MortonCode) const
{
    std::array<FNodeLookupData, 6> NeighboursLookupData;
    uint8 DirectionIdx = 0;
	
    for (const NavmeshDirection Direction : FNavMeshStatic::Directions)
    {
        const LayerIdxType RelationLayerIdx = Relations.GetFromDirection(Direction);
        if (RelationLayerIdx == LAYER_INDEX_INVALID) {
            NeighboursLookupData[DirectionIdx] = FNodeLookupData();
            continue;
        }

        // Calculate ChunkOffset if the direction goes into a different chunk.
        FGlobalVector ChunkOffset(0, 0, 0);
        if (ChunkBorder & Direction) {
            switch (Direction) {
                case DIRECTION_X_NEGATIVE: ChunkOffset.X = -FNavMeshStatic::ChunkSize; break;
                case DIRECTION_Y_NEGATIVE: ChunkOffset.Y = -FNavMeshStatic::ChunkSize; break;
                case DIRECTION_Z_NEGATIVE: ChunkOffset.Z = -FNavMeshStatic::ChunkSize; break;
                case DIRECTION_X_POSITIVE: ChunkOffset.X =  FNavMeshStatic::ChunkSize; break;
                case DIRECTION_Y_POSITIVE: ChunkOffset.Y =  FNavMeshStatic::ChunkSize; break;
                case DIRECTION_Z_POSITIVE: ChunkOffset.Z =  FNavMeshStatic::ChunkSize; break;
                default: break;
            }
        }

        NeighboursLookupData[DirectionIdx].ChunkKey = (ChunkLocation + ChunkOffset).ToKey();
        NeighboursLookupData[DirectionIdx].LayerIndex = RelationLayerIdx;
        NeighboursLookupData[DirectionIdx].MortonCode = FMortonUtils::MoveAndMask(MortonCode, RelationLayerIdx, Direction);
    	++DirectionIdx;
    }

    return NeighboursLookupData;
}

// For the given node, set its children's relation in the Direction to the given LayerIdxToSet. Only the children against the same border in this direction will be updated.
static void UpdateChildRelations(const FChunk* Chunk, const FNodePair& NodePair, const LayerIdxType LayerIdx, const LayerIdxType LayerIdxToSet, const NavmeshDirection Direction)
{
	if(!NodePair.second.HasChildren()) return;
	
	const FMortonVector ParentLocation = FMortonVector::FromMortonCode(NodePair.first);
	const LayerIdxType ChildLayerIdx = LayerIdx+1;
	const uint16 MortonOffset = FNavMeshStatic::MortonOffsets[ChildLayerIdx];

	// Get the morton-codes of the children facing the direction. ( Children against the border of their parent in this direction. )
	std::array<MortonCodeType, 4> ChildMortonCodes;
	switch (Direction)
	{
		case DIRECTION_X_NEGATIVE:
			ChildMortonCodes[0] = (ParentLocation+FMortonVector()).ToMortonCode();																// 1st child
			ChildMortonCodes[1] = (ParentLocation+FMortonVector(0,					MortonOffset,	0)).ToMortonCode();				// 3rd child
			ChildMortonCodes[2] = (ParentLocation+FMortonVector(0,					0,				MortonOffset)).ToMortonCode();	// 5th child
			ChildMortonCodes[3] = (ParentLocation+FMortonVector(0,					MortonOffset,	MortonOffset)).ToMortonCode();	// 7th child
			break;
		case DIRECTION_Y_NEGATIVE:
			ChildMortonCodes[0] = (ParentLocation+FMortonVector()).ToMortonCode();																// 1st child
			ChildMortonCodes[1] = (ParentLocation+FMortonVector(MortonOffset,		0,	0)).ToMortonCode();							// 2nd child
			ChildMortonCodes[2] = (ParentLocation+FMortonVector(0,					0,	MortonOffset)).ToMortonCode();				// 5th child
			ChildMortonCodes[3] = (ParentLocation+FMortonVector(MortonOffset,		0,	MortonOffset)).ToMortonCode();				// 6th child
			break;
		case DIRECTION_Z_NEGATIVE:
			ChildMortonCodes[0] = (ParentLocation+FMortonVector()).ToMortonCode();																// 1st child
			ChildMortonCodes[1] = (ParentLocation+FMortonVector(MortonOffset,		0,				0)).ToMortonCode();				// 2nd child
			ChildMortonCodes[2] = (ParentLocation+FMortonVector(0,					MortonOffset,	0)).ToMortonCode();				// 3rd child
			ChildMortonCodes[3] = (ParentLocation+FMortonVector(MortonOffset,		MortonOffset,	0)).ToMortonCode();				// 4th child
			break;
		case DIRECTION_X_POSITIVE:
			ChildMortonCodes[0] = (ParentLocation+FMortonVector(MortonOffset,		0,				0)).ToMortonCode();				// 2nd child
			ChildMortonCodes[1] = (ParentLocation+FMortonVector(MortonOffset,		MortonOffset,	0)).ToMortonCode();				// 4th child
			ChildMortonCodes[2] = (ParentLocation+FMortonVector(MortonOffset,		0,				MortonOffset)).ToMortonCode();	// 6th child
			ChildMortonCodes[3] = (ParentLocation+FMortonVector(MortonOffset,		MortonOffset,	MortonOffset)).ToMortonCode();	// 8th child
			break;
		case DIRECTION_Y_POSITIVE:
			ChildMortonCodes[0] = (ParentLocation+FMortonVector(0,					MortonOffset,	0)).ToMortonCode();				// 3rd child
			ChildMortonCodes[1] = (ParentLocation+FMortonVector(MortonOffset,		MortonOffset,	0)).ToMortonCode();				// 4th child
			ChildMortonCodes[2] = (ParentLocation+FMortonVector(0,					MortonOffset,	MortonOffset)).ToMortonCode();	// 7th child
			ChildMortonCodes[3] = (ParentLocation+FMortonVector(MortonOffset,		MortonOffset,	MortonOffset)).ToMortonCode();	// 8th child
			break;
		case DIRECTION_Z_POSITIVE:
			ChildMortonCodes[0] = (ParentLocation+FMortonVector(0,					0,				MortonOffset)).ToMortonCode();	// 5th child
			ChildMortonCodes[1] = (ParentLocation+FMortonVector(MortonOffset,		0,				MortonOffset)).ToMortonCode();	// 6th child
			ChildMortonCodes[2] = (ParentLocation+FMortonVector(0,					MortonOffset,	MortonOffset)).ToMortonCode();	// 7th child
			ChildMortonCodes[3] = (ParentLocation+FMortonVector(MortonOffset,		MortonOffset,	MortonOffset)).ToMortonCode();	// 8th child
			break;
		default: break;
	}

	// Update each child's relation in this direction to the LayerIdxToSet. Recursively do the same for their children in this direction.
	for (auto ChildMortonCode : ChildMortonCodes)
	{
		const auto ChildNodeIterator = Chunk->Octrees[0]->Layers[ChildLayerIdx]->find(ChildMortonCode);
		ChildNodeIterator->second.Relations.SetFromDirection(LayerIdxToSet, Direction);
		UpdateChildRelations(Chunk, *ChildNodeIterator, ChildLayerIdx, LayerIdxToSet, Direction);
	}
}

// Updates the relations for the given Node, but only the relations specified in the given RelationsToUpdate.
// Will also update the neighbours, including their children (against the node), to point to this node.
void FNode::UpdateRelations(const FNavMeshPtr& NavMeshPtr, const FChunk& Chunk, const MortonCodeType MortonCode, const LayerIdxType LayerIdx, NavmeshDirection RelationsToUpdate)
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("UpdateRelations");

	FMortonVector NodeLocation = FMortonVector::FromMortonCode(MortonCode);
	
	// Iterate over each direction, from -X to +Z.
    for (const NavmeshDirection Direction : FNavMeshStatic::Directions)
	{
		// Return if the current direction does not need to be updated.
		const NavmeshDirection DirectionToUpdate = RelationsToUpdate & Direction;
		if (!DirectionToUpdate) continue;

		// Get the chunk the neighbour is in, which is a different chunk if the current direction goes outside of the chunk.
    	const FChunk* NeighbourChunk = &Chunk;
		if(ChunkBorder & DirectionToUpdate)
		{
			const auto Iterator = NavMeshPtr->find(Chunk.GetNeighbour(Direction));
			if (Iterator == NavMeshPtr->end())
			{
				RelationsToUpdate &= ~DirectionToUpdate; // Sets only the bit for the current direction-to-update to 0. // todo: check if this is needed??
				continue; // todo: set relation of node to invalid??
			}
			NeighbourChunk = &Iterator->second;
		}

		// Get the morton-code of the neighbour in this direction, in the same layer.
		MortonCodeType NeighbourMortonCode = FMortonUtils::Move(MortonCode, LayerIdx, Direction);
    	FMortonVector NeighbourLocation = FMortonVector::FromMortonCode(NeighbourMortonCode);
		
		// Find the neighbour by checking each layer one by one upwards in the octree, starting from this node's layer.
		for (int NeighbourLayerIdx = LayerIdx; NeighbourLayerIdx >= 0; --NeighbourLayerIdx) // todo
		{
			const auto NeighbourIterator = NeighbourChunk->Octrees[0]->Layers[NeighbourLayerIdx]->find(NeighbourMortonCode);
			if(NeighbourIterator == NeighbourChunk->Octrees[0]->Layers[NeighbourLayerIdx]->end())
			{
				// There is no neighbour on this layer, so try again using the parent of this uninitialized neighbour.
				NeighbourMortonCode = FMortonUtils::GetParent(NeighbourMortonCode, NeighbourLayerIdx);
				continue;
			}
			
			FNode& NeighbourNode = NeighbourIterator->second;
			
			// Set the FoundNeighbourLayerIndex on the Node.Neighbours for this direction,
			// and the Node's layer-index to the NeighbourNode's relation for opposite direction ( where we are looking from ).
			switch (Direction)
			{
				case DIRECTION_X_NEGATIVE:
					Relations.X_Negative_Layer = NeighbourLayerIdx;
					NeighbourNode.Relations.X_Positive_Layer = NeighbourLayerIdx;
					//UpdateChildRelations(NeighbourChunk, *NeighbourIterator, NeighbourLayerIdx, LayerIdx, DIRECTION_X_POSITIVE);
					break;
				case DIRECTION_Y_NEGATIVE:
					Relations.Y_Negative_Layer = NeighbourLayerIdx;
					NeighbourNode.Relations.Y_Positive_Layer = NeighbourLayerIdx;
					//UpdateChildRelations(NeighbourChunk, *NeighbourIterator, NeighbourLayerIdx, LayerIdx, DIRECTION_Y_POSITIVE);
					break;
				case DIRECTION_Z_NEGATIVE:
					Relations.Z_Negative_Layer = NeighbourLayerIdx;
					NeighbourNode.Relations.Z_Positive_Layer = NeighbourLayerIdx;
					//UpdateChildRelations(NeighbourChunk, *NeighbourIterator, NeighbourLayerIdx, LayerIdx, DIRECTION_Z_POSITIVE);
					break;
				case DIRECTION_X_POSITIVE:
					Relations.X_Positive_Layer = NeighbourLayerIdx;
					NeighbourNode.Relations.X_Negative_Layer = NeighbourLayerIdx;
					//UpdateChildRelations(NeighbourChunk, *NeighbourIterator, NeighbourLayerIdx, LayerIdx, DIRECTION_X_NEGATIVE);
					break;
				case DIRECTION_Y_POSITIVE:
					Relations.Y_Positive_Layer = NeighbourLayerIdx;
					NeighbourNode.Relations.Y_Negative_Layer = NeighbourLayerIdx;
					//UpdateChildRelations(NeighbourChunk, *NeighbourIterator, NeighbourLayerIdx, LayerIdx, DIRECTION_Y_NEGATIVE);
					break;
				case DIRECTION_Z_POSITIVE:
					Relations.Z_Positive_Layer = NeighbourLayerIdx;
					NeighbourNode.Relations.Z_Negative_Layer = NeighbourLayerIdx;
					//UpdateChildRelations(NeighbourChunk, *NeighbourIterator, NeighbourLayerIdx, LayerIdx, DIRECTION_Z_NEGATIVE);
					break;
				default: break;
			}
			
			// Relation is now updated.
			break;
		}
	}
}

bool FNode::HasOverlap(const UWorld* World, const FGlobalVector& ChunkLocation, const MortonCodeType MortonCode, const LayerIdxType LayerIdx) const
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

void FNode::Draw(const UWorld* World, const FGlobalVector& ChunkLocation, const MortonCodeType MortonCode, const LayerIdxType LayerIndex, const FColor Color, const uint32 Thickness) const
{
	const float NodeHalveSize = FNavMeshStatic::NodeHalveSizes[LayerIndex];
	const FVector GlobalCenter = GetGlobalLocation(ChunkLocation, MortonCode).ToVector() + NodeHalveSize;
	const FVector Extent(NodeHalveSize);
	DrawDebugBox(World, GlobalCenter, Extent, Color, true, -1, 0, Thickness);
}

ChunkKeyType FChunk::GetNeighbour(const NavmeshDirection Direction) const
{
	FGlobalVector NeighbourLocation = Location;
	switch (Direction) {
		case DIRECTION_X_NEGATIVE: NeighbourLocation.X -= FNavMeshStatic::ChunkSize; break;
		case DIRECTION_Y_NEGATIVE: NeighbourLocation.Y -= FNavMeshStatic::ChunkSize; break;
		case DIRECTION_Z_NEGATIVE: NeighbourLocation.Z -= FNavMeshStatic::ChunkSize; break;
		case DIRECTION_X_POSITIVE: NeighbourLocation.X += FNavMeshStatic::ChunkSize; break;
		case DIRECTION_Y_POSITIVE: NeighbourLocation.Y += FNavMeshStatic::ChunkSize; break;
		case DIRECTION_Z_POSITIVE: NeighbourLocation.Z += FNavMeshStatic::ChunkSize; break;
		default: break;
	}
	return NeighbourLocation.ToKey();
}
