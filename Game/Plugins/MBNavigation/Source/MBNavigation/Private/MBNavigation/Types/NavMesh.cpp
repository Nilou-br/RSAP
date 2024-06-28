// Copyright Melvin Brink 2023. All Rights Reserved.

#include "MBNavigation/Types/NavMesh.h"
#include "MBNavigation/NavMesh/Shared.h"
// #include "Physics/Experimental/PhysInterface_Chaos.h"


std::array<uint8, 6> FNode::GetNeighbourLayerIndexes() const
{
    std::array<uint8, 6> NeighbourLayerIndexes;

    int Index = 0;
    for (int Direction = 0b100000; Direction >= 0b000001; Direction>>=1, ++Index)
    {
        switch (Direction) {
            case DIRECTION_X_NEGATIVE: NeighbourLayerIndexes[Index] = Relations.X_Negative; break;
            case DIRECTION_Y_NEGATIVE: NeighbourLayerIndexes[Index] = Relations.X_Positive; break;
            case DIRECTION_Z_NEGATIVE: NeighbourLayerIndexes[Index] = Relations.Y_Negative; break;
            case DIRECTION_X_POSITIVE: NeighbourLayerIndexes[Index] = Relations.Y_Positive; break;
            case DIRECTION_Y_POSITIVE: NeighbourLayerIndexes[Index] = Relations.Z_Negative; break;
            case DIRECTION_Z_POSITIVE: NeighbourLayerIndexes[Index] = Relations.Z_Positive; break;
            default:break;
        }
    }

    return NeighbourLayerIndexes;
}

std::array<FNodeLookupData, 6> FNode::GetNeighboursLookupData(const FGlobalVector& ChunkLocation, const MortonCodeType MortonCode) const
{
    std::array<FNodeLookupData, 6> NeighboursLookupData;
    
    int Index = 0;
    for (uint8 Direction = 0b100000; Direction >= 0b000001; Direction >>= 1, ++Index) {

        const uint8 RelationLayerIndex = Relations.GetFromDirection(Direction);
        if (RelationLayerIndex == LAYER_INDEX_INVALID) {
            NeighboursLookupData[Index] = FNodeLookupData();
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
        
        // Calculate the local location of the neighbour.
        const uint_fast16_t MortonOffset = FNavMeshStatic::MortonOffsets[RelationLayerIndex];
        const MortonCodeType ParentMortonCode = MortonCode & ~((1 << ParentShiftAmount[RelationLayerIndex]) - 1);
        const FMortonVector ParentLocalLocation = FMortonVector::FromMortonCode(ParentMortonCode);
        FMortonVector NeighbourLocalLocation;
        switch (Direction) {
            case DIRECTION_X_NEGATIVE: NeighbourLocalLocation = ParentLocalLocation - FMortonVector(MortonOffset, 0, 0); break;
            case DIRECTION_Y_NEGATIVE: NeighbourLocalLocation = ParentLocalLocation - FMortonVector(0, MortonOffset, 0); break;
            case DIRECTION_Z_NEGATIVE: NeighbourLocalLocation = ParentLocalLocation - FMortonVector(0, 0, MortonOffset); break;
            case DIRECTION_X_POSITIVE: NeighbourLocalLocation = ParentLocalLocation + FMortonVector(MortonOffset, 0, 0); break;
            case DIRECTION_Y_POSITIVE: NeighbourLocalLocation = ParentLocalLocation + FMortonVector(0, MortonOffset, 0); break;
            case DIRECTION_Z_POSITIVE: NeighbourLocalLocation = ParentLocalLocation + FMortonVector(0, 0, MortonOffset); break;
            default: break;
        }

        NeighboursLookupData[Index].ChunkKey = (ChunkLocation + ChunkOffset).ToKey();
        NeighboursLookupData[Index].LayerIndex = RelationLayerIndex;
        NeighboursLookupData[Index].MortonCode = NeighbourLocalLocation.ToMortonCode();
    }

    return NeighboursLookupData;
}

// For the given node, set its children's relation in the Direction to the given LayerIdxToSet. Only the children against the same border in this direction will be updated.
static void UpdateChildRelations(const FChunk* Chunk, const FNodePair& NodePair, const LayerIdxType LayerIdx, const LayerIdxType LayerIdxToSet, const NavmeshDirection Direction)
{
	if(!NodePair.second.HasChildren()) return;
	
	const FMortonVector ParentMortonLocation = FMortonVector::FromMortonCode(NodePair.first);
	const LayerIdxType ChildLayerIndex = LayerIdx+1;
	const uint16 MortonOffset = FNavMeshStatic::MortonOffsets[ChildLayerIndex];

	// Get the morton-codes of the children facing the direction. ( Children against the border of their parent in this direction. )
	std::array<MortonCodeType, 4> ChildMortonCodes;
	switch (Direction)
	{
		case DIRECTION_X_NEGATIVE:
			ChildMortonCodes[0] = (ParentMortonLocation+FMortonVector()).ToMortonCode();															// 1st child
			ChildMortonCodes[1] = (ParentMortonLocation+FMortonVector(0,					MortonOffset,	0)).ToMortonCode();				// 3rd child
			ChildMortonCodes[2] = (ParentMortonLocation+FMortonVector(0,					0,				MortonOffset)).ToMortonCode();	// 5th child
			ChildMortonCodes[3] = (ParentMortonLocation+FMortonVector(0,					MortonOffset,	MortonOffset)).ToMortonCode();	// 7th child
			break;
		case DIRECTION_Y_NEGATIVE:
			ChildMortonCodes[0] = (ParentMortonLocation+FMortonVector()).ToMortonCode();															// 1st child
			ChildMortonCodes[1] = (ParentMortonLocation+FMortonVector(MortonOffset,		0,	0)).ToMortonCode();							// 2nd child
			ChildMortonCodes[2] = (ParentMortonLocation+FMortonVector(0,					0,	MortonOffset)).ToMortonCode();				// 5th child
			ChildMortonCodes[3] = (ParentMortonLocation+FMortonVector(MortonOffset,		0,	MortonOffset)).ToMortonCode();				// 6th child
			break;
		case DIRECTION_Z_NEGATIVE:
			ChildMortonCodes[0] = (ParentMortonLocation+FMortonVector()).ToMortonCode();															// 1st child
			ChildMortonCodes[1] = (ParentMortonLocation+FMortonVector(MortonOffset,		0,				0)).ToMortonCode();				// 2nd child
			ChildMortonCodes[2] = (ParentMortonLocation+FMortonVector(0,					MortonOffset,	0)).ToMortonCode();				// 3rd child
			ChildMortonCodes[3] = (ParentMortonLocation+FMortonVector(MortonOffset,		MortonOffset,	0)).ToMortonCode();				// 4th child
			break;
		case DIRECTION_X_POSITIVE:
			ChildMortonCodes[0] = (ParentMortonLocation+FMortonVector(MortonOffset,		0,				0)).ToMortonCode();				// 2nd child
			ChildMortonCodes[1] = (ParentMortonLocation+FMortonVector(MortonOffset,		MortonOffset,	0)).ToMortonCode();				// 4th child
			ChildMortonCodes[2] = (ParentMortonLocation+FMortonVector(MortonOffset,		0,				MortonOffset)).ToMortonCode();	// 6th child
			ChildMortonCodes[3] = (ParentMortonLocation+FMortonVector(MortonOffset,		MortonOffset,	MortonOffset)).ToMortonCode();	// 8th child
			break;
		case DIRECTION_Y_POSITIVE:
			ChildMortonCodes[0] = (ParentMortonLocation+FMortonVector(0,					MortonOffset,	0)).ToMortonCode();				// 3rd child
			ChildMortonCodes[1] = (ParentMortonLocation+FMortonVector(MortonOffset,		MortonOffset,	0)).ToMortonCode();				// 4th child
			ChildMortonCodes[2] = (ParentMortonLocation+FMortonVector(0,					MortonOffset,	MortonOffset)).ToMortonCode();	// 7th child
			ChildMortonCodes[3] = (ParentMortonLocation+FMortonVector(MortonOffset,		MortonOffset,	MortonOffset)).ToMortonCode();	// 8th child
			break;
		case DIRECTION_Z_POSITIVE:
			ChildMortonCodes[0] = (ParentMortonLocation+FMortonVector(0,					0,				MortonOffset)).ToMortonCode();	// 5th child
			ChildMortonCodes[1] = (ParentMortonLocation+FMortonVector(MortonOffset,		0,				MortonOffset)).ToMortonCode();	// 6th child
			ChildMortonCodes[2] = (ParentMortonLocation+FMortonVector(0,					MortonOffset,	MortonOffset)).ToMortonCode();	// 7th child
			ChildMortonCodes[3] = (ParentMortonLocation+FMortonVector(MortonOffset,		MortonOffset,	MortonOffset)).ToMortonCode();	// 8th child
			break;
		default: break;
	}

	// Update each child's relation in this direction to the LayerIdxToSet. Recursively do the same for their children in this direction.
	for (auto ChildMortonCode : ChildMortonCodes)
	{
		const auto ChildNodeIterator = Chunk->Octrees[0]->Layers[ChildLayerIndex]->find(ChildMortonCode);
		ChildNodeIterator->second.Relations.SetFromDirection(LayerIdxToSet, Direction);
		UpdateChildRelations(Chunk, *ChildNodeIterator, ChildLayerIndex, LayerIdxToSet, Direction);
	}
}

// Updates the relations for the given Node, but only the relations specified in the given RelationsToUpdate.
// Will also update the neighbours, including their children (against the node), to point to this node.
void FNode::UpdateRelations(const FNavMeshPtr& NavMeshPtr, const FChunk& Chunk, const MortonCodeType MortonCode, const LayerIdxType LayerIdx, NavmeshDirection RelationsToUpdate)
{
    const FMortonVector NodeLocalLocation = FMortonVector::FromMortonCode(MortonCode);
	
	// Iterate over each direction, from -X to +Z.
	for (NavmeshDirection Direction = 0b100000; Direction != DIRECTION_NONE; Direction >>= 1)
	{
		// Return if the current direction does not need to be updated.
		const NavmeshDirection DirectionToUpdate = RelationsToUpdate & Direction;
		if (!DirectionToUpdate) continue;

		// Get the chunk the neighbour is in, which is a different chunk if the current direction goes outside of the chunk.
		const FChunk* NeighbourChunk = ChunkBorder & DirectionToUpdate ? GetNeighbouringChunk(NavMeshPtr, Chunk.Location, Direction) : &Chunk;
		if(!NeighbourChunk)
		{
			// The neighbouring-chunk does not exist, so we can remove this direction from the RelationsToUpdate to prevent this find from being repeated.
			RelationsToUpdate &= ~DirectionToUpdate; // Sets only the bit for the current direction-to-update to 0.
			continue;
		}

		// Get the morton-code of the neighbour in this direction, in the same layer as the given node.
		MortonCodeType NeighbourMortonCode;
		switch (Direction) {
			case DIRECTION_X_NEGATIVE: NeighbourMortonCode = (NodeLocalLocation - FMortonVector(FNavMeshStatic::MortonOffsets[LayerIdx], 0, 0)).ToMortonCode(); break;
			case DIRECTION_Y_NEGATIVE: NeighbourMortonCode = (NodeLocalLocation - FMortonVector(0, FNavMeshStatic::MortonOffsets[LayerIdx], 0)).ToMortonCode(); break;
			case DIRECTION_Z_NEGATIVE: NeighbourMortonCode = (NodeLocalLocation - FMortonVector(0, 0, FNavMeshStatic::MortonOffsets[LayerIdx])).ToMortonCode(); break;
			case DIRECTION_X_POSITIVE: NeighbourMortonCode = (NodeLocalLocation + FMortonVector(FNavMeshStatic::MortonOffsets[LayerIdx], 0, 0)).ToMortonCode(); break;
			case DIRECTION_Y_POSITIVE: NeighbourMortonCode = (NodeLocalLocation + FMortonVector(0, FNavMeshStatic::MortonOffsets[LayerIdx], 0)).ToMortonCode(); break;
			case DIRECTION_Z_POSITIVE: NeighbourMortonCode = (NodeLocalLocation + FMortonVector(0, 0, FNavMeshStatic::MortonOffsets[LayerIdx])).ToMortonCode(); break;
			default: break;
		}
		
		// Find the neighbour by checking each layer one by one upwards in the octree, starting from the given node's layer-idx, until we find the neighbour.
		for (LayerIdxType NeighbourLayerIdx = LayerIdx; NeighbourLayerIdx < LAYER_INDEX_INVALID; --NeighbourLayerIdx)
		{
			const auto NodeIterator = NeighbourChunk->Octrees[0]->Layers[NeighbourLayerIdx]->find(NeighbourMortonCode);
			if(NodeIterator == NeighbourChunk->Octrees[0]->Layers[NeighbourLayerIdx]->end())
			{
				// There is no neighbour on this layer, so try again using the parent of this uninitialized neighbour.
				NeighbourMortonCode = GetParentMortonCode(NeighbourMortonCode, NeighbourLayerIdx);
				continue;
			}
			
			FNode& NeighbourNode = NodeIterator->second;
			
			// Set the FoundNeighbourLayerIndex on the Node.Neighbours for this direction,
			// and the Node's layer-index to the NeighbourNode's relation for opposite direction ( where we are looking from ).
			switch (Direction)
			{
				case DIRECTION_X_NEGATIVE:
					Relations.X_Negative = NeighbourLayerIdx;
					NeighbourNode.Relations.X_Positive = NeighbourLayerIdx;
					UpdateChildRelations(NeighbourChunk, *NodeIterator, NeighbourLayerIdx, LayerIdx, DIRECTION_X_POSITIVE);
					break;
				case DIRECTION_Y_NEGATIVE:
					Relations.Y_Negative = NeighbourLayerIdx;
					NeighbourNode.Relations.Y_Positive = NeighbourLayerIdx;
					UpdateChildRelations(NeighbourChunk, *NodeIterator, NeighbourLayerIdx, LayerIdx, DIRECTION_Y_POSITIVE);
					break;
				case DIRECTION_Z_NEGATIVE:
					Relations.Z_Negative = NeighbourLayerIdx;
					NeighbourNode.Relations.Z_Positive = NeighbourLayerIdx;
					UpdateChildRelations(NeighbourChunk, *NodeIterator, NeighbourLayerIdx, LayerIdx, DIRECTION_Z_POSITIVE);
					break;
				case DIRECTION_X_POSITIVE:
					Relations.X_Positive = NeighbourLayerIdx;
					NeighbourNode.Relations.X_Negative = NeighbourLayerIdx;
					UpdateChildRelations(NeighbourChunk, *NodeIterator, NeighbourLayerIdx, LayerIdx, DIRECTION_X_NEGATIVE);
					break;
				case DIRECTION_Y_POSITIVE:
					Relations.Y_Positive = NeighbourLayerIdx;
					NeighbourNode.Relations.Y_Negative = NeighbourLayerIdx;
					UpdateChildRelations(NeighbourChunk, *NodeIterator, NeighbourLayerIdx, LayerIdx, DIRECTION_Y_NEGATIVE);
					break;
				case DIRECTION_Z_POSITIVE:
					Relations.Z_Positive = NeighbourLayerIdx;
					NeighbourNode.Relations.Z_Negative = NeighbourLayerIdx;
					UpdateChildRelations(NeighbourChunk, *NodeIterator, NeighbourLayerIdx, LayerIdx, DIRECTION_Z_NEGATIVE);
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
