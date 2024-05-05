// Copyright Melvin Brink 2023. All Rights Reserved.

#include "MBNavigation/Types/NavMesh.h"



std::array<uint8, 6> FOctreeNode::GetNeighbourLayerIndexes() const
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

std::array<FNodeLookupData, 6> FOctreeNode::GetNeighboursLookupData(const F3DVector32& ChunkLocation) const
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
        F3DVector32 ChunkOffset(0, 0, 0);
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
        const uint_fast32_t ParentMortonCode = GetMortonCode() & ~((1 << ParentShiftAmount[RelationLayerIndex]) - 1);
        const F3DVector10 ParentLocalLocation = F3DVector10::FromMortonCode(ParentMortonCode);
        F3DVector10 NeighbourLocalLocation;
        switch (Direction) {
            case DIRECTION_X_NEGATIVE: NeighbourLocalLocation = ParentLocalLocation - F3DVector10(MortonOffset, 0, 0); break;
            case DIRECTION_Y_NEGATIVE: NeighbourLocalLocation = ParentLocalLocation - F3DVector10(0, MortonOffset, 0); break;
            case DIRECTION_Z_NEGATIVE: NeighbourLocalLocation = ParentLocalLocation - F3DVector10(0, 0, MortonOffset); break;
            case DIRECTION_X_POSITIVE: NeighbourLocalLocation = ParentLocalLocation + F3DVector10(MortonOffset, 0, 0); break;
            case DIRECTION_Y_POSITIVE: NeighbourLocalLocation = ParentLocalLocation + F3DVector10(0, MortonOffset, 0); break;
            case DIRECTION_Z_POSITIVE: NeighbourLocalLocation = ParentLocalLocation + F3DVector10(0, 0, MortonOffset); break;
            default: break;
        }

        NeighboursLookupData[Index].ChunkKey = (ChunkLocation + ChunkOffset).ToKey();
        NeighboursLookupData[Index].LayerIndex = RelationLayerIndex;
        NeighboursLookupData[Index].MortonCode = NeighbourLocalLocation.ToMortonCode();
    }

    return NeighboursLookupData;
}