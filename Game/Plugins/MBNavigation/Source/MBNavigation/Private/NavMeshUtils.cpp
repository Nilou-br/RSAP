#include "NavMeshUtils.h"
#include "NavMeshTypes.h"



std::array<FNodeLookupData, 6> GetNeighboursLookupData(const FOctreeNode& Node, const F3DVector32& ChunkLocation) {
    std::array<FNodeLookupData, 6> NeighboursLookupData;
    
    int Index = 0;
    for (uint8 Direction = 0b100000; Direction >= 0b000001; Direction >>= 1, ++Index) {

        const uint8 NeighbourLayerIndex = Node.Neighbours.GetFromDirection(Direction);
        if (NeighbourLayerIndex == LAYER_INDEX_INVALID) {
            NeighboursLookupData[Index] = FNodeLookupData();
            continue;
        }

        F3DVector32 ChunkOffset(0, 0, 0);
        const bool bIsDifferentChunk = Node.ChunkBorder & Direction;

        // Calculate ChunkOffset based on direction and if it's a different chunk
        if (bIsDifferentChunk) {
            switch (Direction) {
                case DIRECTION_X_NEGATIVE:
                    ChunkOffset.X = -FNavMeshData::ChunkSize;
                    break;
                case DIRECTION_Y_NEGATIVE:
                    ChunkOffset.Y = -FNavMeshData::ChunkSize;
                    break;
                case DIRECTION_Z_NEGATIVE:
                    ChunkOffset.Z = -FNavMeshData::ChunkSize;
                    break;
                case DIRECTION_X_POSITIVE:
                    ChunkOffset.X = FNavMeshData::ChunkSize;
                    break;
                case DIRECTION_Y_POSITIVE:
                    ChunkOffset.Y = FNavMeshData::ChunkSize;
                    break;
                case DIRECTION_Z_POSITIVE:
                    ChunkOffset.Z = FNavMeshData::ChunkSize;
                    break;
                default:
                    break;
            }
        }
        
        NeighboursLookupData[Index].ChunkKey = (ChunkLocation + ChunkOffset).ToKey();
        NeighboursLookupData[Index].LayerIndex = NeighbourLayerIndex;

        const uint_fast32_t ParentMortonCode = Node.GetMortonCode() & ~((1 << FOctreeNode::ParentShiftAmount[NeighbourLayerIndex]) - 1); // 536870912 = X0, Y0, Z 512
        const F3DVector16 ParentLocalLocation = F3DVector16::FromMortonCode(ParentMortonCode);
        F3DVector16 NeighbourLocalLocation;

        // Calculate morton-code offset
        switch (Direction) {
        case DIRECTION_X_NEGATIVE:
            NeighbourLocalLocation = ParentLocalLocation - F3DVector16(FNavMeshData::MortonOffsets[NeighbourLayerIndex], 0, 0);
            break;
        case DIRECTION_Y_NEGATIVE:
            NeighbourLocalLocation = ParentLocalLocation - F3DVector16(0, FNavMeshData::MortonOffsets[NeighbourLayerIndex], 0);
            break;
        case DIRECTION_Z_NEGATIVE:
            NeighbourLocalLocation = ParentLocalLocation - F3DVector16(0, 0, FNavMeshData::MortonOffsets[NeighbourLayerIndex]);
            break;
        case DIRECTION_X_POSITIVE:
            NeighbourLocalLocation = ParentLocalLocation + F3DVector16(FNavMeshData::MortonOffsets[NeighbourLayerIndex], 0, 0);
            break;
        case DIRECTION_Y_POSITIVE:
            NeighbourLocalLocation = ParentLocalLocation + F3DVector16(0, FNavMeshData::MortonOffsets[NeighbourLayerIndex], 0);
            break;
        case DIRECTION_Z_POSITIVE:
            NeighbourLocalLocation = ParentLocalLocation + F3DVector16(0, 0, FNavMeshData::MortonOffsets[NeighbourLayerIndex]);
            break;
        default:
            break;
        }

        NeighboursLookupData[Index].MortonCode = NeighbourLocalLocation.ToMortonCode();
    }

    return NeighboursLookupData;
}