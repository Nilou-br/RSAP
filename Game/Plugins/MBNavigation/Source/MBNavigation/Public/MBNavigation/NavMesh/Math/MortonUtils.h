// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include <array>
#include "MBNavigation/NavMesh/Types/Static.h"
#include "morton.h"


// Provides all functionality for node morton-codes.
struct FNodeMortonUtils
{
	static inline constexpr NodeMortonType Mask_X = 0b00001001001001001001001001001001;
	static inline constexpr NodeMortonType Mask_Y = 0b00010010010010010010010010010010;
	static inline constexpr NodeMortonType Mask_Z = 0b00100100100100100100100100100100;
	
	static inline constexpr NodeMortonType Mask_XY = Mask_X | Mask_Y;
	static inline constexpr NodeMortonType Mask_XZ = Mask_X | Mask_Z;
	static inline constexpr NodeMortonType Mask_YZ = Mask_Y | Mask_Z;
	
	// Accessed using layer-index of the node you would like to get the parent of.
	static inline constexpr NodeMortonType LayerMasks[10] = {
		static_cast<NodeMortonType>(~((1 << 30) - 1)),
		static_cast<NodeMortonType>(~((1 << 27) - 1)),
		static_cast<NodeMortonType>(~((1 << 24) - 1)),
		static_cast<NodeMortonType>(~((1 << 21) - 1)),
		static_cast<NodeMortonType>(~((1 << 18) - 1)),
		static_cast<NodeMortonType>(~((1 << 15) - 1)),
		static_cast<NodeMortonType>(~((1 << 12) - 1)),
		static_cast<NodeMortonType>(~((1 << 9)  - 1)),
		static_cast<NodeMortonType>(~((1 << 6)  - 1)),
		static_cast<NodeMortonType>(~((1 << 3)  - 1))
	};
	
	// The offsets are: 1024, 512, 256, 128, 64, 32, 16, 8, 4, 2.
	static inline constexpr NodeMortonType LayerOffsets[10] = {
		
		// These are used to offset a single axis on the morton code by a specific node-size.
		// This only works with 'powers of 2' due to the nature of morton-codes, which is how my navmesh is build.
	
		// Every axis can use the same offset. This is because the offsets start at the first bit of an interleaved 'zyx' part ( the bit for 'x' ).
		// Explanation: when masking any two axis on the morton-code, and adding any offset to this result, then the first bit to the left of the offset that is '0' will be set to '1'.
		// The axis you are trying to add the offset to is the only one that remains unmasked, so its the only one that can have any bits set to '0'.
		
		1 << 30, 1 << 27, 1 << 24, 1 << 21, 1 << 18,
		1 << 15, 1 << 12, 1 << 9,  1 << 6,  1 << 3
	};
	

	
	// Get the parent's morton-code. The layer-index is the index of the layer the parent is in.
	FORCEINLINE static NodeMortonType GetParent(const NodeMortonType MortonCode, const LayerIdxType LayerIdx)
	{
		return MortonCode & LayerMasks[LayerIdx-1];
	}

	FORCEINLINE static NodeMortonType GetChild(const NodeMortonType ParentMortonCode, const LayerIdxType ChildLayerIdx, const uint8 ChildIdx)
	{
		switch (ChildIdx)
		{
			case 0: return ParentMortonCode;																// No offset.
			case 1: return AddX(ParentMortonCode, ChildLayerIdx);											// X
			case 2: return AddY(ParentMortonCode, ChildLayerIdx);											// Y
			case 3: return AddX(ParentMortonCode, ChildLayerIdx) | AddY(ParentMortonCode, ChildLayerIdx);	// X+Y
			case 4: return AddZ(ParentMortonCode, ChildLayerIdx);											// Z
			case 5: return AddX(ParentMortonCode, ChildLayerIdx) | AddZ(ParentMortonCode, ChildLayerIdx);	// X+Z
			case 6: return AddY(ParentMortonCode, ChildLayerIdx) | AddZ(ParentMortonCode, ChildLayerIdx);	// Y+Z
			case 7: return Add(ParentMortonCode, ChildLayerIdx);											// X+Y+Z
			default: return ParentMortonCode;
		}
	}

	FORCEINLINE static std::array<NodeMortonType, 8> GetChildren(const NodeMortonType ParentMortonCode, const LayerIdxType ChildLayerIdx)
	{
		// todo: test performance.
		return {
			ParentMortonCode,
			AddX(ParentMortonCode, ChildLayerIdx),
			AddY(ParentMortonCode, ChildLayerIdx),
			AddX(ParentMortonCode, ChildLayerIdx) | AddY(ParentMortonCode, ChildLayerIdx),
			AddZ(ParentMortonCode, ChildLayerIdx),
			AddX(ParentMortonCode, ChildLayerIdx) | AddZ(ParentMortonCode, ChildLayerIdx),
			AddY(ParentMortonCode, ChildLayerIdx) | AddZ(ParentMortonCode, ChildLayerIdx),
			Add(ParentMortonCode, ChildLayerIdx)
		};
	}

	// Moves the morton-code in the given direction. The amount it moves is determined by the layer-index, which translates to the node-size for that layer.
	FORCEINLINE static NodeMortonType Move(const NodeMortonType MortonCode, const LayerIdxType LayerIdx, const DirectionType Direction)
	{
		switch (Direction) {
			case Direction::X_Negative: return SubtractX(MortonCode, LayerIdx);
			case Direction::Y_Negative: return SubtractY(MortonCode, LayerIdx);
			case Direction::Z_Negative: return SubtractZ(MortonCode, LayerIdx);
			case Direction::X_Positive: return AddX(MortonCode, LayerIdx);
			case Direction::Y_Positive: return AddY(MortonCode, LayerIdx);
			case Direction::Z_Positive: return AddZ(MortonCode, LayerIdx);
			default: return MortonCode;
		}
	}

	// Moves the morton-code in the direction, and also masks away the layers below the layer-index. Used to get the neighbour of a node in the given direction, which could also be in an upper layer.
	FORCEINLINE static NodeMortonType MoveAndMask(const NodeMortonType MortonCode, const LayerIdxType LayerIdx, const DirectionType Direction)
	{
		return Move(MortonCode, LayerIdx, Direction) & LayerMasks[LayerIdx];
	}

	// Adds the node-size of the layer-index to the X-axis.
	FORCEINLINE static NodeMortonType AddX(const NodeMortonType MortonCode, const LayerIdxType LayerIdx)
	{
		const NodeMortonType SumX = (MortonCode | Mask_YZ) + LayerOffsets[LayerIdx];
		return SumX & Mask_X | MortonCode & Mask_YZ;
	}

	// Subtracts the node-size of the layer-index from the X-axis.
	FORCEINLINE static NodeMortonType SubtractX(const NodeMortonType MortonCode, const LayerIdxType LayerIdx)
	{
		const NodeMortonType DiffX = (MortonCode & Mask_X) - LayerOffsets[LayerIdx];
		return DiffX & Mask_X | MortonCode & Mask_YZ;
	}

	// Adds the node-size of the layer-index to the Y-axis.
	FORCEINLINE static NodeMortonType AddY(const NodeMortonType MortonCode, const LayerIdxType LayerIdx)
	{
		const NodeMortonType SumY = (MortonCode | Mask_XZ) + LayerOffsets[LayerIdx];
		return SumY & Mask_Y | MortonCode & Mask_XZ;
	}

	// Subtracts the node-size of the layer-index from the Y-axis.
	FORCEINLINE static NodeMortonType SubtractY(const NodeMortonType MortonCode, const LayerIdxType LayerIdx)
	{
		const NodeMortonType DiffY = (MortonCode & Mask_Y) - LayerOffsets[LayerIdx];
		return DiffY & Mask_Y | MortonCode & Mask_XZ;
	}

	// Adds the node-size of the layer-index to the Z-axis.
	FORCEINLINE static NodeMortonType AddZ(const NodeMortonType MortonCode, const LayerIdxType LayerIdx)
	{
		const NodeMortonType SumZ = (MortonCode | Mask_XY) + LayerOffsets[LayerIdx];
		return SumZ & Mask_Z | MortonCode & Mask_XY;
	}

	// Subtracts the node-size of the layer-index from the Z-axis.
	FORCEINLINE static NodeMortonType SubtractZ(const NodeMortonType MortonCode, const LayerIdxType LayerIdx)
	{
		const NodeMortonType DiffZ = (MortonCode & Mask_Z) - LayerOffsets[LayerIdx];
		return DiffZ & Mask_Z | MortonCode & Mask_XY;
	}

	// Adds the node-size of the layer-index to all axis.
	FORCEINLINE static NodeMortonType Add(const NodeMortonType MortonCode, const LayerIdxType LayerIdx)
	{
		const NodeMortonType SumX = (MortonCode | Mask_YZ) + LayerOffsets[LayerIdx];
		const NodeMortonType SumY = (MortonCode | Mask_XZ) + LayerOffsets[LayerIdx];
		const NodeMortonType SumZ = (MortonCode | Mask_XY) + LayerOffsets[LayerIdx];
		return SumX & Mask_X | SumY & Mask_Y | SumZ & Mask_Z;
	}

	// Subtracts the node-size of the layer-index from all axis.
	FORCEINLINE static NodeMortonType Subtract(const NodeMortonType MortonCode, const LayerIdxType LayerIdx)
	{
		const NodeMortonType DiffX = (MortonCode & Mask_X) - LayerOffsets[LayerIdx];
		const NodeMortonType DiffY = (MortonCode & Mask_Y) - LayerOffsets[LayerIdx];
		const NodeMortonType DiffZ = (MortonCode & Mask_Z) - LayerOffsets[LayerIdx];
		return DiffX & Mask_X | DiffY & Mask_Y | DiffZ & Mask_Z;
	}
};



// Provides all functionality for chunk morton-codes.
struct FChunkMortonUtils
{
	static inline constexpr ChunkMortonType Mask_X = 0b0001001001001001001001001001001001001001001001001001001001001001;
	static inline constexpr ChunkMortonType Mask_Y = 0b0010010010010010010010010010010010010010010010010010010010010010;
	static inline constexpr ChunkMortonType Mask_Z = 0b0100100100100100100100100100100100100100100100100100100100100100;

	static inline constexpr ChunkMortonType Mask_XY = Mask_X | Mask_Y;
	static inline constexpr ChunkMortonType Mask_XZ = Mask_X | Mask_Z;
	static inline constexpr ChunkMortonType Mask_YZ = Mask_Y | Mask_Z;
	
	static inline constexpr uint32 EncodeDecodeOffset = 1073741312; // To convert between morton-space and global-space for a chunk.


	
	// Encode global world coordinates into a chunk morton-code. Max range from -1073741312 to +1073741312.
	FORCEINLINE static ChunkMortonType Encode(const int32 X, const int32 Y, const int32 Z) // todo: test these
	{
		// Then left shift the chunk-size to fit back into 32 bits.
		const uint_fast32_t InX = (X + EncodeDecodeOffset) << FNavMeshStatic::ChunkKeyShift;
		const uint_fast32_t InY = (Y + EncodeDecodeOffset) << FNavMeshStatic::ChunkKeyShift;
		const uint_fast32_t InZ = (Z + EncodeDecodeOffset) << FNavMeshStatic::ChunkKeyShift;
		
		return libmorton::morton3D_64_encode(InX, InY, InZ);
	}

	// Decode a chunk's morton-code back into global world coordinates.
	FORCEINLINE static void Decode(const ChunkMortonType ChunkMorton, int32& OutX, int32& OutY, int32& OutZ)
	{
		uint_fast32_t X, Y, Z;
		libmorton::morton3D_64_decode(ChunkMorton, X, Y, Z);
		
		OutX = (X >> FNavMeshStatic::ChunkKeyShift) - EncodeDecodeOffset;
		OutY = (Y >> FNavMeshStatic::ChunkKeyShift) - EncodeDecodeOffset;
		OutZ = (Z >> FNavMeshStatic::ChunkKeyShift) - EncodeDecodeOffset;
	}
	
	// Moves the morton-code exactly one chunk in the given direction.
	FORCEINLINE static ChunkMortonType Move(const ChunkMortonType MortonCode, const DirectionType Direction)
	{
		switch (Direction) {
			case Direction::X_Negative: return SubtractX(MortonCode);
			case Direction::Y_Negative: return SubtractY(MortonCode);
			case Direction::Z_Negative: return SubtractZ(MortonCode);
			case Direction::X_Positive: return AddX(MortonCode);
			case Direction::Y_Positive: return AddY(MortonCode);
			case Direction::Z_Positive: return AddZ(MortonCode);
			default: return MortonCode;
		}
	}

	// Moves one chunk positively along the X-axis.
	FORCEINLINE static ChunkMortonType AddX(const ChunkMortonType MortonCode)
	{
		const ChunkMortonType SumX = (MortonCode | Mask_YZ) + 1;
		return SumX & Mask_X | MortonCode & Mask_YZ;
	}

	// Moves one chunk negatively along the X-axis.
	FORCEINLINE static ChunkMortonType SubtractX(const ChunkMortonType MortonCode)
	{
		const ChunkMortonType DiffX = (MortonCode & Mask_X) - 1;
		return DiffX & Mask_X | MortonCode & Mask_YZ;
	}

	// Moves one chunk positively along the Y-axis.
	FORCEINLINE static ChunkMortonType AddY(const ChunkMortonType MortonCode)
	{
		const ChunkMortonType SumY = (MortonCode | Mask_XZ) + 1;
		return SumY & Mask_Y | MortonCode & Mask_XZ;
	}

	// Moves one chunk negatively along the Y-axis.
	FORCEINLINE static ChunkMortonType SubtractY(const ChunkMortonType MortonCode)
	{
		const ChunkMortonType DiffY = (MortonCode & Mask_Y) - 1;
		return DiffY & Mask_Y | MortonCode & Mask_XZ;
	}

	// Moves one chunk positively along the Z-axis.
	FORCEINLINE static ChunkMortonType AddZ(const ChunkMortonType MortonCode)
	{
		const ChunkMortonType SumZ = (MortonCode | Mask_XY) + 1;
		return SumZ & Mask_Z | MortonCode & Mask_XY;
	}

	// Moves one chunk negatively along the Z-axis.
	FORCEINLINE static ChunkMortonType SubtractZ(const ChunkMortonType MortonCode)
	{
		const ChunkMortonType DiffZ = (MortonCode & Mask_Z) - 1;
		return DiffZ & Mask_Z | MortonCode & Mask_XY;
	}
};