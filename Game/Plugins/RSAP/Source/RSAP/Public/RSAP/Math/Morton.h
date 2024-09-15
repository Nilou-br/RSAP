// Copyright Melvin Brink 2023. All Rights Reserved.

// ReSharper disable CppUE4CodingStandardNamingViolationWarning
#pragma once

#include "RSAP/Definitions.h"
#include "RSAP/ThirdParty/LibMorton/morton.h"



// Provides functionality for morton-codes.
struct FMortonUtils
{
	struct Node
	{
		static inline constexpr node_morton Mask_X = 0b00001001001001001001001001001001;
		static inline constexpr node_morton Mask_Y = 0b00010010010010010010010010010010;
		static inline constexpr node_morton Mask_Z = 0b00100100100100100100100100100100;
		
		static inline constexpr node_morton Mask_XY = Mask_X | Mask_Y;
		static inline constexpr node_morton Mask_XZ = Mask_X | Mask_Z;
		static inline constexpr node_morton Mask_YZ = Mask_Y | Mask_Z;
		
		// Accessed using parent-layer-index of the node you would like to get the parent of.
		static inline constexpr node_morton ParentMasks[11] = {
			static_cast<node_morton>(~((1 << 30) - 1)),
			static_cast<node_morton>(~((1 << 27) - 1)),
			static_cast<node_morton>(~((1 << 24) - 1)),
			static_cast<node_morton>(~((1 << 21) - 1)),
			static_cast<node_morton>(~((1 << 18) - 1)),
			static_cast<node_morton>(~((1 << 15) - 1)),
			static_cast<node_morton>(~((1 << 12) - 1)),
			static_cast<node_morton>(~((1 << 9)  - 1)),
			static_cast<node_morton>(~((1 << 6)  - 1)),
			static_cast<node_morton>(~((1 << 3)  - 1)),
			static_cast<node_morton>(~0)
		};
		
		// The offsets by index are: 1024, 512, 256, 128, 64, 32, 16, 8, 4, 2.
		static inline constexpr node_morton LayerOffsets[11] = {
			
			// These are used to offset a single axis on the morton code by a specific node-size.
			// This only works with values that are a powers of 2. Only a single bit can be set to '1' as the offset.
			// Every axis can use these values to offset by a specific node-size.
			
			// Explanation on how to offset:
			// When you want to offset 'X', for example, you will want to create a masked morton-code where all the bits on the Y and Z axis are all set to '1'.
			// If you then apply this offset on top of this masked morton-code, it will add this to the 'X' axis as if you were doing a normal '+' operation. This is because the bits for Y and Z are all set to '1', so they will all flow to the left ( binary counting ).
			// The reason why this only works for powers of 2 is because the Y and Z axis will flip to '0' when a single bit on 'X' flows to the left.
			
			1 << 30, 1 << 27, 1 << 24, 1 << 21, 1 << 18,
			1 << 15, 1 << 12, 1 << 9,  1 << 6,  1 << 3,
			1
		};
		
		// Encode the node's local coordinates into a node-morton-code.
		FORCEINLINE static node_morton Encode(const uint_fast16_t X, const uint_fast16_t Y, const uint_fast16_t Z)
		{
			using namespace Rsap::NavMesh;
			return libmorton::morton3D_32_encode(X, Y, Z);
		}

		// Decode a node-morton-code back into local node coordinates.
		FORCEINLINE static void Decode(const node_morton MortonCode, uint16& OutX, uint16& OutY, uint16& OutZ)
		{
			using namespace Rsap::NavMesh;
			uint_fast16_t X, Y, Z;
			libmorton::morton3D_32_decode(MortonCode, X, Y, Z);
			OutX = X;
			OutY = Y;
			OutZ = Z;
		}
		
		// Get the parent's morton-code.
		FORCEINLINE static node_morton GetParent(const node_morton MortonCode, const layer_idx ParentLayerIdx)
		{
			return MortonCode & ParentMasks[ParentLayerIdx];
		}

		// Get the child-idx of the node.
		FORCEINLINE static child_idx GetChildIndex(const node_morton MortonCode, const layer_idx LayerIdx)
		{
			// Shift the MortonCode, and mask the last 3 bits.
			// The remainder evaluates directly to the node's index in it's parent.
			static constexpr node_morton ChildIdxMask = 0b00000000000000000000000000000111;
			static constexpr uint8 Shifts[10] = {30, 27, 24, 21, 18, 15, 12, 9, 6, 3};
			return (MortonCode >> Shifts[LayerIdx]) & ChildIdxMask;
		}

		FORCEINLINE static node_morton GetChild(const node_morton ParentMortonCode, const layer_idx ChildLayerIdx, const child_idx ChildIdx)
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

		FORCEINLINE static std::array<node_morton, 8> GetChildren(const node_morton ParentMortonCode, const layer_idx ChildLayerIdx)
		{
			// Compute these values once and reuse them.
			const node_morton AddedX = AddX(ParentMortonCode, ChildLayerIdx);
			const node_morton AddedY = AddY(ParentMortonCode, ChildLayerIdx);
			const node_morton AddedZ = AddZ(ParentMortonCode, ChildLayerIdx);
			
			return {
				ParentMortonCode,
				AddedX,
				AddedY,
				AddedX | AddedY,
				AddedZ,
				AddedX | AddedZ,
				AddedY | AddedZ,
				AddedX | AddedY | AddedZ
			};
		}

		// Moves the morton-code in the given direction. The amount it moves is determined by the layer-index, which translates to the node-size for that layer.
		FORCEINLINE static node_morton Move(const node_morton MortonCode, const layer_idx LayerIdx, const rsap_direction Direction)
		{
			using namespace Rsap::NavMesh::Direction;
			switch (Direction) {
				case Negative::X: return SubtractX(MortonCode, LayerIdx);
				case Negative::Y: return SubtractY(MortonCode, LayerIdx);
				case Negative::Z: return SubtractZ(MortonCode, LayerIdx);
				case Positive::X: return AddX(MortonCode, LayerIdx);
				case Positive::Y: return AddY(MortonCode, LayerIdx);
				case Positive::Z: return AddZ(MortonCode, LayerIdx);
				default: return MortonCode;
			}
		}

		// Gets the neighbour's morton-code of a node in the given direction, which could also be in an upper layer.
		FORCEINLINE static node_morton GetNeighbour(const node_morton MortonCode, const layer_idx NeighbourLayerIdx, const rsap_direction Direction)
		{
			return Move(MortonCode, NeighbourLayerIdx, Direction) & ParentMasks[NeighbourLayerIdx];
		}

		// Adds the node-size of the layer-index to the X-axis.
		FORCEINLINE static node_morton AddX(const node_morton MortonCode, const layer_idx LayerIdx)
		{
			const node_morton SumX = (MortonCode | Mask_YZ) + LayerOffsets[LayerIdx];
			return SumX & Mask_X | MortonCode & Mask_YZ;
		}

		// Subtracts the node-size of the layer-index from the X-axis.
		FORCEINLINE static node_morton SubtractX(const node_morton MortonCode, const layer_idx LayerIdx)
		{
			const node_morton DiffX = (MortonCode & Mask_X) - LayerOffsets[LayerIdx];
			return DiffX & Mask_X | MortonCode & Mask_YZ;
		}

		// Adds the node-size of the layer-index to the Y-axis.
		FORCEINLINE static node_morton AddY(const node_morton MortonCode, const layer_idx LayerIdx)
		{
			const node_morton SumY = (MortonCode | Mask_XZ) + LayerOffsets[LayerIdx];
			return SumY & Mask_Y | MortonCode & Mask_XZ;
		}

		// Subtracts the node-size of the layer-index from the Y-axis.
		FORCEINLINE static node_morton SubtractY(const node_morton MortonCode, const layer_idx LayerIdx)
		{
			const node_morton DiffY = (MortonCode & Mask_Y) - LayerOffsets[LayerIdx];
			return DiffY & Mask_Y | MortonCode & Mask_XZ;
		}

		// Adds the node-size of the layer-index to the Z-axis.
		FORCEINLINE static node_morton AddZ(const node_morton MortonCode, const layer_idx LayerIdx)
		{
			const node_morton SumZ = (MortonCode | Mask_XY) + LayerOffsets[LayerIdx];
			return SumZ & Mask_Z | MortonCode & Mask_XY;
		}

		// Subtracts the node-size of the layer-index from the Z-axis.
		FORCEINLINE static node_morton SubtractZ(const node_morton MortonCode, const layer_idx LayerIdx)
		{
			const node_morton DiffZ = (MortonCode & Mask_Z) - LayerOffsets[LayerIdx];
			return DiffZ & Mask_Z | MortonCode & Mask_XY;
		}

		// Adds the node-size of the layer-index to all axis.
		FORCEINLINE static node_morton Add(const node_morton MortonCode, const layer_idx LayerIdx)
		{
			const node_morton SumX = (MortonCode | Mask_YZ) + LayerOffsets[LayerIdx];
			const node_morton SumY = (MortonCode | Mask_XZ) + LayerOffsets[LayerIdx];
			const node_morton SumZ = (MortonCode | Mask_XY) + LayerOffsets[LayerIdx];
			return SumX & Mask_X | SumY & Mask_Y | SumZ & Mask_Z;
		}

		// Subtracts the node-size of the layer-index from all axis.
		FORCEINLINE static node_morton Subtract(const node_morton MortonCode, const layer_idx LayerIdx)
		{
			const node_morton DiffX = (MortonCode & Mask_X) - LayerOffsets[LayerIdx];
			const node_morton DiffY = (MortonCode & Mask_Y) - LayerOffsets[LayerIdx];
			const node_morton DiffZ = (MortonCode & Mask_Z) - LayerOffsets[LayerIdx];
			return DiffX & Mask_X | DiffY & Mask_Y | DiffZ & Mask_Z;
		}

		// Copies the X coordinate from rhs into lhs and returns the modified lhs.
		FORCEINLINE static node_morton CopyX(const node_morton lhs, const node_morton rhs)
		{
			return lhs & Mask_YZ | rhs & Mask_X;
		}

		// Copies the Y coordinate from rhs into lhs and returns the modified lhs.
		FORCEINLINE static node_morton CopyY(const node_morton lhs, const node_morton rhs)
		{
			return lhs & Mask_XZ | rhs & Mask_Y;
		}

		// Copies the Z coordinate from rhs into lhs and returns the modified lhs.
		FORCEINLINE static node_morton CopyZ(const node_morton lhs, const node_morton rhs)
		{
			return lhs & Mask_XY | rhs & Mask_Z;
		}

		FORCEINLINE static bool XEqualsZero(const node_morton MortonCode) { return (MortonCode & Mask_X) == 0; }
		FORCEINLINE static bool YEqualsZero(const node_morton MortonCode) { return (MortonCode & Mask_Y) == 0; }
		FORCEINLINE static bool ZEqualsZero(const node_morton MortonCode) { return (MortonCode & Mask_Z) == 0; }

		FORCEINLINE static bool HasMovedIntoNewChunk(const node_morton PrevMortonCode, const node_morton CurrMortonCode, const rsap_direction Direction)
		{
			using namespace Rsap::NavMesh::Direction;
			switch (Direction) {
				case Negative::X: return XEqualsZero(PrevMortonCode);
				case Negative::Y: return YEqualsZero(PrevMortonCode);
				case Negative::Z: return ZEqualsZero(PrevMortonCode);
				case Positive::X: return XEqualsZero(CurrMortonCode);
				case Positive::Y: return YEqualsZero(CurrMortonCode);
				case Positive::Z: return ZEqualsZero(CurrMortonCode);
				default: return false;
			}
		}
	};


	
	struct Chunk
	{
		static inline constexpr chunk_morton Mask_X = 0b0001001001001001001001001001001001001001001001001001001001001001;
		static inline constexpr chunk_morton Mask_Y = 0b0010010010010010010010010010010010010010010010010010010010010010;
		static inline constexpr chunk_morton Mask_Z = 0b0100100100100100100100100100100100100100100100100100100100100100;

		static inline constexpr chunk_morton Mask_XY = Mask_X | Mask_Y;
		static inline constexpr chunk_morton Mask_XZ = Mask_X | Mask_Z;
		static inline constexpr chunk_morton Mask_YZ = Mask_Y | Mask_Z;
		
		// Encode the global world coordinates into a chunk morton-code.
		static chunk_morton Encode(const int32 X, const int32 Y, const int32 Z) // todo: test these: todo: add FORCEINLINE
		{
			using namespace Rsap::NavMesh;
			
			// Apply offset to make the coordinates positive, and do a shift to compress the value.
			// The last 'SizeBits' amount of bits will always be set to '0'.
			const uint_fast32_t InX = (X + Rsap::NavMesh::Chunk::SignOffset) >> Rsap::NavMesh::Chunk::SizeBits;
			const uint_fast32_t InY = (Y + Rsap::NavMesh::Chunk::SignOffset) >> Rsap::NavMesh::Chunk::SizeBits;
			const uint_fast32_t InZ = (Z + Rsap::NavMesh::Chunk::SignOffset) >> Rsap::NavMesh::Chunk::SizeBits;
			
			return libmorton::morton3D_64_encode(InX, InY, InZ);
		}

		// Decode a chunk's morton-code back into global world coordinates.
		static void Decode(const chunk_morton ChunkMorton, int32& OutX, int32& OutY, int32& OutZ)
		{
			uint_fast32_t X, Y, Z;
			libmorton::morton3D_64_decode(ChunkMorton, X, Y, Z);
			
			OutX = (X << Rsap::NavMesh::Chunk::SizeBits) - Rsap::NavMesh::Chunk::SignOffset;
			OutY = (Y << Rsap::NavMesh::Chunk::SizeBits) - Rsap::NavMesh::Chunk::SignOffset;
			OutZ = (Z << Rsap::NavMesh::Chunk::SizeBits) - Rsap::NavMesh::Chunk::SignOffset;
		}
		
		// Moves the morton-code exactly one chunk in the given direction. todo: rename to GetNeighbour?
		FORCEINLINE static chunk_morton Move(const chunk_morton MortonCode, const rsap_direction Direction)
		{
			using namespace Rsap::NavMesh::Direction;
			switch (Direction) {
				case Negative::X: return DecrementX(MortonCode);
				case Negative::Y: return DecrementY(MortonCode);
				case Negative::Z: return DecrementZ(MortonCode);
				case Positive::X: return IncrementX(MortonCode);
				case Positive::Y: return IncrementY(MortonCode);
				case Positive::Z: return IncrementZ(MortonCode);
				default: return MortonCode;
			}
		}

		// Get the neighbouring chunk's morton-code in the given direction.
		FORCEINLINE static chunk_morton GetNeighbour(const chunk_morton MortonCode, const rsap_direction Direction)
		{
			using namespace Rsap::NavMesh::Direction;
			switch (Direction) {
				case Negative::X: return DecrementX(MortonCode);
				case Negative::Y: return DecrementY(MortonCode);
				case Negative::Z: return DecrementZ(MortonCode);
				case Positive::X: return IncrementX(MortonCode);
				case Positive::Y: return IncrementY(MortonCode);
				case Positive::Z: return IncrementZ(MortonCode);
				default: return MortonCode;
			}
		}

		// Moves one chunk positively along the X-axis.
		FORCEINLINE static chunk_morton IncrementX(const chunk_morton MortonCode)
		{
			const chunk_morton SumX = (MortonCode | Mask_YZ) + 1;
			return SumX & Mask_X | MortonCode & Mask_YZ;
		}

		// Moves one chunk negatively along the X-axis.
		FORCEINLINE static chunk_morton DecrementX(const chunk_morton MortonCode)
		{
			const chunk_morton DiffX = (MortonCode & Mask_X) - 1;
			return DiffX & Mask_X | MortonCode & Mask_YZ;
		}

		// Moves one chunk positively along the Y-axis.
		FORCEINLINE static chunk_morton IncrementY(const chunk_morton MortonCode)
		{
			const chunk_morton SumY = (MortonCode | Mask_XZ) + 1;
			return SumY & Mask_Y | MortonCode & Mask_XZ;
		}

		// Moves one chunk negatively along the Y-axis.
		FORCEINLINE static chunk_morton DecrementY(const chunk_morton MortonCode)
		{
			const chunk_morton DiffY = (MortonCode & Mask_Y) - 1;
			return DiffY & Mask_Y | MortonCode & Mask_XZ;
		}

		// Moves one chunk positively along the Z-axis.
		FORCEINLINE static chunk_morton IncrementZ(const chunk_morton MortonCode)
		{
			const chunk_morton SumZ = (MortonCode | Mask_XY) + 1;
			return SumZ & Mask_Z | MortonCode & Mask_XY;
		}

		// Moves one chunk negatively along the Z-axis.
		FORCEINLINE static chunk_morton DecrementZ(const chunk_morton MortonCode)
		{
			const chunk_morton DiffZ = (MortonCode & Mask_Z) - 1;
			return DiffZ & Mask_Z | MortonCode & Mask_XY;
		}
		
		// Copies the X coordinate from rhs into lhs and returns the modified lhs.
		FORCEINLINE static chunk_morton CopyX(const chunk_morton lhs, const chunk_morton rhs)
		{
			return lhs & Mask_YZ | rhs & Mask_X;
		}

		// Copies the Y coordinate from rhs into lhs and returns the modified lhs.
		FORCEINLINE static chunk_morton CopyY(const chunk_morton lhs, const chunk_morton rhs)
		{
			return lhs & Mask_XZ | rhs & Mask_Y;
		}

		// Copies the Z coordinate from rhs into lhs and returns the modified lhs.
		FORCEINLINE static chunk_morton CopyZ(const chunk_morton lhs, const chunk_morton rhs)
		{
			return lhs & Mask_XY | rhs & Mask_Z;
		}
	};
};