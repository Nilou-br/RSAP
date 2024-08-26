// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once
#include "RSAP/Definitions.h"

using namespace Rsap::NavMesh;



/**
 * Each side of a node holds a relation to another node. There is a relation for each side of a node, and it stores the layer and type of the node in this direction.
 * Used for pathfinding. Relations are certain to be valid, meaning we won't have to check for this validity.
 */
struct FNodeRelations
{
	
	rsap_direction LayerIdx_Negative_X: 4 = Layer::Empty;
	rsap_direction LayerIdx_Negative_Y: 4 = Layer::Empty;
	rsap_direction LayerIdx_Negative_Z: 4 = Layer::Empty;
	rsap_direction LayerIdx_Positive_X: 4 = Layer::Empty;
	rsap_direction LayerIdx_Positive_Y: 4 = Layer::Empty;
	rsap_direction LayerIdx_Positive_Z: 4 = Layer::Empty;
	
	node_state NodeState_Negative_X: 1 = Node::State::Static;
	node_state NodeState_Negative_Y: 1 = Node::State::Static;
	node_state NodeState_Negative_Z: 1 = Node::State::Static;
	node_state NodeState_Positive_X: 1 = Node::State::Static;
	node_state NodeState_Positive_Y: 1 = Node::State::Static;
	node_state NodeState_Positive_Z: 1 = Node::State::Static;

	FORCEINLINE rsap_direction GetFromDirection(const rsap_direction Direction) const
	{
		using namespace Direction;
		switch (Direction) {
			case Negative::X: return LayerIdx_Negative_X;
			case Negative::Y: return LayerIdx_Negative_Y;
			case Negative::Z: return LayerIdx_Negative_Z;
			case Positive::X: return LayerIdx_Positive_X;
			case Positive::Y: return LayerIdx_Positive_Y;
			case Positive::Z: return LayerIdx_Positive_Z;
			default: return Layer::Empty;
		}
	}

	FORCEINLINE void SetFromDirection(const rsap_direction Direction, const layer_idx LayerIdx)
	{
		using namespace Direction;
		switch (Direction) {
			case Negative::X: LayerIdx_Negative_X = LayerIdx; break;
			case Negative::Y: LayerIdx_Negative_Y = LayerIdx; break;
			case Negative::Z: LayerIdx_Negative_Z = LayerIdx; break;
			case Positive::X: LayerIdx_Positive_X = LayerIdx; break;
			case Positive::Y: LayerIdx_Positive_Y = LayerIdx; break;
			case Positive::Z: LayerIdx_Positive_Z = LayerIdx; break;
			default: break;
		}
	}

	// Same as SetFromDirection, but will set the opposite relation from the given direction.
	FORCEINLINE void SetFromDirectionInverse(const rsap_direction Direction, const layer_idx LayerIdx)
	{
		using namespace Direction;
		switch (Direction) {
			case Negative::X: LayerIdx_Positive_X = LayerIdx; break;
			case Negative::Y: LayerIdx_Positive_Y = LayerIdx; break;
			case Negative::Z: LayerIdx_Positive_Z = LayerIdx; break;
			case Positive::X: LayerIdx_Negative_X = LayerIdx; break;
			case Positive::Y: LayerIdx_Negative_Y = LayerIdx; break;
			case Positive::Z: LayerIdx_Negative_Z = LayerIdx; break;
			default: break;
		}
	}

	FORCEINLINE bool IsRelationEmpty(const rsap_direction Direction) const
	{
		using namespace Direction;
		switch (Direction) {
			case Negative::X: return LayerIdx_Negative_X == Layer::Empty;
			case Negative::Y: return LayerIdx_Negative_Y == Layer::Empty;
			case Negative::Z: return LayerIdx_Negative_Z == Layer::Empty;
			case Positive::X: return LayerIdx_Positive_X == Layer::Empty;
			case Positive::Y: return LayerIdx_Positive_Y == Layer::Empty;
			case Positive::Z: return LayerIdx_Positive_Z == Layer::Empty;
			default: return false;
		}
	}

	uint32 Pack() const {
		return static_cast<uint32>(LayerIdx_Negative_X)			|
			   static_cast<uint32>(LayerIdx_Negative_Y << 4)	|
			   static_cast<uint32>(LayerIdx_Negative_Z << 8)	|
			   static_cast<uint32>(LayerIdx_Positive_X << 12)	|
			   static_cast<uint32>(LayerIdx_Positive_Y << 16)	|
			   static_cast<uint32>(LayerIdx_Positive_Z << 20)	|
			   	
			   static_cast<uint32>(NodeState_Negative_X << 24)  |
			   static_cast<uint32>(NodeState_Negative_Y << 25)  |
			   static_cast<uint32>(NodeState_Negative_Z << 26)  |
			   static_cast<uint32>(NodeState_Positive_X << 27)  |
			   static_cast<uint32>(NodeState_Positive_Y << 28)  |
			   static_cast<uint32>(NodeState_Positive_Z << 29);
	}

	void Unpack(const uint32 PackedData) {
		LayerIdx_Negative_X  = PackedData;
		LayerIdx_Negative_Y  = PackedData >> 4;
		LayerIdx_Negative_Z  = PackedData >> 8;
		LayerIdx_Positive_X  = PackedData >> 12;
		LayerIdx_Positive_Y  = PackedData >> 16;
		LayerIdx_Positive_Z  = PackedData >> 20;
		
		NodeState_Negative_X = PackedData >> 24;
		NodeState_Negative_Y = PackedData >> 25;
		NodeState_Negative_Z = PackedData >> 26;
		NodeState_Positive_X = PackedData >> 27;
		NodeState_Positive_Y = PackedData >> 28;
		NodeState_Positive_Z = PackedData >> 29;
	}
};