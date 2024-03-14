#pragma once

std::array<struct FNodeLookupData, 6> GetNeighboursLookupData(const struct FOctreeNode* Node, const struct F3DVector32& ChunkLocation);