// // Copyright Melvin Brink 2023. All Rights Reserved.
//
// #include "RSAP/NavMesh/Types/Chunk.h"
// #include "RSAP/NavMesh/Types/Node.h"
// #include "RSAP/Math/Bounds.h"
// #include "RSAP/Math/Overlap.h"
// #include <chrono>
// #include <ranges>
// #include <set>
//
//
//
// // void FNavMeshGenerator::Generate(const FBoundsMap& BoundsMap)
// // {
// // 	TRACE_CPUPROFILER_EVENT_SCOPE_STR("NavMesh Generate");
// // 	if (!World)
// // 	{
// // 		UE_LOG(LogRsap, Error, TEXT("Invalid 'World'. Cannot generate the navmesh without an existing world."))
// // 		return;
// // 	}
// //
// // #if WITH_EDITOR
// // 	const auto StartTime = std::chrono::high_resolution_clock::now();
// // #endif
// //
// // 	// Start generation
// // 	NavMesh->clear();
// // 	GenerateChunks(BoundsMap);
// //
// // #if WITH_EDITOR
// // 	const float DurationSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(
// // 		std::chrono::high_resolution_clock::now() - StartTime).count() / 1000.0f;
// // 	UE_LOG(LogNavMeshGenerator, Log, TEXT("Generation took : '%f' seconds"), DurationSeconds);
// // #endif
// // 	
// // }
//
// /*
//  * Create a grid of chunks filling the entire area of the level-boundaries.
//  * Chunks are be placed so that their origin align with the world coordinates x0,y0,z0.
//  */
// void GenerateChunks(const FBoundsMap& BoundsMap)
// {
// 	// Get all the chunks.
// 	std::set<chunk_morton> ChunkMortonCodes;
// 	for (auto Bounds : BoundsMap | std::views::values)
// 	{
// 		const std::unordered_set<chunk_morton> IntersectingChunks = Bounds.GetIntersectingChunks();
// 		ChunkMortonCodes.insert(IntersectingChunks.begin(), IntersectingChunks.end());
// 	}
//
// 	for (auto ChunkMorton : ChunkMortonCodes)
// 	{
// 		// Skip this chunk if it does not overlap anything.
// 		if(!HasOverlap(World, FGlobalVector::FromChunkMorton(ChunkMorton), 0)) continue;
//
// 		// Initialize chunk.
// 		auto [ChunkIterator, IsInserted] = NavMesh->emplace(ChunkMorton, FChunk(ChunkMorton, 0));
// 		FChunk& Chunk = ChunkIterator->second;
// 		
// 		FOctreeLayer& Layer = *Chunk.Octrees[0]->Layers[0];
//
// 		// Rasterize the octree starting from the root-node until static-depth is reached.
// 		auto [NodeIterator, bInserted] = Layer.emplace(0, FNode(static_cast<rsap_direction>(Direction::All)));
// 		RasterizeStaticNode(Chunk, *NodeIterator, 0);
//
// 		// Set all the relations to the nodes that are in the negative direction from this chunk.
// 		// Chunks are generated from negative to positive, so any chunks in the positive direction do not exist yet.
// 		SetNegativeNeighbourRelations(Chunk);
// 	}
// }
//
// /**
//  * Rasterize a static node, only if it occludes anything.
//  * This method is called recursively until it either reaches the static-depth or if it does not occlude anything.
//  */
// void FNavMeshGenerator::RasterizeStaticNode(FChunk& Chunk, FNodePair& NodePair, const layer_idx LayerIdx)
// {
// 	const node_morton MortonCode = NodePair.first;
// 	FNode& Node = NodePair.second;
// 	
// 	// If overlapping any static object.
// 	if (!Node.HasOverlap(World, Chunk.Location, MortonCode, LayerIdx)) return;
// 	Node.SetOccluded(true);
//
// 	// Stop recursion at the static-depth.
// 	if (LayerIdx >= RsapStatic::StaticDepth) return;
// 	Node.SetHasChildren(true);
//
// 	const layer_idx ChildLayerIdx = LayerIdx + 1;
// 	FOctreeLayer& ChildLayer = *Chunk.Octrees[0]->Layers[ChildLayerIdx];
//
// 	// Initialize the children of this node.
// 	ChildLayer.reserve(ChildLayer.size() + 8);
// 	for (uint8 ChildIdx = 0; ChildIdx < 8; ++ChildIdx)
// 	{
// 		const node_morton ChildMortonCode = FNodeMortonUtils::GetChild(MortonCode, ChildLayerIdx, ChildIdx);
//
// 		// Add this new child-node to the child-layer.
// 		const auto ChildNodeIterator = Chunk.Octrees[0]->Layers[ChildLayerIdx]->emplace(ChildMortonCode, FNode(ChildIdx, Node.ChunkBorder)).first;
//
// 		// Recursively rasterize this child-node.
// 		RasterizeStaticNode(Chunk, *ChildNodeIterator, ChildLayerIdx);
// 	}
// }
//
// /**
//  * Sets all the neighbour relations on the nodes within the static octree of the given chunk.
//  * 
//  * If a neighbour is found, it's layer-index will be added to the node's neighbours,
//  * and vice versa the node's layer-index on the found-neighbour
//  *
//  * Only neighbouring node's on the same-layer as, or higher than ( lower res ), will be added as a neighbour on a node.
//  * This means that node's cannot have neighbours smaller in size than themselves.
//  *
//  * @note should be called during the generation loop from negative most to positive most chunk.
//  * 
//  * @param Chunk Chunk holding the nodes you want to set the relations of.
//  */
// void FNavMeshGenerator::SetNegativeNeighbourRelations(const FChunk& Chunk)
// {
// 	TRACE_CPUPROFILER_EVENT_SCOPE_STR("SetNegativeNeighbourRelations");
// 	
// 	// Loop through all static nodes sorted by morton-code.
// 	layer_idx LayerIdx = 0; 
// 	for (auto& Layer : Chunk.Octrees[0]->Layers)
// 	{
// 		for (auto& [MortonCode, Node] : *Layer)
// 		{
// 			// Node.UpdateRelations(NavMesh, Chunk, MortonCode, LayerIdx, Direction::XYZ_Negative);
// 		}
// 		++LayerIdx;
// 	}
// }