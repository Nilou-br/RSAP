// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once



/**
 * Same as RsapStatic but stores static debug values instead.
 *
 * Used by the debugger to determine what to draw.
 */
struct FNavMeshDebugSettings
{
	static inline bool bDebugEnabled = false;
	static inline bool bDisplayNodes = false;
	static inline bool bDisplayNodeBorder = false;
	static inline bool bDisplayRelations = false;
	static inline bool bDisplayPaths = false;
	static inline bool bDisplayChunks = false;

	static void Initialize(
		const bool InbDebugEnabled = false, const bool InbDisplayNodes = false,
		const bool InbDisplayNodeBorder = false, const bool InbDisplayRelations = false,
		const bool InbDisplayPaths = false, const bool InbDisplayChunks = false)
	{
		bDebugEnabled = InbDebugEnabled;
		bDisplayNodes = InbDisplayNodes;
		bDisplayNodeBorder = InbDisplayNodeBorder;
		bDisplayRelations = InbDisplayRelations;
		bDisplayPaths = InbDisplayPaths;
		bDisplayChunks = InbDisplayChunks;
	}

	FORCEINLINE static bool ShouldDisplayDebug()
	{
		return bDebugEnabled && (bDisplayNodes || bDisplayNodeBorder || bDisplayRelations || bDisplayPaths || bDisplayChunks);
	}
};