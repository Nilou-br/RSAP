// Copyright Melvin Brink 2023. All Rights Reserved.

#include "RSAP_Editor/Public/NavMesh/Tasks/RsapEditorUpdateTask.h"
#include "RSAP/Math/Bounds.h"



/**
 * Updates the navmesh using the given list of bound-pairs which indicates the areas that needs to be updated.
 */
uint32 FRsapEditorUpdateTask::Run() // todo: updater runs when starting editor for some reason.
{
	// For-each actor:
	// For-each chunk the actor's bounds are in, sorted by the chunk's morton:
	// For-each node within the chunk, sorted by the node's morton:
	// Get the node and its relations to update.

	return 0;
}