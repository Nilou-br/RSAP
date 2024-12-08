#pragma once

#include "CoreMinimal.h"
#include "GenericPlatform/GenericPlatformMisc.h"



struct RSAPSHADERS_API FVoxelizationDispatchParams
{
	const FStaticMeshLODResources& LODResources;
	
	explicit FVoxelizationDispatchParams(const FStaticMeshLODResources& InLODResources)
		: LODResources(InLODResources)
	{}
};

// This is a public interface that we define so outside code can invoke our compute shader.
class RSAPSHADERS_API FVoxelizationInterface {
public:
	// Executes this shader on the render thread
	static void DispatchRenderThread(FRHICommandListImmediate& RHICmdList,const FVoxelizationDispatchParams& Params);

	// Executes this shader on the render thread from the game thread via EnqueueRenderThreadCommand
	static void DispatchGameThread(FVoxelizationDispatchParams Params)
	{
		ENQUEUE_RENDER_COMMAND(SceneDrawCompletion)(
		[Params](FRHICommandListImmediate& RHICmdList)
		{
			DispatchRenderThread(RHICmdList, Params);
		});
	}

	// Dispatches this shader. Can be called from any thread
	static void Dispatch(
		const FVoxelizationDispatchParams& Params
	)
	{
		if (IsInRenderingThread())
		{
			DispatchRenderThread(GetImmediateCommandList_ForRenderCommand(), Params);
		}
		else
		{
			DispatchGameThread(Params);
		}
	}
};