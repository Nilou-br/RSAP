#pragma once

#include "CoreMinimal.h"
#include "GenericPlatform/GenericPlatformMisc.h"
#include "RsapShared/Public/Rsap/Math/Vectors.h"


struct RSAPSHADERS_API FVoxelizationDispatchParams
{
	TArray<TObjectPtr<UStaticMeshComponent>> StaticMeshComponents;
	
	explicit FVoxelizationDispatchParams(TArray<TObjectPtr<UStaticMeshComponent>>&& Components)
		: StaticMeshComponents(MoveTemp(Components))
	{}
};

// This is a public interface that we define so outside code can invoke our compute shader.
class RSAPSHADERS_API FVoxelizationInterface
{

	static void StartnitialGraph();
	static void StartVoxelGraph();
	
public:
	// Executes this shader on the render thread
	static void DispatchRenderThread(FRHICommandListImmediate& RHICmdList, const FVoxelizationDispatchParams& Params, const TFunction<void(const TArray<FUintVector3>&)>& Callback);

	// Executes this shader on the render thread from the game thread via EnqueueRenderThreadCommand
	static void DispatchGameThread(FVoxelizationDispatchParams Params, TFunction<void(const TArray<FUintVector3>&)> Callback)
	{
		ENQUEUE_RENDER_COMMAND(SceneDrawCompletion)(
		[Params, Callback](FRHICommandListImmediate& RHICmdList)
		{
			DispatchRenderThread(RHICmdList, Params, Callback);
		});
	}

	// Dispatches this shader. Can be called from any thread
	static void Dispatch(const FVoxelizationDispatchParams& Params, const TFunction<void(const TArray<FUintVector3>&)>& Callback)
	{
		if (IsInRenderingThread())
		{
			DispatchRenderThread(GetImmediateCommandList_ForRenderCommand(), Params, Callback);
		}
		else
		{
			DispatchGameThread(Params, Callback);
		}
	}
};