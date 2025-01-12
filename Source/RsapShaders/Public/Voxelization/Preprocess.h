#pragma once

#include "CoreMinimal.h"
#include "GenericPlatform/GenericPlatformMisc.h"

class FRsapNavmeshShaderProxy;



struct RSAPSHADERS_API FVoxelizationPreprocessDispatchParams
{
	TArray<TObjectPtr<UStaticMeshComponent>> StaticMeshComponents;
	
	explicit FVoxelizationPreprocessDispatchParams(TArray<TObjectPtr<UStaticMeshComponent>>&& Components)
		: StaticMeshComponents(MoveTemp(Components))
	{}
};

// This is a public interface that we define so outside code can invoke our compute shader.
class RSAPSHADERS_API FVoxelizationPreprocessInterface
{
	DECLARE_DELEGATE(FOnVoxelizationPreprocessComplete)
	
public:
	static FOnVoxelizationPreprocessComplete OnVoxelizationPreprocessComplete;
	
	// Executes this shader on the render thread
	static void DispatchRenderThread(FRHICommandListImmediate& RHICmdList, FRsapNavmeshShaderProxy& NavmeshShaderProxy);

	// Executes this shader on the render thread from the game thread via EnqueueRenderThreadCommand
	static void DispatchGameThread(FRsapNavmeshShaderProxy& NavmeshShaderProxy)
	{
		ENQUEUE_RENDER_COMMAND(SceneDrawCompletion)(
		[&](FRHICommandListImmediate& RHICmdList)
		{
			DispatchRenderThread(RHICmdList, NavmeshShaderProxy);
		});
	}

	// Dispatches this shader. Can be called from any thread
	static void Dispatch(FRsapNavmeshShaderProxy& NavmeshShaderProxy)
	{
		if (IsInRenderingThread())
		{
			DispatchRenderThread(GetImmediateCommandList_ForRenderCommand(), NavmeshShaderProxy);
		}
		else
		{
			DispatchGameThread(NavmeshShaderProxy);
		}
	}
};