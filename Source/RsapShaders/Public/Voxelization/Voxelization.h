#pragma once

#include "CoreMinimal.h"
#include "GenericPlatform/GenericPlatformMisc.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Materials/MaterialRenderProxy.h"



struct RSAPSHADERS_API FVoxelizationDispatchParams
{
	int X, Y, Z;
	
	int Input[2] = {0, 0};
	int Output = 0;
	
	FVoxelizationDispatchParams(const int InX, const int InY, const int InZ)
		: X(InX), Y(InY), Z(InZ)
	{}
};

// This is a public interface that we define so outside code can invoke our compute shader.
class RSAPSHADERS_API FVoxelizationInterface {
public:
	// Executes this shader on the render thread
	static void DispatchRenderThread(
		FRHICommandListImmediate& RHICmdList,
		FVoxelizationDispatchParams Params,
		TFunction<void(int OutputVal)> AsyncCallback
	);

	// Executes this shader on the render thread from the game thread via EnqueueRenderThreadCommand
	static void DispatchGameThread(
		FVoxelizationDispatchParams Params,
		TFunction<void(int OutputVal)> AsyncCallback
	)
	{
		ENQUEUE_RENDER_COMMAND(SceneDrawCompletion)(
		[Params, AsyncCallback](FRHICommandListImmediate& RHICmdList)
		{
			DispatchRenderThread(RHICmdList, Params, AsyncCallback);
		});
	}

	// Dispatches this shader. Can be called from any thread
	static void Dispatch(
		const FVoxelizationDispatchParams& Params,
		const TFunction<void(int OutputVal)>& AsyncCallback
	)
	{
		if (IsInRenderingThread())
		{
			DispatchRenderThread(GetImmediateCommandList_ForRenderCommand(), Params, AsyncCallback);
		}
		else
		{
			DispatchGameThread(Params, AsyncCallback);
		}
	}
};