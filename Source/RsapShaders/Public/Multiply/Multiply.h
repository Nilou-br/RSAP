#pragma once

#include "GenericPlatform/GenericPlatformMisc.h"



struct RSAPSHADERS_API FMultiplyShaderDispatchParams
{
	const int X = 1, Y = 1, Z = 1;

	int Input[2];
	int Output = 0;
	
	FMultiplyShaderDispatchParams(const int Lhs, const int Rhs)
		: Input{Lhs, Rhs}
	{}
};

// This is an interface we define so outside code can invoke our compute shader.
class RSAPSHADERS_API FMultiplyShaderInterface
{
	// Executes this shader on the render thread.
	static void DispatchRenderThread(
		FRHICommandListImmediate& RHICmdList,
		FMultiplyShaderDispatchParams Params,
		TFunction<void(int OutputVal)> AsyncCallback
	);

	// Executes this shader on the render thread from the game thread via EnqueueRenderThreadCommand.
	static void DispatchGameThread(FMultiplyShaderDispatchParams Params, TFunction<void(int OutputVal)> AsyncCallback)
	{
		ENQUEUE_RENDER_COMMAND(SceneDrawCompletion)(
		[Params, AsyncCallback](FRHICommandListImmediate& RHICmdList)
		{
			DispatchRenderThread(RHICmdList, Params, AsyncCallback);
		});
	}

public:
	// Dispatches this shader. Can be called from any thread.
	static void Dispatch(const FMultiplyShaderDispatchParams& Params, const TFunction<void(int OutputVal)>& AsyncCallback)
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