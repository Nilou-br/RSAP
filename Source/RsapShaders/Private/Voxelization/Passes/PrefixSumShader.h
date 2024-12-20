#pragma once

#include "CoreMinimal.h"
#include "RHICommandList.h"
#include "RenderGraphBuilder.h"
#include "ShaderParameterUtils.h"
#include "RHIStaticStates.h"
#include "Shader.h"
#include "RHI.h"
#include "GlobalShader.h"
#include "RenderGraphUtils.h"
#include "ShaderParameterStruct.h"
#include "UniformBuffer.h"
#include "RHICommandList.h"
#include "ShaderCompilerCore.h"
#include "EngineDefines.h"
#include "RendererInterface.h"
#include "RenderResource.h"
#include "RenderGraphResources.h"

#define NUM_TASKS_PER_THREAD 1
#define NUM_THREAD_GROUP_SIZE 32

struct FPrefixSumResult
{
	uint32 Sum;
	uint32 ProjectedAxis;
};

class RSAPSHADERS_API FBlockSumShader: public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FBlockSumShader);
	SHADER_USE_PARAMETER_STRUCT(FBlockSumShader, FGlobalShader);

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FPrefixSumResult>, InputBuffer)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint32>, OutPrefixSums)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint32>, OutGroupSums)
		SHADER_PARAMETER(uint32, NumTriangles)
	END_SHADER_PARAMETER_STRUCT()
	
	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
	{
		return true;
	}

	static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
	{
		FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);

		OutEnvironment.SetDefine(TEXT("TASKS_PER_THREAD"), NUM_TASKS_PER_THREAD);
		OutEnvironment.SetDefine(TEXT("THREAD_GROUP_SIZE"), NUM_THREAD_GROUP_SIZE);
	}
};
IMPLEMENT_GLOBAL_SHADER(FBlockSumShader, "/RsapShadersShaders/Voxelization/BlockSum.usf", "Main", SF_Compute);

class RSAPSHADERS_API FPrefixSumShader: public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FPrefixSumShader);
	SHADER_USE_PARAMETER_STRUCT(FPrefixSumShader, FGlobalShader);

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FPrefixSumResult>, InputBuffer)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint32>, OutPrefixSums)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint32>, OutGroupSums)
		SHADER_PARAMETER(uint32, NumTriangles)
	END_SHADER_PARAMETER_STRUCT()
	
	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
	{
		return true;
	}

	static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
	{
		FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);

		OutEnvironment.SetDefine(TEXT("TASKS_PER_THREAD"), NUM_TASKS_PER_THREAD);
		OutEnvironment.SetDefine(TEXT("THREAD_GROUP_SIZE"), NUM_THREAD_GROUP_SIZE);
	}
};
IMPLEMENT_GLOBAL_SHADER(FPrefixSumShader, "/RsapShadersShaders/Voxelization/PrefixSum.usf", "Main", SF_Compute);

struct FPrefixSumShaderInterface
{
	static FRDGBufferRef AddPass(FRDGBuilder& GraphBuilder, const FRDGBufferRef ProjectionResultBuffer, const uint32 NumTriangles, const uint32 PassIdx)
	{
		TShaderMapRef<FBlockSumShader> BlockSumShader(GetGlobalShaderMap(GMaxRHIFeatureLevel), FBlockSumShader::FPermutationDomain());
		TShaderMapRef<FPrefixSumShader> PrefixSumShader(GetGlobalShaderMap(GMaxRHIFeatureLevel), FPrefixSumShader::FPermutationDomain());
		const FRDGBufferSRVRef ProjectionResultSRV = GraphBuilder.CreateSRV(ProjectionResultBuffer, PF_R32G32_UINT);
		FIntVector GroupCount = FComputeShaderUtils::GetGroupCount(NumTriangles, NUM_THREAD_GROUP_SIZE);
		
		const FRDGBufferRef PrefixSumBuffer = GraphBuilder.CreateBuffer(
			FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), NumTriangles),
			*FString::Printf(TEXT("Rsap.PrefixSumShader.%i.PrefixSums"), PassIdx)
		);
		
		const FRDGBufferRef GroupSumsBuffer = GraphBuilder.CreateBuffer(
			FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), GroupCount.X),
			*FString::Printf(TEXT("Rsap.PrefixSumShader.%i.GroupSums"), PassIdx)
		);
		
		FBlockSumShader::FParameters* FirstPassParameters = GraphBuilder.AllocParameters<FBlockSumShader::FParameters>();
		FirstPassParameters->InputBuffer	= ProjectionResultSRV;
		FirstPassParameters->OutPrefixSums	= GraphBuilder.CreateUAV(PrefixSumBuffer, PF_R32_UINT);
		FirstPassParameters->OutGroupSums	= GraphBuilder.CreateUAV(GroupSumsBuffer, PF_R32_UINT);
		FirstPassParameters->NumTriangles	= NumTriangles;

		GraphBuilder.AddPass(
			RDG_EVENT_NAME("%s", *FString::Printf(TEXT("Rsap.BlockSumShader.%i.Dispatch"), PassIdx)),
			FirstPassParameters,
			ERDGPassFlags::Compute,
			[BlockSumShader, FirstPassParameters, GroupCount](FRHIComputeCommandList& RHICommandList)
			{
				FComputeShaderUtils::Dispatch(RHICommandList, BlockSumShader, *FirstPassParameters, GroupCount);
			}
		);

		// Second pass to calculate the prefix sums.
		// const FRDGBufferRef PrefixSumResultBuffer = GraphBuilder.CreateBuffer(
		// 	FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), NumTriangles),
		// 	*FString::Printf(TEXT("Rsap.PrefixSumShader.%i.Output"), PassIdx)
		// );
		// FPrefixSumShader::FParameters* SumPrefixPassParameters = GraphBuilder.AllocParameters<FPrefixSumShader::FParameters>();
		// SumPrefixPassParameters->InputBuffer   = ProjectionResultSRV;
		// SumPrefixPassParameters->BlockSums	   = GraphBuilder.CreateSRV(BlockSumsBuffer, PF_R32_UINT);
		// SumPrefixPassParameters->OutputBuffer  = GraphBuilder.CreateUAV(PrefixSumResultBuffer, PF_R32_UINT);
		// SumPrefixPassParameters->NumTriangles  = NumTriangles;
		//
		// GraphBuilder.AddPass(
		// 	RDG_EVENT_NAME("%s", *FString::Printf(TEXT("Rsap.PrefixSumShader.%i.Dispatch"), PassIdx)),
		// 	SumPrefixPassParameters,
		// 	ERDGPassFlags::Compute,
		// 	[PrefixSumShader, SumPrefixPassParameters, GroupCount](FRHIComputeCommandList& RHICommandList)
		// 	{
		// 		FComputeShaderUtils::Dispatch(RHICommandList, PrefixSumShader, *SumPrefixPassParameters, GroupCount);
		// 	}
		// );

		return GroupSumsBuffer;
	}
};