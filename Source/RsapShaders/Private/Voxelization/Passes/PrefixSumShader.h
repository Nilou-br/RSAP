#pragma once

#include <array>

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

#define NUM_TASKS_PER_THREAD 8
#define NUM_THREAD_GROUP_SIZE 128
#define NUM_GROUP_TOTAL_TASKS NUM_TASKS_PER_THREAD * NUM_THREAD_GROUP_SIZE



class RSAPSHADERS_API FSinglePrefixSumShader: public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FSinglePrefixSumShader);
	SHADER_USE_PARAMETER_STRUCT(FSinglePrefixSumShader, FGlobalShader);

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint32>,   InputBuffer)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint32>, OutPrefixSums)
		SHADER_PARAMETER(uint32, NumElements)
	END_SHADER_PARAMETER_STRUCT()
	
	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
	{
		return true;
	}

	static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
	{
		FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);

		OutEnvironment.SetDefine(TEXT("TASKS_PER_THREAD"),	NUM_TASKS_PER_THREAD);
		OutEnvironment.SetDefine(TEXT("THREAD_GROUP_SIZE"),NUM_THREAD_GROUP_SIZE);
		OutEnvironment.SetDefine(TEXT("GROUP_TOTAL_TASKS"),NUM_GROUP_TOTAL_TASKS);
	}
};
IMPLEMENT_GLOBAL_SHADER(FSinglePrefixSumShader, "/RsapShadersShaders/Voxelization/PrefixSum/SinglePrefixSum.usf", "Main", SF_Compute);


class RSAPSHADERS_API FGroupedPrefixSumShader: public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FGroupedPrefixSumShader);
	SHADER_USE_PARAMETER_STRUCT(FGroupedPrefixSumShader, FGlobalShader);

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint32>,   InputBuffer)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint32>, OutPrefixSums)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint32>, OutGroupSums)
		SHADER_PARAMETER(uint32, NumElements)
	END_SHADER_PARAMETER_STRUCT()
	
	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
	{
		return true;
	}

	static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
	{
		FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);

		OutEnvironment.SetDefine(TEXT("TASKS_PER_THREAD"), NUM_TASKS_PER_THREAD);
		OutEnvironment.SetDefine(TEXT("THREAD_GROUP_SIZE"),NUM_THREAD_GROUP_SIZE);
		OutEnvironment.SetDefine(TEXT("GROUP_TOTAL_TASKS"),NUM_GROUP_TOTAL_TASKS);
	}
};
IMPLEMENT_GLOBAL_SHADER(FGroupedPrefixSumShader, "/RsapShadersShaders/Voxelization/PrefixSum/GroupedPrefixSum.usf", "Main", SF_Compute);


class RSAPSHADERS_API FApplyGroupSumsShader: public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FApplyGroupSumsShader);
	SHADER_USE_PARAMETER_STRUCT(FApplyGroupSumsShader, FGlobalShader);

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint32>, InitialPrefixSums)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint32>, GroupPrefixSums)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint32>, OutPrefixSums)
		SHADER_PARAMETER(uint32, NumElements)
	END_SHADER_PARAMETER_STRUCT()
	
	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
	{
		return true;
	}

	static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
	{
		FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);

		OutEnvironment.SetDefine(TEXT("TASKS_PER_THREAD"), NUM_TASKS_PER_THREAD);
		OutEnvironment.SetDefine(TEXT("THREAD_GROUP_SIZE"),NUM_THREAD_GROUP_SIZE);
		OutEnvironment.SetDefine(TEXT("GROUP_TOTAL_TASKS"),NUM_GROUP_TOTAL_TASKS);
	}
};
IMPLEMENT_GLOBAL_SHADER(FApplyGroupSumsShader, "/RsapShadersShaders/Voxelization/PrefixSum/ApplyGroupSums.usf", "Main", SF_Compute);


struct FPrefixSumDebugResult
{
	std::array<FRDGBufferRef, 3> PrefixSums;
	std::array<FRDGBufferRef, 3> GroupSums;
	std::array<FRDGBufferRef, 3> AppliedSums;
};

struct FPrefixSumShaderInterface
{
	static constexpr uint32 Stride = 4;

	// Calculates the complete prefix-sum of the input-buffer.
	static void AddPass(FRDGBuilder& GraphBuilder, const FRDGBufferSRVRef InputBufferSRV, const FRDGBufferUAVRef OutputBufferUAV, const uint32 NumElements)
	{
		PerformRecursivePass(GraphBuilder, InputBufferSRV, OutputBufferUAV, NumElements);
	}

private:
	static void AddSinglePrefixSumPass(FRDGBuilder& GraphBuilder, const FRDGBufferSRVRef InputBufferSRV, const FRDGBufferUAVRef OutputBufferUAV, const uint32 NumElements, const uint32 IterationIdx)
	{
		FIntVector GroupCount = FComputeShaderUtils::GetGroupCount(NumElements, NUM_GROUP_TOTAL_TASKS);
		TShaderMapRef<FSinglePrefixSumShader> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel), FSinglePrefixSumShader::FPermutationDomain());

		FSinglePrefixSumShader::FParameters* Parameters = GraphBuilder.AllocParameters<FSinglePrefixSumShader::FParameters>();
		Parameters->InputBuffer   = InputBufferSRV;
		Parameters->OutPrefixSums = OutputBufferUAV;
		Parameters->NumElements   = NumElements;

		GraphBuilder.AddPass(
			RDG_EVENT_NAME("%s", *FString::Printf(TEXT("Rsap.Single-Prefix-Sum.%i.Dispatch"), IterationIdx)),
			Parameters,
			ERDGPassFlags::Compute,
			[Shader, Parameters, GroupCount](FRHIComputeCommandList& RHICommandList)
			{
				FComputeShaderUtils::Dispatch(RHICommandList, Shader, *Parameters, GroupCount);
			}
		);
	}
	
	struct FIntermediateResult
	{
		FRDGBufferRef PrefixSums;
		FRDGBufferRef GroupSums;
		uint32 GroupCount;
	};
	
	static void AddGroupedPrefixSumPass(FRDGBuilder& GraphBuilder, const FRDGBufferSRVRef InputBufferSRV, const FRDGBufferUAVRef OutputBufferUAV, const FRDGBufferUAVRef GroupSumsBufferUAV, const uint32 NumElements, const uint32 IterationIdx)
	{
		TShaderMapRef<FGroupedPrefixSumShader> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel), FGroupedPrefixSumShader::FPermutationDomain());

		FGroupedPrefixSumShader::FParameters* Parameters = GraphBuilder.AllocParameters<FGroupedPrefixSumShader::FParameters>();
		Parameters->InputBuffer   = InputBufferSRV;
		Parameters->OutPrefixSums = OutputBufferUAV;
		Parameters->OutGroupSums  = GroupSumsBufferUAV;
		Parameters->NumElements   = NumElements;

		FIntVector GroupCount = FComputeShaderUtils::GetGroupCount(NumElements, NUM_GROUP_TOTAL_TASKS);
		GraphBuilder.AddPass(
			RDG_EVENT_NAME("%s", *FString::Printf(TEXT("Rsap.Grouped-Prefix-Sum.%i.Dispatch"), IterationIdx)),
			Parameters,
			ERDGPassFlags::Compute,
			[Shader, Parameters, GroupCount](FRHIComputeCommandList& RHICommandList)
			{
				FComputeShaderUtils::Dispatch(RHICommandList, Shader, *Parameters, GroupCount);
			}
		);
	}
	
	static void ApplyGroupSumsPass(FRDGBuilder& GraphBuilder,  const FRDGBufferSRVRef InputBufferSRV, const FRDGBufferSRVRef GroupSumsBufferSRV, const FRDGBufferUAVRef OutputBufferUAV, const uint32 NumElements, const uint32 IterationIdx)
	{
		FIntVector GroupCount = FComputeShaderUtils::GetGroupCount(NumElements, NUM_GROUP_TOTAL_TASKS);
		TShaderMapRef<FApplyGroupSumsShader> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel), FApplyGroupSumsShader::FPermutationDomain());

		FApplyGroupSumsShader::FParameters* Parameters = GraphBuilder.AllocParameters<FApplyGroupSumsShader::FParameters>();
		Parameters->InitialPrefixSums = InputBufferSRV;
		Parameters->GroupPrefixSums   = GroupSumsBufferSRV;
		Parameters->OutPrefixSums     = OutputBufferUAV;
		Parameters->NumElements		  = NumElements;

		GraphBuilder.AddPass(
			RDG_EVENT_NAME("%s", *FString::Printf(TEXT("Rsap.Voxelization.ApplyGroupSums.%i"), IterationIdx)),
			Parameters,
			ERDGPassFlags::Compute,
			[Shader, Parameters, GroupCount](FRHIComputeCommandList& RHICmdList)
			{
				FComputeShaderUtils::Dispatch(RHICmdList, Shader, *Parameters, GroupCount);
			}
		);
	}

	static void PerformRecursivePass(FRDGBuilder& GraphBuilder, const FRDGBufferSRVRef InputBufferSRV, const FRDGBufferUAVRef OutputBufferUAV, const uint32 NumElements, const uint32 IterationIdx = 0)
	{
		if(NumElements <= NUM_GROUP_TOTAL_TASKS)
		{
			// The input fits in a single thread group, so a single pass would make it a complete prefix sum.
			AddSinglePrefixSumPass(GraphBuilder, InputBufferSRV, OutputBufferUAV, NumElements, IterationIdx);
			return;
		}

		// We require more than one thread group, so do a grouped-prefix-sum pass which also returns the group-sums.
		const FIntVector GroupCount = FComputeShaderUtils::GetGroupCount(NumElements, NUM_GROUP_TOTAL_TASKS);
		const FRDGBufferRef GroupSums = GraphBuilder.CreateBuffer(
			FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), GroupCount.X),
			*FString::Printf(TEXT("Rsap.Group-Sums-Buffer"))
		);
		const FRDGBufferSRVRef GroupSumsBufferSRV = GraphBuilder.CreateSRV(GroupSums, PF_R32_UINT);
		const FRDGBufferUAVRef GroupSumsBufferUAV = GraphBuilder.CreateUAV(GroupSums, PF_R32_UINT);
		
		AddGroupedPrefixSumPass(GraphBuilder, InputBufferSRV, OutputBufferUAV, GroupSumsBufferUAV, NumElements, IterationIdx);

		// Recursive method to also turn the group-sums into a prefix-sum.
		PerformRecursivePass(GraphBuilder, GroupSumsBufferSRV, GroupSumsBufferUAV, GroupCount.X, IterationIdx+1);

		// Apply this to each element in the corresponding group in the initial prefix-sums.
		ApplyGroupSumsPass(GraphBuilder, InputBufferSRV, GroupSumsBufferSRV, OutputBufferUAV, NumElements, IterationIdx);
	}
};