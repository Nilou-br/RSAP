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
IMPLEMENT_GLOBAL_SHADER(FSinglePrefixSumShader, "/RsapShadersShaders/Voxelization/SinglePrefixSum.usf", "Main", SF_Compute);


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
IMPLEMENT_GLOBAL_SHADER(FGroupedPrefixSumShader, "/RsapShadersShaders/Voxelization/GroupedPrefixSum.usf", "Main", SF_Compute);


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
IMPLEMENT_GLOBAL_SHADER(FApplyGroupSumsShader, "/RsapShadersShaders/Voxelization/ApplyGroupSums.usf", "Main", SF_Compute);


struct FPrefixSumDebugResult
{
	std::array<FRDGBufferRef, 3> PrefixSums;
	std::array<FRDGBufferRef, 3> GroupSums;
	std::array<FRDGBufferRef, 3> AppliedSums;
};

struct FPrefixSumShaderInterface
{
	static FRDGBufferRef AddPass(FRDGBuilder& GraphBuilder, const FRDGBufferRef InputBuffer, const uint32 NumElements, FPrefixSumDebugResult& DebugResult)
	{
		return PerformRecursivePass(GraphBuilder, InputBuffer, NumElements, DebugResult);
	}

private:
	static FRDGBufferRef AddSinglePrefixSumPass(FRDGBuilder& GraphBuilder, const FRDGBufferRef InputBuffer, const uint32 NumElements, const uint32 IterationIdx)
	{
		FIntVector GroupCount = FComputeShaderUtils::GetGroupCount(NumElements, NUM_GROUP_TOTAL_TASKS);
		TShaderMapRef<FSinglePrefixSumShader> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel), FSinglePrefixSumShader::FPermutationDomain());
		const FRDGBufferSRVRef InputBufferSRV = GraphBuilder.CreateSRV(InputBuffer, PF_R32_UINT);

		const FRDGBufferRef PrefixSums = GraphBuilder.CreateBuffer(
			FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), NumElements),
			*FString::Printf(TEXT("Rsap.Prefix-Sums-Buffer"))
		);

		FSinglePrefixSumShader::FParameters* Parameters = GraphBuilder.AllocParameters<FSinglePrefixSumShader::FParameters>();
		Parameters->InputBuffer   = InputBufferSRV;
		Parameters->OutPrefixSums = GraphBuilder.CreateUAV(PrefixSums, PF_R32_UINT);
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

		return PrefixSums;
	}
	
	struct FIntermediateResult
	{
		FRDGBufferRef PrefixSums;
		FRDGBufferRef GroupSums;
		uint32 GroupCount;
	};
	
	static FIntermediateResult AddGroupedPrefixSumPass(FRDGBuilder& GraphBuilder, const FRDGBufferRef InputBuffer, const uint32 NumElements, const uint32 IterationIdx)
	{
		FIntermediateResult Result;
		FIntVector GroupCount = FComputeShaderUtils::GetGroupCount(NumElements, NUM_GROUP_TOTAL_TASKS);
		Result.GroupCount = GroupCount.X;
		TShaderMapRef<FGroupedPrefixSumShader> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel), FGroupedPrefixSumShader::FPermutationDomain());
		const FRDGBufferSRVRef InputBufferSRV = GraphBuilder.CreateSRV(InputBuffer, PF_R32_UINT);

		Result.PrefixSums = GraphBuilder.CreateBuffer(
			FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), NumElements),
			*FString::Printf(TEXT("Rsap.Prefix-Sums-Buffer"))
		);
		Result.GroupSums = GraphBuilder.CreateBuffer(
			FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), GroupCount.X),
			*FString::Printf(TEXT("Rsap.Group-Sums-Buffer"))
		);

		FGroupedPrefixSumShader::FParameters* Parameters = GraphBuilder.AllocParameters<FGroupedPrefixSumShader::FParameters>();
		Parameters->InputBuffer   = InputBufferSRV;
		Parameters->OutPrefixSums = GraphBuilder.CreateUAV(Result.PrefixSums,	PF_R32_UINT);
		Parameters->OutGroupSums  = GraphBuilder.CreateUAV(Result.GroupSums,	PF_R32_UINT);
		Parameters->NumElements   = NumElements;

		GraphBuilder.AddPass(
			RDG_EVENT_NAME("%s", *FString::Printf(TEXT("Rsap.Grouped-Prefix-Sum.%i.Dispatch"), IterationIdx)),
			Parameters,
			ERDGPassFlags::Compute,
			[Shader, Parameters, GroupCount](FRHIComputeCommandList& RHICommandList)
			{
				FComputeShaderUtils::Dispatch(RHICommandList, Shader, *Parameters, GroupCount);
			}
		);

		return Result;
	}
	
	static FRDGBufferRef ApplyGroupSumsPass(FRDGBuilder& GraphBuilder, const FRDGBufferRef& InitialPrefixSums, const uint32 NumElements, const FRDGBufferRef& GroupPrefixSum, const uint32 IterationIdx)
	{
		FIntVector GroupCount = FComputeShaderUtils::GetGroupCount(NumElements, NUM_THREAD_GROUP_SIZE);
		TShaderMapRef<FApplyGroupSumsShader> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel), FApplyGroupSumsShader::FPermutationDomain());

		const FRDGBufferRef AppliedSums = GraphBuilder.CreateBuffer(
			FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), NumElements),
			*FString::Printf(TEXT("Rsap.Applied-Sums-Buffer"))
		);

		FApplyGroupSumsShader::FParameters* Parameters = GraphBuilder.AllocParameters<FApplyGroupSumsShader::FParameters>();
		Parameters->InitialPrefixSums = GraphBuilder.CreateSRV(InitialPrefixSums,	PF_R32_UINT);
		Parameters->GroupPrefixSums   = GraphBuilder.CreateSRV(GroupPrefixSum,		PF_R32_UINT);
		Parameters->OutPrefixSums     = GraphBuilder.CreateUAV(AppliedSums,	PF_R32_UINT);
		Parameters->NumElements		  = NumElements;

		GraphBuilder.AddPass(
			RDG_EVENT_NAME("%s", *FString::Printf(TEXT("Rsap.Apply-Group-Sums.%i.Dispatch"), IterationIdx)),
			Parameters,
			ERDGPassFlags::Compute,
			[Shader, Parameters, GroupCount](FRHIComputeCommandList& RHICmdList)
			{
				FComputeShaderUtils::Dispatch(RHICmdList, Shader, *Parameters, GroupCount);
			}
		);

		return AppliedSums;
	}

	static FRDGBufferRef PerformRecursivePass(FRDGBuilder& GraphBuilder, const FRDGBufferRef InputBuffer, const uint32 NumElements, FPrefixSumDebugResult& DebugResult, const uint32 IterationIdx = 0)
	{
		if(NumElements <= NUM_GROUP_TOTAL_TASKS)
		{
			// The input fits in a single thread group, so a single pass would make it a complete prefix sum.
			const auto Result = AddSinglePrefixSumPass(GraphBuilder, InputBuffer, NumElements, IterationIdx);
			DebugResult.PrefixSums[IterationIdx] = Result;
			return Result;
		}

		// We require more than one thread group, so add a grouped-prefix-sum pass where which also returns a group-sums buffer.
		const auto [PrefixSums, GroupSums, GroupCount] = AddGroupedPrefixSumPass(GraphBuilder, InputBuffer, NumElements, IterationIdx);
		DebugResult.PrefixSums[IterationIdx] = PrefixSums;
		DebugResult.GroupSums[IterationIdx] = GroupSums;

		// Recursively create another prefix-sum for the group-sums.
		const FRDGBufferRef GroupPrefixSums = PerformRecursivePass(GraphBuilder, GroupSums, GroupCount, DebugResult, IterationIdx+1);

		// Apply the group-sums to each element in the corresponding group in the initial prefix-sums.
		return ApplyGroupSumsPass(GraphBuilder, PrefixSums, NumElements, GroupPrefixSums, IterationIdx);
	}
};