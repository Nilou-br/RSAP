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

class RSAPSHADERS_API FPrefixSumShader: public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FPrefixSumShader);
	SHADER_USE_PARAMETER_STRUCT(FPrefixSumShader, FGlobalShader);

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint32>,   InputBuffer)
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
	static FRDGBufferRef AddPass(FRDGBuilder& GraphBuilder, const FRDGBufferRef InputBuffer, const uint32 NumTriangles, const uint32 PassIdx)
	{
		TShaderMapRef<FPrefixSumShader> PrefixSumShader(GetGlobalShaderMap(GMaxRHIFeatureLevel), FPrefixSumShader::FPermutationDomain());
		const FRDGBufferSRVRef InputBufferSRV = GraphBuilder.CreateSRV(InputBuffer, PF_R32_UINT);
		FIntVector GroupCount = FComputeShaderUtils::GetGroupCount(NumTriangles, NUM_THREAD_GROUP_SIZE);
		
		const FRDGBufferRef PrefixSumsBuffer = GraphBuilder.CreateBuffer(
			FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), NumTriangles),
			*FString::Printf(TEXT("Rsap.PrefixSumShader.%i.PrefixSums"), PassIdx)
		);
		
		const FRDGBufferRef GroupSumsBuffer = GraphBuilder.CreateBuffer(
			FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), GroupCount.X),
			*FString::Printf(TEXT("Rsap.PrefixSumShader.%i.GroupSums"), PassIdx)
		);
		
		FPrefixSumShader::FParameters* Parameters = GraphBuilder.AllocParameters<FPrefixSumShader::FParameters>();
		Parameters->InputBuffer	  = InputBufferSRV;
		Parameters->OutPrefixSums = GraphBuilder.CreateUAV(PrefixSumsBuffer, PF_R32_UINT);
		Parameters->OutGroupSums  = GraphBuilder.CreateUAV(GroupSumsBuffer, PF_R32_UINT);
		Parameters->NumTriangles  = NumTriangles;

		GraphBuilder.AddPass(
			RDG_EVENT_NAME("%s", *FString::Printf(TEXT("Rsap.BlockSumShader.%i.Dispatch"), PassIdx)),
			Parameters,
			ERDGPassFlags::Compute,
			[PrefixSumShader, Parameters, GroupCount](FRHIComputeCommandList& RHICommandList)
			{
				FComputeShaderUtils::Dispatch(RHICommandList, PrefixSumShader, *Parameters, GroupCount);
			}
		);

		return PrefixSumsBuffer;
	}
};