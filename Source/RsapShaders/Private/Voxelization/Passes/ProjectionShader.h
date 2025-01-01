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

#define NUM_THREADS_PROJECTION_X 64
#define NUM_THREADS_PROJECTION_Y 1
#define NUM_THREADS_PROJECTION_Z 1

struct FProjectionResult
{
	uint32 PointCount;
	uint32 ProjectedAxis;
};

class RSAPSHADERS_API FProjectionShader: public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FProjectionShader);
	SHADER_USE_PARAMETER_STRUCT(FProjectionShader, FGlobalShader);

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER_SRV(Buffer<float3>, VertexBuffer)
		SHADER_PARAMETER_SRV(Buffer<uint>, IndexBuffer)
		SHADER_PARAMETER(uint32, NumTriangles)
		SHADER_PARAMETER(uint32, bIsIndex32Bit)
		SHADER_PARAMETER(FMatrix44f, GlobalTransformMatrix)
		SHADER_PARAMETER(FUintVector, ChunkLocation)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint32>, CountsBuffer)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint32>, ProjectedAxisBuffer)
	END_SHADER_PARAMETER_STRUCT()
	
	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
	{
		return true;
	}

	static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
	{
		FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
		
		OutEnvironment.SetDefine(TEXT("AXIS_X"), 0);
		OutEnvironment.SetDefine(TEXT("AXIS_Y"), 1);
		OutEnvironment.SetDefine(TEXT("AXIS_Z"), 2);

		OutEnvironment.SetDefine(TEXT("VOXEL_SIZE"), 8);

		OutEnvironment.SetDefine(TEXT("THREADS_X"), NUM_THREADS_PROJECTION_X);
		OutEnvironment.SetDefine(TEXT("THREADS_Y"), NUM_THREADS_PROJECTION_Y);
		OutEnvironment.SetDefine(TEXT("THREADS_Z"), NUM_THREADS_PROJECTION_Z);
	}
};

IMPLEMENT_GLOBAL_SHADER(FProjectionShader, "/RsapShadersShaders/Voxelization/Projection.usf", "Main", SF_Compute);

struct FProjectionShaderResult
{
	FRDGBufferRef CountsBuffer; // 2D points per triangle's AABB.
	FRDGBufferRef AxisBuffer;   // Major axis projected to.
};

struct FProjectionShaderInterface
{
	static constexpr uint32 Stride = 4;
	
	// Counts how many 2D points intersect with a triangle projected onto it's major axis.
	static void AddPass(FRDGBuilder& GraphBuilder, FRHIShaderResourceView* VertexBufferSRV, FRHIShaderResourceView* IndexBufferSRV, const FRDGBufferUAVRef CountsBufferUAV, const FRDGBufferUAVRef AxisBufferUAV, const uint32 NumTriangles, const FMatrix44f& ComponentTransform)
	{
		TShaderMapRef<FProjectionShader> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel), FProjectionShader::FPermutationDomain());
		
		FProjectionShader::FParameters* PassParameters = GraphBuilder.AllocParameters<FProjectionShader::FParameters>();
		PassParameters->VertexBuffer = VertexBufferSRV;
		PassParameters->IndexBuffer = IndexBufferSRV;
		PassParameters->NumTriangles = NumTriangles;
		PassParameters->GlobalTransformMatrix = ComponentTransform;
		PassParameters->ChunkLocation = FUintVector(0, 0, 0);
		PassParameters->CountsBuffer = CountsBufferUAV;
		PassParameters->ProjectedAxisBuffer = AxisBufferUAV;
		
		FIntVector GroupCount = FComputeShaderUtils::GetGroupCount(NumTriangles, NUM_THREADS_PROJECTION_X);
		GraphBuilder.AddPass(
			RDG_EVENT_NAME("%s", *FString::Printf(TEXT("Rsap.ProjectionShader.Dispatch"))),
			PassParameters,
			ERDGPassFlags::Compute,
			[Shader, PassParameters, GroupCount](FRHIComputeCommandList& RHICommandList)
			{
				FComputeShaderUtils::Dispatch(RHICommandList, Shader, *PassParameters, GroupCount);
			}
		);
	}
};