#include "Voxelization.h"
#include "RsapShaders/Public/Voxelization/Voxelization.h"
#include "PixelShaderUtils.h"
#include "MeshPassProcessor.inl"
#include "StaticMeshResources.h"
#include "DynamicMeshBuilder.h"
#include "RenderGraphResources.h"
#include "GlobalShader.h"
#include "UnifiedBuffer.h"
#include "CanvasTypes.h"
#include "MeshDrawShaderBindings.h"
#include "RHIGPUReadback.h"
#include "MeshPassUtils.h"
#include "MaterialShader.h"

DECLARE_STATS_GROUP(TEXT("Voxelization"), STATGROUP_Voxelization, STATCAT_Advanced);
DECLARE_CYCLE_STAT(TEXT("Voxelization Execute"), STAT_Voxelization_Execute, STATGROUP_Voxelization);

// This class carries our parameter declarations and acts as the bridge between cpp and HLSL.
class RSAPSHADERS_API FVoxelization: public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FVoxelization);
	SHADER_USE_PARAMETER_STRUCT(FVoxelization, FGlobalShader);
	
	class FVoxelization_Perm_Test : SHADER_PERMUTATION_INT("TEST", 1);
	using FPermutationDomain = TShaderPermutationDomain<FVoxelization_Perm_Test>;

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER_SRV(Buffer<float3>, VertexBuffer)
		SHADER_PARAMETER(uint32, NumVertices)
		SHADER_PARAMETER(FMatrix44f, TransformMatrix)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float3>, OutputBuffer)
	END_SHADER_PARAMETER_STRUCT()

public:
	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
	{
		const FPermutationDomain PermutationVector(Parameters.PermutationId);
		return true;
	}

	static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
	{
		FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);

		const FPermutationDomain PermutationVector(Parameters.PermutationId);

		/*
		* Here you define constants that can be used statically in the shader code.
		* Example:
		*/
		// OutEnvironment.SetDefine(TEXT("MY_CUSTOM_CONST"), TEXT("1"));

		/*
		* These defines are used in the thread count section of our shader
		*/
		OutEnvironment.SetDefine(TEXT("THREADS_X"), NUM_THREADS_VOXELIZATION_X);
		OutEnvironment.SetDefine(TEXT("THREADS_Y"), NUM_THREADS_VOXELIZATION_Y);
		OutEnvironment.SetDefine(TEXT("THREADS_Z"), NUM_THREADS_VOXELIZATION_Z);

		// This shader must support typed UAV load and we are testing if it is supported at runtime using RHIIsTypedUAVLoadSupported
		//OutEnvironment.CompilerFlags.Add(CFLAG_AllowTypedUAVLoads);

		// FForwardLightingParameters::ModifyCompilationEnvironment(Parameters.Platform, OutEnvironment);
	}
private:
};

// This will tell the engine to create the shader and where the shader entry point is.
// ShaderType | ShaderPath | Shader function name | Type
IMPLEMENT_GLOBAL_SHADER(FVoxelization, "/RsapShadersShaders/Voxelization/Voxelization.usf", "Voxelization", SF_Compute);

void FVoxelizationInterface::DispatchRenderThread(FRHICommandListImmediate& RHICmdList, const FVoxelizationDispatchParams& Params, const TFunction<void(const TArray<FVector3f>&)>& Callback)
{
	FRDGBuilder GraphBuilder(RHICmdList);
    TShaderMapRef<FVoxelization> ComputeShader(GetGlobalShaderMap(GMaxRHIFeatureLevel), FVoxelization::FPermutationDomain());
	TArray<FRHIGPUBufferReadback*> GPUBufferReadbacks;

	// The VertexBufferSRV initializer.
	FRHIViewDesc::FBufferSRV::FInitializer SRVInitializer = FRHIViewDesc::CreateBufferSRV();
	SRVInitializer.SetType(FRHIViewDesc::EBufferType::Typed);
	SRVInitializer.SetFormat(PF_R32G32B32F);

	uint32 Idx = 0; // For giving each pass a unique identifier.
	for (const TObjectPtr<UStaticMeshComponent>& StaticMeshComponent : Params.ChangedSMComponents)
	{
		++Idx;
		const FStaticMeshRenderData* RenderData = StaticMeshComponent->GetStaticMesh()->GetRenderData();
		const FStaticMeshLODResources& LODResources = RenderData->LODResources[0];
		const FPositionVertexBuffer& PositionVertexBuffer = LODResources.VertexBuffers.PositionVertexBuffer;
		const uint32 NumVertices = PositionVertexBuffer.GetNumVertices();
		FRHIShaderResourceView* VertexBufferSRV = RHICmdList.CreateShaderResourceView(PositionVertexBuffer.VertexBufferRHI, SRVInitializer);

		FVoxelization::FParameters* PassParameters = GraphBuilder.AllocParameters<FVoxelization::FParameters>();
		PassParameters->VertexBuffer = VertexBufferSRV;
		PassParameters->NumVertices = NumVertices;
		PassParameters->TransformMatrix = FMatrix44f(StaticMeshComponent->GetComponentTransform().ToMatrixWithScale().GetTransposed());

		// Create output buffer
		const FRDGBufferRef OutputBuffer = GraphBuilder.CreateBuffer(
			FRDGBufferDesc::CreateStructuredDesc(sizeof(FVector3f), NumVertices),
			*FString::Printf(TEXT("Rsap.Voxelization.Output.Buffer.%i"), Idx)
		);
		PassParameters->OutputBuffer = GraphBuilder.CreateUAV(OutputBuffer, PF_R32G32B32F);

		FIntVector GroupCount = FComputeShaderUtils::GetGroupCount(NumVertices, 64);
		GraphBuilder.AddPass(
			RDG_EVENT_NAME("%s", *FString::Printf(TEXT("Rsap.Voxelization.Dispatch.%i"), Idx)),
			PassParameters,
			ERDGPassFlags::Compute,
			[ComputeShader, PassParameters, GroupCount](FRHIComputeCommandList& RHICommandList)
			{
				FComputeShaderUtils::Dispatch(RHICommandList, ComputeShader, *PassParameters, GroupCount);
			}
		);
		
		FRHIGPUBufferReadback* GPUBufferReadback = new FRHIGPUBufferReadback(*FString::Printf(TEXT("Rsap.Voxelization.Output.Readback.%i"), Idx));
		AddEnqueueCopyPass(GraphBuilder, GPUBufferReadback, OutputBuffer, NumVertices*PositionVertexBuffer.GetStride());
		GPUBufferReadbacks.Add(GPUBufferReadback);
	}

	if(GPUBufferReadbacks.IsEmpty()) return;
	
	GraphBuilder.Execute();
	RHICmdList.BlockUntilGPUIdle();

	TArray<FVector3f> Vertices;

	// Fetch the data back to the CPU
	for (FRHIGPUBufferReadback* Readback : GPUBufferReadbacks)
	{
		void* MappedData = Readback->Lock(0);
		const uint32 NumElements = Readback->GetGPUSizeBytes() / sizeof(FVector3f);
		const FVector3f* TransformedVertices = static_cast<FVector3f*>(MappedData);
		for (uint32 i = 0; i < NumElements; ++i) Vertices.Emplace(TransformedVertices[i]);
		Readback->Unlock();
		delete Readback;
	}

	Callback(Vertices);
}