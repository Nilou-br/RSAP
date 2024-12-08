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
		SHADER_PARAMETER(FVector3f, FixedTransform)
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

void FVoxelizationInterface::DispatchRenderThread(FRHICommandListImmediate& RHICmdList, const FVoxelizationDispatchParams& Params) {
	FRDGBuilder GraphBuilder(RHICmdList);
        
    TShaderMapRef<FVoxelization> ComputeShader(GetGlobalShaderMap(GMaxRHIFeatureLevel), FVoxelization::FPermutationDomain());
    
    FVoxelization::FParameters* PassParameters = GraphBuilder.AllocParameters<FVoxelization::FParameters>();

	// This code just fetched the CPU side vertex-buffer, got a copy of this buffer stored on the gpu, and moved the GPU. It was unnecessary, but could be useful for when geometry was edited?
	// const FPositionVertexBuffer& VertexBuffer = Params.LODResources.VertexBuffers.PositionVertexBuffer;
	// const uint32 NumVertices = VertexBuffer.GetNumVertices();
	// void* VertexBufferData = RHICmdList.LockBuffer(VertexBuffer.VertexBufferRHI, 0, VertexBuffer.GetNumVertices() * VertexBuffer.GetStride(), RLM_WriteOnly);
	// FMemory::Memcpy(VertexBufferData, VertexBuffer.GetVertexData(), VertexBuffer.GetNumVertices() * VertexBuffer.GetStride());
	// RHICmdList.UnlockBuffer(VertexBuffer.VertexBufferRHI);
	//
	// Create an RDG buffer
	// const FRDGBufferRef VertexBufferRef = GraphBuilder.CreateBuffer(
	// 	FRDGBufferDesc::CreateBufferDesc(sizeof(FVector), NumVertices),
	// 	TEXT("VertexBuffer")
	// );
	//
	// // Upload the buffer to GPU
	// GraphBuilder.QueueBufferUpload(VertexBufferRef, VertexBufferData, VertexBuffer.GetAllocatedSize());
	// PassParameters->VertexBuffer = VertexBuffer.VertexBufferRHI;

	const FPositionVertexBuffer& PositionVertexBuffer = Params.LODResources.VertexBuffers.PositionVertexBuffer;
	FRHIBuffer* Buffer = PositionVertexBuffer.VertexBufferRHI;
	FRHIViewDesc::FBufferSRV::FInitializer Initializer = FRHIViewDesc::CreateBufferSRV();
	Initializer.SetType(FRHIViewDesc::EBufferType::Typed);
	Initializer.SetFormat(PF_R32G32B32F);
	FRHIShaderResourceView* VertexBufferSRV = RHICmdList.CreateShaderResourceView(Buffer, Initializer);
	PassParameters->VertexBuffer = VertexBufferSRV;

	const uint32 NumVertices = PositionVertexBuffer.GetNumVertices();
	PassParameters->NumVertices = NumVertices;
	PassParameters->FixedTransform = FVector3f(10.0f, 10.0f, 10.0f);

	// Create output buffer
	const FRDGBufferRef OutputBuffer = GraphBuilder.CreateBuffer(
		FRDGBufferDesc::CreateStructuredDesc(sizeof(FVector3f), NumVertices),
		TEXT("OutputBuffer")
	);
	PassParameters->OutputBuffer = GraphBuilder.CreateUAV(OutputBuffer, PF_R32G32B32F);

	// Dispatch compute shader
	FIntVector GroupCount = FComputeShaderUtils::GetGroupCount(NumVertices, 64);
	GraphBuilder.AddPass(
		RDG_EVENT_NAME("ExecuteVoxelization"),
		PassParameters,
		ERDGPassFlags::Compute,
		[ComputeShader, PassParameters, GroupCount](FRHIComputeCommandList& RHICommandList)
		{
			FComputeShaderUtils::Dispatch(RHICommandList, ComputeShader, *PassParameters, GroupCount);
		}
	);

	// Read back output buffer
	FRHIGPUBufferReadback* GPUBufferReadback = new FRHIGPUBufferReadback(TEXT("VoxelizationOutput"));
	AddEnqueueCopyPass(GraphBuilder, GPUBufferReadback, OutputBuffer, 0);

	GraphBuilder.Execute();
	RHICmdList.BlockUntilGPUIdle();

	// Fetch the data back to the CPU
	if (GPUBufferReadback->IsReady())
	{
		void* MappedData = GPUBufferReadback->Lock(0);
		const FVector3f* TransformedVertices = static_cast<FVector3f*>(MappedData);
	
		// Log the transformed vertices for verification
		for (uint32 i = 0; i < NumVertices; ++i)
		{
			UE_LOG(LogTemp, Log, TEXT("Transformed Vertex %d: %s"), i, *TransformedVertices[i].ToString());
		}
	
		GPUBufferReadback->Unlock();
	}

	//Test GPU
	void* MappedVertexBufferData = RHICmdList.LockBuffer(PositionVertexBuffer.VertexBufferRHI, 0, NumVertices * PositionVertexBuffer.GetStride(), RLM_ReadOnly);
	const FVector3f* VertexData = static_cast<FVector3f*>(MappedVertexBufferData);
	for (uint32 i = 0; i < NumVertices; ++i)
	{
		UE_LOG(LogTemp, Log, TEXT("Input Vertex %d: %s"), i, *VertexData[i].ToString());
	}
	RHICmdList.UnlockBuffer(PositionVertexBuffer.VertexBufferRHI);

	UE_LOG(LogTemp, Log, TEXT("Stride: %d"), PositionVertexBuffer.GetStride());
}