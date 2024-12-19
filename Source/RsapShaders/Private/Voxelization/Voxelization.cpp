#include "Voxelization.h"
#include "RsapShaders/Public/Voxelization/Voxelization.h"
#include "PixelShaderUtils.h"
#include "MeshPassProcessor.inl"
#include "StaticMeshResources.h"
#include "RenderGraphResources.h"
#include "GlobalShader.h"
#include "RHIGPUReadback.h"



struct FTriangle
{
	FIntVector Vertex0, Vertex1, Vertex2;
	FTriangle(const FVector& InVertex0, const FVector& InVertex1, const FVector& InVertex2)
	{
		Vertex0 = FIntVector(InVertex0);
		Vertex1 = FIntVector(InVertex1);
		Vertex2 = FIntVector(InVertex2);
	}
};

struct FProjectionResult
{
	uint32 PointCount;
	uint32 ProjectedAxis;
};

DECLARE_STATS_GROUP(TEXT("Voxelization"), STATGROUP_Voxelization, STATCAT_Advanced);
DECLARE_CYCLE_STAT(TEXT("Voxelization Execute"), STAT_Voxelization_Execute, STATGROUP_Voxelization);

class RSAPSHADERS_API FVoxelization: public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FVoxelization);
	SHADER_USE_PARAMETER_STRUCT(FVoxelization, FGlobalShader);
	
	class FVoxelization_Perm_Test : SHADER_PERMUTATION_INT("TEST", 1);
	using FPermutationDomain = TShaderPermutationDomain<FVoxelization_Perm_Test>;

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER_SRV(Buffer<float3>, VertexBuffer)
		SHADER_PARAMETER_SRV(Buffer<uint>, IndexBuffer)
		SHADER_PARAMETER(uint32, NumVertices)
		SHADER_PARAMETER(uint32, NumTriangles)
		SHADER_PARAMETER(uint32, bIsIndex32Bit)
		SHADER_PARAMETER(FMatrix44f, GlobalTransformMatrix)
		SHADER_PARAMETER(FUintVector, ChunkLocation)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<FProjectionResult>, OutputBuffer)
	END_SHADER_PARAMETER_STRUCT()


	
	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
	{
		const FPermutationDomain PermutationVector(Parameters.PermutationId);
		return true;
	}

	static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
	{
		FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
		
		const FPermutationDomain PermutationVector(Parameters.PermutationId);

		// Major axis enum
		OutEnvironment.SetDefine(TEXT("AXIS_X"), 0);
		OutEnvironment.SetDefine(TEXT("AXIS_Y"), 1);
		OutEnvironment.SetDefine(TEXT("AXIS_Z"), 2);

		OutEnvironment.SetDefine(TEXT("VOXEL_SIZE"), 8);

		OutEnvironment.SetDefine(TEXT("THREADS_X"), NUM_THREADS_VOXELIZATION_X);
		OutEnvironment.SetDefine(TEXT("THREADS_Y"), NUM_THREADS_VOXELIZATION_Y);
		OutEnvironment.SetDefine(TEXT("THREADS_Z"), NUM_THREADS_VOXELIZATION_Z);
	}
};

// This will tell the engine to create the shader and where the shader entry point is.
// ShaderType | ShaderPath | Shader function name | Type
IMPLEMENT_GLOBAL_SHADER(FVoxelization, "/RsapShadersShaders/Voxelization/Voxelization.usf", "Voxelization", SF_Compute);

void FVoxelizationInterface::DispatchRenderThread(FRHICommandListImmediate& RHICmdList, const FVoxelizationDispatchParams& Params, const TFunction<void(const TArray<FUintVector3>&)>& Callback)
{
	FRDGBuilder GraphBuilder(RHICmdList);
    TShaderMapRef<FVoxelization> ComputeShader(GetGlobalShaderMap(GMaxRHIFeatureLevel), FVoxelization::FPermutationDomain());
	TArray<FRHIGPUBufferReadback*> GPUBufferReadbacks;

	// See 'FRHIBufferDesc' and 'EBufferUsageFlags' to create own buffer.
	
	FRHIViewDesc::FBufferSRV::FInitializer VertexBufferInitializer = FRHIViewDesc::CreateBufferSRV();
	VertexBufferInitializer.SetType(FRHIViewDesc::EBufferType::Typed);
	VertexBufferInitializer.SetFormat(PF_R32G32B32F);

	FRHIViewDesc::FBufferSRV::FInitializer IndexBuffer16Initializer = FRHIViewDesc::CreateBufferSRV();
	IndexBuffer16Initializer.SetType(FRHIViewDesc::EBufferType::Typed);
	IndexBuffer16Initializer.SetFormat(PF_R16_UINT);
	
	FRHIViewDesc::FBufferSRV::FInitializer IndexBuffer32Initializer = FRHIViewDesc::CreateBufferSRV();
	IndexBuffer32Initializer.SetType(FRHIViewDesc::EBufferType::Typed);
	IndexBuffer32Initializer.SetFormat(PF_R32_UINT);

	uint32 PassIdx = 0;
	for (const TObjectPtr<UStaticMeshComponent>& StaticMeshComponent : Params.ChangedSMComponents)
	{
		++PassIdx;
		const FStaticMeshRenderData* RenderData = StaticMeshComponent->GetStaticMesh()->GetRenderData();
		const FStaticMeshLODResources& LODResources = RenderData->LODResources[0];
		
		const FPositionVertexBuffer& PositionVertexBuffer = LODResources.VertexBuffers.PositionVertexBuffer;
		const FIndexBuffer& IndexBuffer = LODResources.IndexBuffer;

		FRHIShaderResourceView* VertexBufferSRV = RHICmdList.CreateShaderResourceView(PositionVertexBuffer.VertexBufferRHI, VertexBufferInitializer);
		
		const FBufferRHIRef& IndexBufferRHI = IndexBuffer.GetRHI();
		const uint32 IndexStride = IndexBuffer.IndexBufferRHI->GetStride();
		const bool bIsIndex32Bit = IndexStride == 4;
		FRHIShaderResourceView* IndexBufferSRV = RHICmdList.CreateShaderResourceView(IndexBufferRHI, bIsIndex32Bit ? IndexBuffer32Initializer : IndexBuffer16Initializer);
		
		const uint32 NumVertices = LODResources.GetNumVertices();
		const uint32 NumTriangles = LODResources.GetNumTriangles();

		// Input
		FVoxelization::FParameters* PassParameters = GraphBuilder.AllocParameters<FVoxelization::FParameters>();
		PassParameters->VertexBuffer = VertexBufferSRV;
		PassParameters->IndexBuffer = IndexBufferSRV;
		PassParameters->NumVertices = NumVertices;
		PassParameters->NumTriangles = NumTriangles;
		PassParameters->GlobalTransformMatrix = FMatrix44f(StaticMeshComponent->GetComponentTransform().ToMatrixWithScale().GetTransposed());
		PassParameters->ChunkLocation = FUintVector(0, 0, 0);

		// Output
		const FRDGBufferRef OutputBuffer = GraphBuilder.CreateBuffer(
			FRDGBufferDesc::CreateStructuredDesc(sizeof(FProjectionResult), NumTriangles),
			*FString::Printf(TEXT("Rsap.Voxelization.Output.Buffer.%i"), PassIdx)
		);
		PassParameters->OutputBuffer = GraphBuilder.CreateUAV(OutputBuffer);

		// One triangle per thread.
		FIntVector GroupCount = FComputeShaderUtils::GetGroupCount(NumTriangles, 64);
		GraphBuilder.AddPass(
			RDG_EVENT_NAME("%s", *FString::Printf(TEXT("Rsap.Voxelization.Dispatch.%i"), PassIdx)),
			PassParameters,
			ERDGPassFlags::Compute,
			[ComputeShader, PassParameters, GroupCount](FRHIComputeCommandList& RHICommandList)
			{
				FComputeShaderUtils::Dispatch(RHICommandList, ComputeShader, *PassParameters, GroupCount);
			}
		);
		
		FRHIGPUBufferReadback* GPUBufferReadback = new FRHIGPUBufferReadback(*FString::Printf(TEXT("Rsap.Voxelization.Output.Readback.%i"), PassIdx));
		AddEnqueueCopyPass(GraphBuilder, GPUBufferReadback, OutputBuffer, NumTriangles * sizeof(FProjectionResult));
		GPUBufferReadbacks.Add(GPUBufferReadback);
	}

	if(GPUBufferReadbacks.IsEmpty()) return;
	
	GraphBuilder.Execute();
	RHICmdList.BlockUntilGPUIdle();
	
	for (FRHIGPUBufferReadback* Readback : GPUBufferReadbacks)
	{
		void* TriangleData = Readback->Lock(0);
		const uint32 NumElements = Readback->GetGPUSizeBytes() / sizeof(FProjectionResult);
		FProjectionResult* Results = static_cast<FProjectionResult*>(TriangleData);

		uint32 TotalCount = 0;
		for (uint32 i = 0; i < NumElements; ++i)
		{
			const auto [PointCount, ProjectedAxis] = Results[i];
			TotalCount+=PointCount;
			UE_LOG(LogTemp, Log, TEXT("Count: %i, ProjectedAxis: %i"), PointCount, ProjectedAxis)
		}
		UE_LOG(LogTemp, Log, TEXT("Total-Count: %i,"), TotalCount)
	
		Readback->Unlock();
		delete Readback;
	}

// #include "DrawDebugHelpers.h"
// #include "Async/Async.h"
// 	
// 	TArray<FTriangle> Triangles;
// 	for (FRHIGPUBufferReadback* Readback : GPUBufferReadbacks)
// 	{
// 		void* Data = Readback->Lock(0);
// 		const uint32 NumElements = Readback->GetGPUSizeBytes() / sizeof(FTriangle); // Total uint3 entries
// 		FTriangle* TrianglesData = static_cast<FTriangle*>(Data);
//
// 		for (uint32 i = 0; i < NumElements; ++i)
// 		{
// 			Triangles.Emplace(TrianglesData[i]);
// 		}
//
// 		Readback->Unlock();
// 		delete Readback;
// 	}
//
// 	AsyncTask(ENamedThreads::GameThread, [Triangles]()
// 	{
// 		const UWorld* World = GEngine->GetWorldFromContextObjectChecked(GEditor->GetEditorWorldContext().World());
// 		if (!World) return;
//
// 		for (const auto& [Vertex0, Vertex1, Vertex2] : Triangles)
// 		{
// 			const FColor LineColor = FColor::MakeRandomColor();
// 			constexpr float Thickness = 10.0f;
// 			DrawDebugLine(World, FVector(Vertex0), FVector(Vertex1), LineColor, true, -1, 0, Thickness);
// 			DrawDebugLine(World, FVector(Vertex1), FVector(Vertex2), LineColor, true, -1, 0, Thickness);
// 			DrawDebugLine(World, FVector(Vertex2), FVector(Vertex0), LineColor, true, -1, 0, Thickness);
// 		}
// 	});

	//Callback(Vertices);
}