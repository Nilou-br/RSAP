#include "Voxelization.h"
#include "RsapShaders/Public/Voxelization/Voxelization.h"
#include "PixelShaderUtils.h"
#include "MeshPassProcessor.inl"
#include "StaticMeshResources.h"
#include "RenderGraphResources.h"
#include "GlobalShader.h"
#include "RHIGPUReadback.h"
#include "Passes/PrefixSumShader.h"
#include "Passes/ProjectionShader.h"
#include "RsapShared/Public/Rsap/Math/Bounds.h"


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
		return true;
	}

	static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
	{
		FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
		
		OutEnvironment.SetDefine(TEXT("AXIS_X"), 0);
		OutEnvironment.SetDefine(TEXT("AXIS_Y"), 1);
		OutEnvironment.SetDefine(TEXT("AXIS_Z"), 2);

		OutEnvironment.SetDefine(TEXT("VOXEL_SIZE"), 8);

		OutEnvironment.SetDefine(TEXT("THREADS_X"), NUM_THREADS_VOXELIZATION_X);
		OutEnvironment.SetDefine(TEXT("THREADS_Y"), NUM_THREADS_VOXELIZATION_Y);
		OutEnvironment.SetDefine(TEXT("THREADS_Z"), NUM_THREADS_VOXELIZATION_Z);
	}
};

void FVoxelizationInterface::DispatchRenderThread(FRHICommandListImmediate& RHICmdList, const FVoxelizationDispatchParams& Params, const TFunction<void(const TArray<FUintVector3>&)>& Callback)
{
	TRefCountPtr<FRDGPooledBuffer> SharedBufferPooled;
	TRefCountPtr<FRDGPooledBuffer> ProjectedAxisBufferPooled;
	
	FRDGBuilder GraphBuilder(RHICmdList);
	
	TArray<FRHIGPUBufferReadback*> CountsResults;

	FRHIViewDesc::FBufferSRV::FInitializer IndexBuffer16Initializer = FRHIViewDesc::CreateBufferSRV();
	IndexBuffer16Initializer.SetType(FRHIViewDesc::EBufferType::Typed);
	IndexBuffer16Initializer.SetFormat(PF_R16_UINT);
	
	FRHIViewDesc::FBufferSRV::FInitializer IndexBuffer32Initializer = FRHIViewDesc::CreateBufferSRV();
	IndexBuffer32Initializer.SetType(FRHIViewDesc::EBufferType::Typed);
	IndexBuffer32Initializer.SetFormat(PF_R32_UINT);

	
	
	for (const TObjectPtr<UStaticMeshComponent>& StaticMeshComponent : Params.StaticMeshComponents)
	{
		const FStaticMeshRenderData* RenderData = StaticMeshComponent->GetStaticMesh()->GetRenderData();
		const FStaticMeshLODResources& LODResources = RenderData->LODResources[0];
		const FMatrix44f ComponentTransform(StaticMeshComponent->GetComponentTransform().ToMatrixWithScale().GetTransposed());
		
		const FPositionVertexBuffer& PositionVertexBuffer = LODResources.VertexBuffers.PositionVertexBuffer;
		const FBufferRHIRef& IndexBufferRHI = LODResources.IndexBuffer.GetRHI();
		const bool bIsIndexBuffer32Bit = IndexBufferRHI->GetStride() == 4;
		const uint32 NumTriangles = LODResources.GetNumTriangles();

		FRHIShaderResourceView* VertexBufferSRV = PositionVertexBuffer.GetSRV();
		FRHIShaderResourceView* IndexBufferSRV= RHICmdList.CreateShaderResourceView(IndexBufferRHI, bIsIndexBuffer32Bit ? IndexBuffer32Initializer : IndexBuffer16Initializer);

		
		// Create buffers and resource/access-views

		// This buffer is reused between multiple passes. The num-elements and stride stay the same between them, and the RDG handles dependencies.
		const FRDGBufferRef SharedBuffer = GraphBuilder.CreateBuffer(
			FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), NumTriangles),
			*FString::Printf(TEXT("Rsap.Voxelization.SharedBuffer"))
		);
		FRDGBufferSRVRef SharedBufferSRV = GraphBuilder.CreateSRV(SharedBuffer, PF_R32_UINT);
		FRDGBufferUAVRef SharedBufferUAV = GraphBuilder.CreateUAV(SharedBuffer, PF_R32_UINT);

		
		const FRDGBufferRef ProjectedAxisBuffer = GraphBuilder.CreateBuffer(
			FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), NumTriangles),
			*FString::Printf(TEXT("Rsap.Voxelization.AxisBuffer"))
		);
		FRDGBufferSRVRef AxisBufferSRV = GraphBuilder.CreateSRV(ProjectedAxisBuffer, PF_R32_UINT);
		FRDGBufferUAVRef AxisBufferUAV = GraphBuilder.CreateUAV(ProjectedAxisBuffer, PF_R32_UINT);
		

		// Add the passes in order of the dependencies between them.
		
		FProjectionShaderInterface::AddPass(GraphBuilder, VertexBufferSRV, IndexBufferSRV, SharedBufferUAV, AxisBufferUAV, NumTriangles, ComponentTransform);
		FPrefixSumShaderInterface::AddPass(GraphBuilder, SharedBufferSRV, SharedBufferUAV, NumTriangles);

		// Get uint32 total-count from SharedBuffer ...

		// Create new RDG buffer using this value ...

		// Third shader pass that uses this buffer.
		
		FRHIGPUBufferReadback* CountsResultReadback = new FRHIGPUBufferReadback(*FString::Printf(TEXT("Rsap.PrefixSum.Output.Readback")));
		AddEnqueueCopyPass(GraphBuilder, CountsResultReadback, SharedBuffer, NumTriangles * sizeof(uint32));
		CountsResults.Add(CountsResultReadback);
	}
	
	GraphBuilder.Execute();
	RHICmdList.BlockUntilGPUIdle();


	GraphBuilder.EndEventScope();

	// FRHIResourceCreateInfo NavMeshBufferCreateInfo(TEXT("RsapNavMeshBuffer"));
	// constexpr EBufferUsageFlags NavMeshBufferUsageFlags = BUF_UnorderedAccess | BUF_ShaderResource;
	// FBufferRHIRef NavMeshBufferRHI = RHICmdList.CreateStructuredBuffer(4, 100, NavMeshBufferUsageFlags, NavMeshBufferCreateInfo);
	
	// Readback test.
	uint32 PrefixSumIteration = 0;
	for (FRHIGPUBufferReadback* Readback : CountsResults)
	{
		void* PrefixSumResultsData = Readback->Lock(0);
		const uint32 NumElements = Readback->GetGPUSizeBytes() / sizeof(uint32);
		uint32* Buffer = static_cast<uint32*>(PrefixSumResultsData);

		uint32 TotalCount = 0;
		for (uint32 i = 0; i < NumElements; ++i)
		{
			const uint32 Count = Buffer[i];
			// TotalCount+=Count;
			UE_LOG(LogTemp, Log, TEXT("Index: %i, Count: %i"), i, Count)
		}
		UE_LOG(LogTemp, Log, TEXT("Total-Count: %i,"), TotalCount)
	
		Readback->Unlock();
		delete Readback;
	
		++PrefixSumIteration;
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
