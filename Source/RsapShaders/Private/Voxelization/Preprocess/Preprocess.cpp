#include "Preprocess.h"
#include "RsapShaders/Public/Voxelization/Preprocess.h"
#include "PixelShaderUtils.h"
#include "MeshPassProcessor.inl"
#include "StaticMeshResources.h"
#include "RenderGraphResources.h"
#include "GlobalShader.h"
#include "RHIGPUReadback.h"
#include "Passes/PrefixSumShader.h"
#include "Passes/ProjectionShader.h"
#include "Rsap/NavMesh/NavmeshShaderProxy.h"
#include "GPUSort.h"

FVoxelizationPreprocessInterface::FOnVoxelizationPreprocessComplete FVoxelizationPreprocessInterface::OnVoxelizationPreprocessComplete;



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

class RSAPSHADERS_API FVoxelizationPreprocess: public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FVoxelizationPreprocess);
	SHADER_USE_PARAMETER_STRUCT(FVoxelizationPreprocess, FGlobalShader);
	
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

void FVoxelizationPreprocessInterface::DispatchRenderThread(FRHICommandListImmediate& RHICmdList, FRsapNavmeshShaderProxy& NavmeshShaderProxy)
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

	
	// // These buffers are used for all meshes to store their prefix-sum and total-sum.
	// const FRDGBufferRef SharedSumBuffer = GraphBuilder.CreateBuffer(
	// 	FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), NumTriangles),
	// 	*FString::Printf(TEXT("Rsap.Voxelization.SharedBuffer"))
	// );
	// FRDGBufferSRVRef SharedSumBufferSRV = GraphBuilder.CreateSRV(SharedBuffer, PF_R32_UINT);
	// FRDGBufferUAVRef SharedSumBufferUAV = GraphBuilder.CreateUAV(SharedBuffer, PF_R32_UINT);
	//
	//
	//
	// for (const TObjectPtr<UStaticMeshComponent>& StaticMeshComponent : NavmeshShaderProxy.PreprocessBatch)
	// {
	// 	// Get necessary static-mesh render-data
	// 	const FStaticMeshRenderData* RenderData = StaticMeshComponent->GetStaticMesh()->GetRenderData();
	// 	const FStaticMeshLODResources& LODResources = RenderData->LODResources[0];
	// 	const FMatrix44f ComponentTransform(StaticMeshComponent->GetComponentTransform().ToMatrixWithScale().GetTransposed());
	// 	
	// 	const FPositionVertexBuffer& PositionVertexBuffer = LODResources.VertexBuffers.PositionVertexBuffer;
	// 	const FBufferRHIRef& IndexBufferRHI = LODResources.IndexBuffer.GetRHI();
	// 	const bool bIsIndexBuffer32Bit = IndexBufferRHI->GetStride() == 4;
	// 	const uint32 NumTriangles = LODResources.GetNumTriangles();
	//
	// 	FRHIShaderResourceView* VertexBufferSRV = PositionVertexBuffer.GetSRV();
	// 	FRHIShaderResourceView* IndexBufferSRV= RHICmdList.CreateShaderResourceView(IndexBufferRHI, bIsIndexBuffer32Bit ? IndexBuffer32Initializer : IndexBuffer16Initializer);
	// 	
	//
	// 	// Get/init our render-data for this static-mesh-component.
	// 	FRsapMeshComponentRenderData ComponentRenderData = NavmeshShaderProxy.MeshComponentsRenderData.FindOrAdd(StaticMeshComponent);
	// 	FRDGBufferDesc BufferDesc = FRDGBufferDesc::CreateStructuredDesc(4, NumTriangles);
	// 	ComponentRenderData.PrefixSumBuffer = AllocatePooledBuffer(BufferDesc, TEXT("PrefixSumBuffer"));
	// 	
	// 	const FRDGBufferRef ProjectedAxisBuffer = GraphBuilder.CreateBuffer(
	// 		FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), NumTriangles),
	// 		*FString::Printf(TEXT("Rsap.Voxelization.AxisBuffer"))
	// 	);
	// 	FRDGBufferSRVRef AxisBufferSRV = GraphBuilder.CreateSRV(ProjectedAxisBuffer, PF_R32_UINT);
	// 	FRDGBufferUAVRef AxisBufferUAV = GraphBuilder.CreateUAV(ProjectedAxisBuffer, PF_R32_UINT);
	// 	
	// 	FProjectionShaderInterface::AddPass(GraphBuilder, VertexBufferSRV, IndexBufferSRV, SharedBufferUAV, AxisBufferUAV, NumTriangles, ComponentTransform);
	// 	FPrefixSumShaderInterface::AddPass(GraphBuilder, SharedBufferSRV, SharedBufferUAV, NumTriangles);
	// 	
	// 	FRHIGPUBufferReadback* CountsResultReadback = new FRHIGPUBufferReadback(*FString::Printf(TEXT("Rsap.PrefixSum.Output.Readback")));
	// 	AddEnqueueCopyPass(GraphBuilder, CountsResultReadback, SharedBuffer, NumTriangles * sizeof(uint32));
	// 	CountsResults.Add(CountsResultReadback);
	// 	
	// 	FRHIResourceCreateInfo ExternalBufferInfo(TEXT("PersistentBuffer"));
	// 	FBufferRHIRef ExternalBuffer = FRHICommandList::CreateStructuredBuffer(
	// 		sizeof(uint32), 
	// 		NumTriangles * sizeof(uint32), 
	// 		BUF_ShaderResource | BUF_UnorderedAccess, 
	// 		ExternalBufferInfo
	// 	);
	// }
	
	GraphBuilder.Execute();
	RHICmdList.BlockUntilGPUIdle();

	OnVoxelizationPreprocessComplete.Execute();

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
