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

// This will tell the engine to create the shader and where the shader entry point is.
// ShaderType | ShaderPath | Shader function name | Type
//IMPLEMENT_GLOBAL_SHADER(FVoxelization, "/RsapShadersShaders/Voxelization/Voxelization.usf", "Voxelization", SF_Compute);

void FVoxelizationInterface::DispatchRenderThread(FRHICommandListImmediate& RHICmdList, const FVoxelizationDispatchParams& Params, const TFunction<void(const TArray<FUintVector3>&)>& Callback)
{
	FRDGBuilder GraphBuilder(RHICmdList);
	TArray<FRHIGPUBufferReadback*> ProjectionResults;
	TArray<FRHIGPUBufferReadback*> PrefixSumResults;
	TArray<FRHIGPUBufferReadback*> GroupSumResults;
	TArray<FRHIGPUBufferReadback*> AppliedSumResults;
	TArray<FRHIGPUBufferReadback*> CompletePrefixSumResults;
	
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
		const FMatrix44f ComponentTransform(StaticMeshComponent->GetComponentTransform().ToMatrixWithScale().GetTransposed());
		
		const FPositionVertexBuffer& PositionVertexBuffer = LODResources.VertexBuffers.PositionVertexBuffer;
		const FIndexBuffer& IndexBuffer = LODResources.IndexBuffer;

		FRHIShaderResourceView* VertexBufferSRV = RHICmdList.CreateShaderResourceView(PositionVertexBuffer.VertexBufferRHI, VertexBufferInitializer);
		
		const FBufferRHIRef& IndexBufferRHI = IndexBuffer.GetRHI();
		const uint32 IndexStride = IndexBuffer.IndexBufferRHI->GetStride();
		const bool bIsIndex32Bit = IndexStride == 4;
		FRHIShaderResourceView* IndexBufferSRV = RHICmdList.CreateShaderResourceView(IndexBufferRHI, bIsIndex32Bit ? IndexBuffer32Initializer : IndexBuffer16Initializer);
		
		const uint32 NumVertices = LODResources.GetNumVertices();
		const uint32 NumTriangles = LODResources.GetNumTriangles();
		
		const FProjectionShaderResult ProjectionResult = FProjectionShaderInterface::AddPass(GraphBuilder, VertexBufferSRV, IndexBufferSRV, NumVertices, NumTriangles, ComponentTransform, PassIdx);
		FRHIGPUBufferReadback* ProjectionResultReadback = new FRHIGPUBufferReadback(*FString::Printf(TEXT("Rsap.Projection.Output.Readback.%i"), PassIdx));
		AddEnqueueCopyPass(GraphBuilder, ProjectionResultReadback, ProjectionResult.CountsBuffer, NumTriangles * sizeof(uint32));
		ProjectionResults.Add(ProjectionResultReadback);

		std::array<uint32, 3> Divider = {1, 1024, 1024};
		std::array<uint32, 3> Divider2 = {1024, 1024, 1};
		FPrefixSumDebugResult DebugResult;
		const FRDGBufferRef PrefixSumResultBuffer = FPrefixSumShaderInterface::AddPass(GraphBuilder, ProjectionResult.CountsBuffer, NumTriangles, DebugResult);
		// for (int i = 0; i < 2; ++i)
		// {
		// 	const uint32 GroupSize = FMath::DivideAndRoundUp(NumTriangles, Divider[i]);
		// 	
		// 	FRHIGPUBufferReadback* PrefixSumResultReadback = new FRHIGPUBufferReadback(*FString::Printf(TEXT("Rsap.PrefixSum.Output.Readback.%i"), PassIdx));
		// 	AddEnqueueCopyPass(GraphBuilder, PrefixSumResultReadback, DebugResult.PrefixSums[i], GroupSize * sizeof(uint32));
		// 	PrefixSumResults.Add(PrefixSumResultReadback);
		//
		// 	if(i==0) continue;
		// 	//
		// 	// FRHIGPUBufferReadback* AppliedSumResultReadback = new FRHIGPUBufferReadback(*FString::Printf(TEXT("Rsap.PrefixSum.Output.Readback.%i"), PassIdx));
		// 	// AddEnqueueCopyPass(GraphBuilder, AppliedSumResultReadback, DebugResult.AppliedSums[i], NumTriangles / (1 << Shift) * sizeof(uint32));
		// 	// AppliedSumResults.Add(AppliedSumResultReadback);
		// 	//
		// 	const uint32 GroupSize2 = FMath::DivideAndRoundUp(NumTriangles, Divider2[i]);
		// 	FRHIGPUBufferReadback* GroupSumResultReadback = new FRHIGPUBufferReadback(*FString::Printf(TEXT("Rsap.PrefixSum.Output.Readback.%i"), PassIdx));
		// 	AddEnqueueCopyPass(GraphBuilder, GroupSumResultReadback, DebugResult.GroupSums[i], GroupSize2 * sizeof(uint32));
		// 	GroupSumResults.Add(GroupSumResultReadback);
		// }

		FRHIGPUBufferReadback* CompleteResultReadback = new FRHIGPUBufferReadback(*FString::Printf(TEXT("Rsap.PrefixSum.Output.Readback.%i"), PassIdx));
		AddEnqueueCopyPass(GraphBuilder, CompleteResultReadback, PrefixSumResultBuffer, NumTriangles * sizeof(uint32));
		CompletePrefixSumResults.Add(CompleteResultReadback);
	}
	
	GraphBuilder.Execute();
	RHICmdList.BlockUntilGPUIdle();
	
	for (FRHIGPUBufferReadback* Readback : ProjectionResults)
	{
		void* ProjectionResultData = Readback->Lock(0);
		const uint32 NumElements = Readback->GetGPUSizeBytes() / sizeof(uint32);
		uint32* Buffer = static_cast<uint32*>(ProjectionResultData);

		uint32 TotalCount = 0;
		for (uint32 i = 0; i < NumElements; ++i)
		{
			const uint32 Count = Buffer[i];
			TotalCount+=Count;
			UE_LOG(LogTemp, Log, TEXT("Index: %i, Count: %i"), i, Count)
		}
		UE_LOG(LogTemp, Log, TEXT("Total-Count: %i,"), TotalCount)
	
		Readback->Unlock();
		delete Readback;
	}

	uint32 PrefixSumIteration = 0;
	for (FRHIGPUBufferReadback* Readback : PrefixSumResults)
	{
		void* PrefixSumResultsData = Readback->Lock(0);
		const uint32 NumElements = Readback->GetGPUSizeBytes() / sizeof(uint32);
		uint32* Buffer = static_cast<uint32*>(PrefixSumResultsData);
	
		UE_LOG(LogTemp, Log, TEXT("Prefix-sum-iteration: %i"), PrefixSumIteration)
		for (uint32 i = 0; i < NumElements; ++i)
		{
			UE_LOG(LogTemp, Log, TEXT("Index: %i, Prefix-sum: %i"), i, Buffer[i])
		}
	
		Readback->Unlock();
		delete Readback;
	
		++PrefixSumIteration;
	}

	for (FRHIGPUBufferReadback* Readback : GroupSumResults)
	{
		void* PrefixSumResultsData = Readback->Lock(0);
		const uint32 NumElements = Readback->GetGPUSizeBytes() / sizeof(uint32);
		uint32* Buffer = static_cast<uint32*>(PrefixSumResultsData);
	
		for (uint32 i = 0; i < NumElements; ++i)
		{
			UE_LOG(LogTemp, Log, TEXT("Index: %i, Group-sum: %i"), i, Buffer[i])
		}
	
		Readback->Unlock();
		delete Readback;
	}

	// for (FRHIGPUBufferReadback* Readback : AppliedSumResults)
	// {
	// 	void* PrefixSumResultsData = Readback->Lock(0);
	// 	const uint32 NumElements = Readback->GetGPUSizeBytes() / sizeof(uint32);
	// 	uint32* Buffer = static_cast<uint32*>(PrefixSumResultsData);
	//
	// 	for (uint32 i = 0; i < NumElements; ++i)
	// 	{
	// 		UE_LOG(LogTemp, Log, TEXT("Index: %i, Applied-sum: %i"), i, Buffer[i])
	// 	}
	//
	// 	Readback->Unlock();
	// 	delete Readback;
	// }

	for (FRHIGPUBufferReadback* Readback : CompletePrefixSumResults)
	{
		void* PrefixSumResultsData = Readback->Lock(0);
		const uint32 NumElements = Readback->GetGPUSizeBytes() / sizeof(uint32);
		uint32* Buffer = static_cast<uint32*>(PrefixSumResultsData);
	
		for (uint32 i = 0; i < NumElements; ++i)
		{
			UE_LOG(LogTemp, Log, TEXT("Index: %i, Complete-prefix-sum: %i"), i, Buffer[i])
		}
	
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