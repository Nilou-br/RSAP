#include "Voxelization.h"
#include "RsapShaders/Public/Voxelization/Voxelization.h"
#include "PixelShaderUtils.h"
#include "MeshPassProcessor.inl"
#include "StaticMeshResources.h"
#include "RenderGraphResources.h"
#include "GlobalShader.h"
#include "RHIGPUReadback.h"



struct FAABB2D
{
	FIntVector2 Min;
	FIntVector2 Max;

	FAABB2D(const FIntVector2& InMin, const FIntVector2& InMax)
		: Min(InMin), Max(InMax)
	{}

	uint32 GetWidth()  const { return Max.X - Min.X; }
	uint32 GetHeight() const { return Max.Y - Min.Y; }

	void ClampToGrid(const int32 GridSize)
	{
		Min.X = (Min.X / GridSize) * GridSize;
		Min.Y = (Min.Y / GridSize) * GridSize;
		Max.X = ((Max.X + GridSize - 1) / GridSize) * GridSize;
		Max.Y = ((Max.Y + GridSize - 1) / GridSize) * GridSize;
	}

	uint32 GetPointCount(const int32 GridSize = 8)
	{
		// Clamp to the grid size first
		ClampToGrid(GridSize);

		// Calculate points in both dimensions
		int PointsX = GetWidth() / GridSize;
		int PointsY = GetHeight() / GridSize;

		// Return total points
		return static_cast<uint32>(PointsX * PointsY);
	}
};

struct F2DProjectedVertex
{
	int32 X = 0;
	int32 Y = 0;
	uint32 MajorAxisValue = 0;
};

struct FTriangle3D
{
	FIntVector Vertex0;
	FIntVector Vertex1;
	FIntVector Vertex2;
};

struct F2DProjectedTriangle
{
	F2DProjectedVertex Vertex0;
	F2DProjectedVertex Vertex1;
	F2DProjectedVertex Vertex2;
	uint32 MajorAxis;

	FAABB2D GetAABB()
	{
		const int32 MinX = std::min({Vertex0.X, Vertex1.X, Vertex2.X});
		const int32 MinY = std::min({Vertex0.Y, Vertex1.Y, Vertex2.Y});
		const int32 MaxX = std::max({Vertex0.X, Vertex1.X, Vertex2.X});
		const int32 MaxY = std::max({Vertex0.Y, Vertex1.Y, Vertex2.Y});
		return FAABB2D(FIntVector2(MinX, MinY), FIntVector2(MaxX, MaxY));
	}

	FTriangle3D GetTriangle3D() const
	{
		FTriangle3D Triangle3D;
		switch (MajorAxis)
		{
		case 0:
			Triangle3D.Vertex0.X = Vertex0.MajorAxisValue;
			Triangle3D.Vertex1.X = Vertex1.MajorAxisValue;
			Triangle3D.Vertex2.X = Vertex2.MajorAxisValue;
			Triangle3D.Vertex0.Y = Vertex0.X;
			Triangle3D.Vertex1.Y = Vertex1.X;
			Triangle3D.Vertex2.Y = Vertex2.X;
			Triangle3D.Vertex0.Z = Vertex0.Y;
			Triangle3D.Vertex1.Z = Vertex1.Y;
			Triangle3D.Vertex2.Z = Vertex2.Y;
			break;
		case 1:
			Triangle3D.Vertex0.X = Vertex0.X;
			Triangle3D.Vertex1.X = Vertex1.X;
			Triangle3D.Vertex2.X = Vertex2.X;
			Triangle3D.Vertex0.Y = Vertex0.MajorAxisValue;
			Triangle3D.Vertex1.Y = Vertex1.MajorAxisValue;
			Triangle3D.Vertex2.Y = Vertex2.MajorAxisValue;
			Triangle3D.Vertex0.Z = Vertex0.Y;
			Triangle3D.Vertex1.Z = Vertex1.Y;
			Triangle3D.Vertex2.Z = Vertex2.Y;
			break;
		case 2:
			Triangle3D.Vertex0.X = Vertex0.X;
			Triangle3D.Vertex1.X = Vertex1.X;
			Triangle3D.Vertex2.X = Vertex2.X;
			Triangle3D.Vertex0.Y = Vertex0.Y;
			Triangle3D.Vertex1.Y = Vertex1.Y;
			Triangle3D.Vertex2.Y = Vertex2.Y;
			Triangle3D.Vertex0.Z = Vertex0.MajorAxisValue;
			Triangle3D.Vertex1.Z = Vertex1.MajorAxisValue;
			Triangle3D.Vertex2.Z = Vertex2.MajorAxisValue;
			break;
		default: break;
		}
		return Triangle3D;
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
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<F2DProjectedTriangle>, OutputBuffer)
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
			FRDGBufferDesc::CreateStructuredDesc(sizeof(F2DProjectedTriangle), NumTriangles),
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
		AddEnqueueCopyPass(GraphBuilder, GPUBufferReadback, OutputBuffer, NumTriangles * sizeof(F2DProjectedTriangle));
		GPUBufferReadbacks.Add(GPUBufferReadback);
	}

	if(GPUBufferReadbacks.IsEmpty()) return;
	
	GraphBuilder.Execute();
	RHICmdList.BlockUntilGPUIdle();

#include "DrawDebugHelpers.h"
#include "Async/Async.h"
	
	// Fetch the data back to the CPU
	//TArray<FUintVector3> Vertices;
	TArray<FTriangle3D> Triangles3D;
	TArray<FAABB2D> BoundsList;
	for (FRHIGPUBufferReadback* Readback : GPUBufferReadbacks)
	{
		void* TriangleData = Readback->Lock(0);
		const uint32 NumElements = Readback->GetGPUSizeBytes() / sizeof(F2DProjectedTriangle); // Total uint3 entries
		F2DProjectedTriangle* Triangles = static_cast<F2DProjectedTriangle*>(TriangleData);

		for (uint32 i = 0; i < NumElements; ++i)
		{
			Triangles3D.Emplace(Triangles[i].GetTriangle3D());
			FAABB2D Bounds = Triangles[i].GetAABB();
			Bounds.ClampToGrid(8);
			BoundsList.Emplace(Bounds);
			UE_LOG(LogTemp, Log, TEXT("Count: %i"), Bounds.GetPointCount())
		}

		Readback->Unlock();
		delete Readback;
	}

	AsyncTask(ENamedThreads::GameThread, [Triangles3D, BoundsList]()
   {
	   const UWorld* World = GEngine->GetWorldFromContextObjectChecked(GEditor->GetEditorWorldContext().World());
	   if (!World) return;

	   // for (const auto& [Vertex0, Vertex1, Vertex2] : Triangles3D)
	   // {
		  //  const FColor LineColor = FColor::MakeRandomColor();
		  //  constexpr float Thickness = 10.0f;
		  //  DrawDebugLine(World, FVector(Vertex0), FVector(Vertex1), LineColor, true, -1, 0, Thickness);
		  //  DrawDebugLine(World, FVector(Vertex1), FVector(Vertex2), LineColor, true, -1, 0, Thickness);
		  //  DrawDebugLine(World, FVector(Vertex2), FVector(Vertex0), LineColor, true, -1, 0, Thickness);
	   // }

	   for (const auto& Bounds : BoundsList)
	   {
			const FColor LineColor = FColor::MakeRandomColor();
	   		constexpr float Thickness = 10.0f;
			FVector TopLeft(Bounds.Min.X, Bounds.Min.Y, 0);
			FVector TopRight(Bounds.Max.X, Bounds.Min.Y, 0);
			FVector BottomLeft(Bounds.Min.X, Bounds.Max.Y, 0);
			FVector BottomRight(Bounds.Max.X, Bounds.Max.Y, 0);

			DrawDebugLine(World, TopLeft, TopRight, LineColor, true, -1, 0, Thickness);
			DrawDebugLine(World, TopLeft, BottomLeft, LineColor, true, -1, 0, Thickness);
			DrawDebugLine(World, TopRight, BottomRight, LineColor, true, -1, 0, Thickness);
			DrawDebugLine(World, BottomLeft, BottomRight, LineColor, true, -1, 0, Thickness);
	   }
		
   });

	//Callback(Vertices);
}