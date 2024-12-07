#include "Multiply.h"
#include "RsapShaders/Public/Multiply/Multiply.h"
#include "GlobalShader.h"
#include "MeshPassProcessor.inl"
#include "RenderGraphResources.h"
#include "RHIGPUReadback.h"
#include "StaticMeshResources.h"
#include "UnifiedBuffer.h"

DECLARE_STATS_GROUP(TEXT("MultiplyShader"), STATGROUP_MultiplyShader, STATCAT_Advanced);
DECLARE_CYCLE_STAT(TEXT("MultiplyShader Execute"), STAT_MultiplyShader_Execute, STATGROUP_MultiplyShader);



// This class carries our parameter declarations and acts as the bridge between cpp and HLSL.
class RSAPSHADERS_API FMultiplyShader: public FGlobalShader
{
public:

	DECLARE_GLOBAL_SHADER(FMultiplyShader);
	SHADER_USE_PARAMETER_STRUCT(FMultiplyShader, FGlobalShader);

	class FMultiplyShader_Perm_Test : SHADER_PERMUTATION_INT("TEST", 1);
	using FPermutationDomain = TShaderPermutationDomain<FMultiplyShader_Perm_Test>;
	
	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
	
		/*
		* Here's where you define one or more of the input parameters for your shader.
		* Some examples:
		*/
		// SHADER_PARAMETER(uint32, MyUint32) // On the shader side: uint32 MyUint32;
		// SHADER_PARAMETER(FVector3f, MyVector) // On the shader side: float3 MyVector;

		// SHADER_PARAMETER_TEXTURE(Texture2D, MyTexture) // On the shader side: Texture2D<float4> MyTexture; (float4 should be whatever you expect each pixel in the texture to be, in this case float4(R,G,B,A) for 4 channels)
		// SHADER_PARAMETER_SAMPLER(SamplerState, MyTextureSampler) // On the shader side: SamplerState MySampler; // CPP side: TStaticSamplerState<ESamplerFilter::SF_Bilinear>::GetRHI();

		// SHADER_PARAMETER_ARRAY(float, MyFloatArray, [3]) // On the shader side: float MyFloatArray[3];

		// SHADER_PARAMETER_UAV(RWTexture2D<FVector4f>, MyTextureUAV) // On the shader side: RWTexture2D<float4> MyTextureUAV;
		// SHADER_PARAMETER_UAV(RWStructuredBuffer<FMyCustomStruct>, MyCustomStructs) // On the shader side: RWStructuredBuffer<FMyCustomStruct> MyCustomStructs;
		// SHADER_PARAMETER_UAV(RWBuffer<FMyCustomStruct>, MyCustomStructs) // On the shader side: RWBuffer<FMyCustomStruct> MyCustomStructs;

		// SHADER_PARAMETER_SRV(StructuredBuffer<FMyCustomStruct>, MyCustomStructs) // On the shader side: StructuredBuffer<FMyCustomStruct> MyCustomStructs;
		// SHADER_PARAMETER_SRV(Buffer<FMyCustomStruct>, MyCustomStructs) // On the shader side: Buffer<FMyCustomStruct> MyCustomStructs;
		// SHADER_PARAMETER_SRV(Texture2D<FVector4f>, MyReadOnlyTexture) // On the shader side: Texture2D<float4> MyReadOnlyTexture;

		// SHADER_PARAMETER_STRUCT_REF(FMyCustomStruct, MyCustomStruct)

		SHADER_PARAMETER_RDG_BUFFER_SRV(Buffer<int>, Input)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<int>, Output)

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
		OutEnvironment.SetDefine(TEXT("THREADS_X"), NUM_THREADS_MULTIPLY_X);
		OutEnvironment.SetDefine(TEXT("THREADS_Y"), NUM_THREADS_MULTIPLY_Y);
		OutEnvironment.SetDefine(TEXT("THREADS_Z"), NUM_THREADS_MULTIPLY_Z);

		// This shader must support typed UAV load and we are testing if it is supported at runtime using RHIIsTypedUAVLoadSupported
		//OutEnvironment.CompilerFlags.Add(CFLAG_AllowTypedUAVLoads);

		// FForwardLightingParameters::ModifyCompilationEnvironment(Parameters.Platform, OutEnvironment);
	}
};

// This will tell the engine to create the shader and where the shader entry point is.
// ShaderType | ShaderPath | Shader function name | Type
IMPLEMENT_GLOBAL_SHADER(FMultiplyShader, "/RsapShadersShaders/Multiply/Multiply.usf", "MultiplyShader", SF_Compute);

void FMultiplyShaderInterface::DispatchRenderThread(FRHICommandListImmediate& RHICmdList, FMultiplyShaderDispatchParams Params, TFunction<void(int OutputVal)> AsyncCallback)
{
	FRDGBuilder GraphBuilder(RHICmdList);

	{
		SCOPE_CYCLE_COUNTER(STAT_MultiplyShader_Execute);
		DECLARE_GPU_STAT(MultiplyShader)
		RDG_EVENT_SCOPE(GraphBuilder, "MultiplyShader");
		RDG_GPU_STAT_SCOPE(GraphBuilder, MultiplyShader);

		const FMultiplyShader::FPermutationDomain PermutationVector;

		// Add any static permutation options here
		// PermutationVector.Set<FMultiplyShader::FMyPermutationName>(12345);

		TShaderMapRef<FMultiplyShader> ComputeShader(GetGlobalShaderMap(GMaxRHIFeatureLevel), PermutationVector);

		if (!ComputeShader.IsValid())
		{
			#if WITH_EDITOR
				GEngine->AddOnScreenDebugMessage(static_cast<uint64>(42145125184), 6.f, FColor::Red, FString(TEXT("The compute shader has a problem.")));
			#endif
			return; // We exit here as we don't want to crash the game if the shader is not found or has an error.
		}

		FMultiplyShader::FParameters* PassParameters = GraphBuilder.AllocParameters<FMultiplyShader::FParameters>();

		const void* RawData = static_cast<void*>(Params.Input);
		constexpr int NumInputs = 2;
		constexpr int InputSize = sizeof(int);
		const FRDGBufferRef InputBuffer = CreateUploadBuffer(GraphBuilder, TEXT("InputBuffer"), InputSize, NumInputs, RawData, InputSize * NumInputs);
		PassParameters->Input = GraphBuilder.CreateSRV(FRDGBufferSRVDesc(InputBuffer, PF_R32_SINT));

		const FRDGBufferRef OutputBuffer = GraphBuilder.CreateBuffer(
			FRDGBufferDesc::CreateBufferDesc(sizeof(int32), 1),
			TEXT("OutputBuffer")
		);
		PassParameters->Output = GraphBuilder.CreateUAV(FRDGBufferUAVDesc(OutputBuffer, PF_R32_SINT));

		auto GroupCount = FComputeShaderUtils::GetGroupCount(FIntVector(Params.X, Params.Y, Params.Z), FComputeShaderUtils::kGolden2DGroupSize);
		GraphBuilder.AddPass(
			RDG_EVENT_NAME("ExecuteMultiplyShader"),
			PassParameters,
			ERDGPassFlags::AsyncCompute,
			[&PassParameters, ComputeShader, GroupCount](FRHIComputeCommandList& RHICommandList)
		{
			FComputeShaderUtils::Dispatch(RHICommandList, ComputeShader, *PassParameters, GroupCount);
		});

		FRHIGPUBufferReadback* GPUBufferReadback = new FRHIGPUBufferReadback(TEXT("ExecuteMultiplyShaderOutput"));
		AddEnqueueCopyPass(GraphBuilder, GPUBufferReadback, OutputBuffer, 0u);

		auto RunnerFunc = [GPUBufferReadback, AsyncCallback](auto&& RunnerFunction) -> void {
			if (GPUBufferReadback->IsReady()) {
				
				const int32* Buffer = static_cast<int32*>(GPUBufferReadback->Lock(1));
				int OutVal = Buffer[0];
				
				GPUBufferReadback->Unlock();

				AsyncTask(ENamedThreads::GameThread, [AsyncCallback, OutVal]() {
					AsyncCallback(OutVal);
				});

				delete GPUBufferReadback;
			} else {
				AsyncTask(ENamedThreads::ActualRenderingThread, [RunnerFunction]() {
					RunnerFunction(RunnerFunction);
				});
			}
		};

		AsyncTask(ENamedThreads::ActualRenderingThread, [RunnerFunc]() {
			RunnerFunc(RunnerFunc);
		});
	}

	GraphBuilder.Execute();
}