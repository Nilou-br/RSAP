#pragma once

#include "Rsap/NavMesh/Types/MeshComponentProxy.h"



/**
* Render-thread navmesh that communicates with the GPU.
 */
class RSAPSHARED_API FRsapNavmeshShaderProxy
{
public:
	TMap<TObjectPtr<UStaticMeshComponent>, FRsapMeshComponentRenderData> MeshComponentsRenderData;
	
	TSet<TObjectPtr<UStaticMeshComponent>> PreprocessBatch;
	TSet<TObjectPtr<UStaticMeshComponent>> VoxelizeBatch;

	FRsapNavmeshShaderProxy() = default;
};