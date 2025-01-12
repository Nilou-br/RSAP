#include "Rsap/NavMesh/Navmesh.h"
#include "RsapShaders/Public/Voxelization/Preprocess.h"



FRsapNavmesh::FRsapNavmesh()
{
	FVoxelizationPreprocessInterface::OnVoxelizationPreprocessComplete.BindRaw(this, &FRsapNavmesh::OnPreprocessCompleted);
	//FVoxelizationPreprocessInterface::OnVoxelizationComplete.BindRaw(this, &FRsapNavmesh::OnVoxelizationCompleted);
}

// Initializes the navmesh by voxelizing the given static-mesh-components.
void FRsapNavmesh::Initialize(const TArray<TObjectPtr<UStaticMeshComponent>>& StaticMeshComponents)
{
	ShaderProxy.PreprocessBatch.Append(StaticMeshComponents);
	FVoxelizationPreprocessInterface::Dispatch(ShaderProxy);
}

void FRsapNavmesh::MarkComponentDirty(TObjectPtr<UStaticMeshComponent>& StaticMeshComponent)
{
	DirtyMeshComponents.Emplace(StaticMeshComponent);
}

void FRsapNavmesh::OnPreprocessCompleted()
{
	//FVoxelizationInterface::Dispatch(ShaderProxy);
	UE_LOG(LogRsap, Log, TEXT("OnPreprocessCompleted"))
}

void FRsapNavmesh::OnVoxelizationCompleted()
{
	
}
