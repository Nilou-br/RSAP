// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include "EditorUtilityWidget.h"
#include "NavMeshUtilityWidget.generated.h"



UCLASS()
class UNavMeshEditorUtilityWidget : public UEditorUtilityWidget
{
	GENERATED_BODY()

protected:
	UFUNCTION(BlueprintCallable)
	void GenerateNavMesh(uint8 StaticDepth, uint8 DynamicDepth, const float SmallestVoxelSize, const float ChunkSize);

	UFUNCTION(BlueprintCallable)
	FString GetChunkSizeString(const float ChunkSize);
};