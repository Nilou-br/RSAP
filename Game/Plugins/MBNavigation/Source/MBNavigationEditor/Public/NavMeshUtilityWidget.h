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
	void GenerateNavMesh(const float ChunkSizeFloat, const float StaticDepthFloat,  const bool bDisplayDebug);

	UFUNCTION(BlueprintCallable)
	FString GetChunkSizeString(const int32 ChunkSize);
};