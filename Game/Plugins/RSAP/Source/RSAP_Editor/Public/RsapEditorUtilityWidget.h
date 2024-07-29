// Copyright Melvin Brink 2023. All Rights Reserved.

#pragma once

#include <EditorUtilityWidget.h>
#include "RsapEditorUtilityWidget.generated.h"



UCLASS()
class URsapEditorUtilityWidget : public UEditorUtilityWidget
{
	GENERATED_BODY()

protected:
	// Simple helper method for displaying a readable value for the chunk-size.
	UFUNCTION(BlueprintCallable)
	static FString GetChunkSizeString(const int32 ChunkSize)
	{
		if (ChunkSize < 100) return FString::Printf(TEXT("%i cm"), ChunkSize);
		if (ChunkSize < 100000) return FString::Printf(TEXT("%.2f m"), FMath::RoundToFloat(ChunkSize / 100.0f * 100.0f) / 100.0f);
		return FString::Printf(TEXT("%.2f km"), FMath::RoundToFloat(ChunkSize / 100000.0f * 100.0f) / 100.0f);
	}
};