#pragma once

#include "EditorUtilityWidget.h"
#include "NavMeshUtilityWidget.generated.h"



UCLASS()
class UNavMeshEditorUtilityWidget : public UEditorUtilityWidget
{
	GENERATED_BODY()

protected:
	UFUNCTION()
	void GenerateNavMesh();
};