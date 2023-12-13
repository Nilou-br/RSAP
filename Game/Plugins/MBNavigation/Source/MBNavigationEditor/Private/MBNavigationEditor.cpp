// Copyright Melvin Brink 2023. All Rights Reserved.

#include "MBNavigationEditor.h"
#include "EditorStyleSet.h"
#include "EditorStyleSet.h"
#include "LevelEditor.h"
#include "ToolMenus.h"
#include "WorldNavigationManager.h"
#include "Generation/NavMeshGenerator.h"
#include "Widgets/Input/SSlider.h"

#define LOCTEXT_NAMESPACE "FMBNavigationEditorModule"



void FMBNavigationEditorModule::StartupModule()
{
	// Register tool menu
	UToolMenus::RegisterStartupCallback(FSimpleMulticastDelegate::FDelegate::CreateRaw(this, &FMBNavigationEditorModule::RegisterToolbarSection));
}

void FMBNavigationEditorModule::ShutdownModule()
{
	IModuleInterface::ShutdownModule();
}

void FMBNavigationEditorModule::RegisterToolbarSection()
{
	// Get the toolbar
	UToolMenu *Toolbar = UToolMenus::Get()->ExtendMenu("LevelEditor.LevelEditorToolBar.PlayToolBar");

	// Add a new section for the world-navigation
	FToolMenuSection &WorldNavigationSection = Toolbar->AddSection("MBWorldNavigationSection");

	// Create button for opening navmesh generation menu
	const FToolMenuEntry NavMeshGeneratorDropdownMenu = FToolMenuEntry::InitComboButton(
		"MBWorldNavigationSection.NavMeshGeneratorDropdownMenu_Button",
		FUIAction(),
		FOnGetContent::CreateRaw(this, &FMBNavigationEditorModule::MakeNavmeshGeneratorMenu),
		LOCTEXT("SettingsCombo", "Navmesh generator"),
		LOCTEXT("SettingsCombo_Tooltip", "Open generator menu"),
		FSlateIcon(),
		false
	);

	// Add button(s) to the section.
	WorldNavigationSection.AddEntry(NavMeshGeneratorDropdownMenu);

	// Register the menu's for this World-Navigation Section
	RegisterMenus();
}

void FMBNavigationEditorModule::RegisterMenus()
{
	// Register the navmesh-generator menu
	UToolMenu* NavmeshGeneratorMenu = UToolMenus::Get()->RegisterMenu("MBWorldNavigation_Section.MBNavmeshGeneratorMenu");

	// Create generate-button section
	FToolMenuSection& GenerateSection = NavmeshGeneratorMenu->AddSection("MBNavmeshGeneratorMenu.Generate_Section");

	// Create and add generate-button to this section
	GenerateSection.AddMenuEntry(
		"MBNavmeshGeneratorMenu.Generate_Section.Generate_Button",
		LOCTEXT("Generate_Button_Label", "Generate Navmesh"),
		LOCTEXT("Generate_Button_Tooltip", "Generate the navigation mesh for current level."),
		FSlateIcon(),
		FUIAction(FExecuteAction::CreateRaw(this, &FMBNavigationEditorModule::GenerateNavmesh))
	);

	// Section separator
	NavmeshGeneratorMenu->AddSection("MBNavmeshGeneratorMenu.Divider_Section").AddSeparator("MBNavmeshGeneratorMenu.Divider");

	// Create settings section
	FToolMenuSection& SettingsSection = NavmeshGeneratorMenu->AddSection("MBNavmeshGeneratorMenu.Settings_Section");

	// Create and add Chunk-size slider to this section
	SettingsSection.AddEntry(FToolMenuEntry::InitWidget(
	   "MBNavmeshGeneratorMenu.Settings_Section.ChunkSize_Slider",
	   SNew(SHorizontalBox)
	   + SHorizontalBox::Slot()
	   .AutoWidth()
	   [
		   SNew(SBox)
		   .MinDesiredWidth(200)
		   [
			   SNew(SSlider)
			   .Value_Raw(this, &FMBNavigationEditorModule::GetChunkSize)
			   .OnValueChanged_Raw(this, &FMBNavigationEditorModule::OnChunkSizeChanged)
			   .StepSize(100)
			   .MinValue(800)
			   .MaxValue(16000)
		   ]
	   ]
	   + SHorizontalBox::Slot()
	   .AutoWidth()
	   .Padding(10, 0)
	   [
		   SNew(SBox)
		   .MinDesiredWidth(120)
		   [
				SNew(STextBlock)
				.Text_Raw(this, &FMBNavigationEditorModule::GetChunkSizeText)
		   ]
		],
	   LOCTEXT("ChunkSize_Label", "Chunk Size"),
	   false
   ));

	// Create and add Smallest-Voxel-Size slider to this section
	SettingsSection.AddEntry(FToolMenuEntry::InitWidget(
	   "MBNavmeshGeneratorMenu.Settings_Section.SmallestVoxelSize_Slider",
	   SNew(SHorizontalBox)
	   + SHorizontalBox::Slot()
	   .AutoWidth()
	   [
		   SNew(SBox)
		   .MinDesiredWidth(100)
		   [
			   SNew(SSlider)
			   .Value_Raw(this, &FMBNavigationEditorModule::GetSmallestVoxelSizeValue)
			   .OnValueChanged_Raw(this, &FMBNavigationEditorModule::OnSmallestVoxelSizeChanged)
			   .StepSize(1)
			   .MinValue(1)
			   .MaxValue(20)
		   ]
	   ]
	   + SHorizontalBox::Slot()
	   .AutoWidth()
	   .Padding(10, 0)
	   [
		   SNew(SBox)
		   .MinDesiredWidth(60)
		   [
				SNew(STextBlock)
				.Text_Raw(this, &FMBNavigationEditorModule::GetSmallestVoxelSizeText)
		   ]
		],
	   LOCTEXT("SmallestVoxelSize_Label", "Smallest Voxel Size"),
	   false
   ));
}

TSharedRef<SWidget> FMBNavigationEditorModule::MakeNavmeshGeneratorMenu()
{
	const FToolMenuContext MenuContext;
	return UToolMenus::Get()->GenerateWidget("MBWorldNavigation_Section.MBNavmeshGeneratorMenu", MenuContext);
}

void FMBNavigationEditorModule::GenerateNavmesh()
{
	UWorld* EditorWorld = GEditor->GetEditorWorldContext().World();
	if(!EditorWorld) return;

	const UWorldNavigationManager* WorldNavManager = EditorWorld->GetSubsystem<UWorldNavigationManager>(EditorWorld);
	if(!WorldNavManager) return;

	UNavMeshGenerator* NavMeshGenerator = NewObject<UNavMeshGenerator>();
	if(!NavMeshGenerator) return;
	
	NavMeshGenerator->Initialize(EditorWorld, FNavMeshSettings(SmallestVoxelSize, ChunkSize));
	FNavMesh NavMesh = NavMeshGenerator->Generate(WorldNavManager->GetLevelBoundaries());
	

	// Debug
	FlushPersistentDebugLines(EditorWorld);
	TArray<FChunk> Chunks;
	NavMesh.GenerateValueArray(Chunks);
	for (FChunk Chunk: Chunks)
	{
		DrawDebugBox(EditorWorld, Chunk.Location, FVector(ChunkSize/2), FColor::Orange, true);
	}
}

float FMBNavigationEditorModule::GetChunkSize() const
{
	return ChunkSize;
}

void FMBNavigationEditorModule::OnChunkSizeChanged(const float NewValue)
{
	// Round the value to the nearest multiple of 100
	const int32 RoundedValue = FMath::RoundToInt(NewValue / 100.0f) * 100;
	ChunkSize = static_cast<float>(RoundedValue);
}

FText FMBNavigationEditorModule::GetChunkSizeText() const
{
	FNumberFormattingOptions NumberFormat;
	NumberFormat.UseGrouping = false;

	const FText ValueCm = FText::AsNumber(static_cast<int32>(ChunkSize), &NumberFormat);
	const FText ValueMeter = FText::AsNumber(static_cast<int32>(ChunkSize / 100.0f), &NumberFormat);
	return FText::Format(NSLOCTEXT("", "SliderValue", " {0} {1} / {2} {3}"), ValueCm, FText::FromString(TEXT("cm")) , ValueMeter, FText::FromString(TEXT("m")));
}

float FMBNavigationEditorModule::GetSmallestVoxelSizeValue() const
{
	return SmallestVoxelSize;
}

void FMBNavigationEditorModule::OnSmallestVoxelSizeChanged(const float NewValue)
{
	// Round the value to the nearest multiple of 1
	const int32 RoundedValue = FMath::RoundToInt(NewValue);
	SmallestVoxelSize = static_cast<float>(RoundedValue);
}

FText FMBNavigationEditorModule::GetSmallestVoxelSizeText() const
{
	FNumberFormattingOptions NumberFormat;
	NumberFormat.UseGrouping = false;

	const FText ValueCm = FText::AsNumber(static_cast<int32>(SmallestVoxelSize), &NumberFormat);
	return FText::Format(NSLOCTEXT("", "SmallestVoxelSizeValueFormat", "{0}{1}"), ValueCm, FText::FromString(TEXT(" cm")));
}



#undef LOCTEXT_NAMESPACE
    
IMPLEMENT_MODULE(FMBNavigationEditorModule, MBNavigationEditor)