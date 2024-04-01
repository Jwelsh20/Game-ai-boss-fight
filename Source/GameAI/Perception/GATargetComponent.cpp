#include "GATargetComponent.h"
#include "Kismet/GameplayStatics.h"
#include "GameAI/Grid/GAGridActor.h"
#include "GAPerceptionSystem.h"
#include "ProceduralMeshComponent.h"
#include "CoreMinimal.h"
#include "GameAI/Pathfinding/GAPathComponent.h"
#include "Math/UnrealMathUtility.h"



UGATargetComponent::UGATargetComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	// A bit of Unreal magic to make TickComponent below get called
	PrimaryComponentTick.bCanEverTick = true;

	SetTickGroup(ETickingGroup::TG_PostUpdateWork);

	// Generate a new guid
	TargetGuid = FGuid::NewGuid();
}


AGAGridActor* UGATargetComponent::GetGridActor() const
{
	AGAGridActor* Result = GridActor.Get();
	if (Result)
	{
		return Result;
	}
	else
	{
		AActor* GenericResult = UGameplayStatics::GetActorOfClass(this, AGAGridActor::StaticClass());
		if (GenericResult)
		{
			Result = Cast<AGAGridActor>(GenericResult);
			if (Result)
			{
				// Cache the result
				// Note, GridActor is marked as mutable in the header, which is why this is allowed in a const method
				GridActor = Result;
			}
		}

		return Result;
	}
}


void UGATargetComponent::OnRegister()
{
	Super::OnRegister();

	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		PerceptionSystem->RegisterTargetComponent(this);
	}

	const AGAGridActor* Grid = GetGridActor();
	if (Grid)
	{
		OccupancyMap = FGAGridMap(Grid, 0.0f);
	}
}

void UGATargetComponent::OnUnregister()
{
	Super::OnUnregister();

	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		PerceptionSystem->UnregisterTargetComponent(this);
	}
}

void UGATargetComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	bool isImmediate = false;

	// update my perception state FSM
	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		TArray<TObjectPtr<UGAPerceptionComponent>> &PerceptionComponents = PerceptionSystem->GetAllPerceptionComponents();
		for (UGAPerceptionComponent* PerceptionComponent : PerceptionComponents)
		{
			const FTargetData* TargetData = PerceptionComponent->GetTargetData(TargetGuid);
			if (TargetData && (TargetData->Awareness >= 1.0f))
			{
				isImmediate = true;
				break;
			}
		}
	}

	if (isImmediate)
	{
		AActor* Owner = GetOwner();
		LastKnownState.State = GATS_Immediate;

		// REFRESH MY STATE
		LastKnownState.Set(Owner->GetActorLocation(), Owner->GetVelocity());

		// Tell the omap to clear out and put all the probability in the observed location
		OccupancyMapSetPosition(LastKnownState.Position);
	}
	else if (IsKnown())
	{
		LastKnownState.State = GATS_Hidden;
	}

	if (LastKnownState.State == GATS_Hidden)
	{
		OccupancyMapUpdate();
	}

	// As long as I'm known, whether I'm immediate or not, diffuse the probability in the omap

	if (IsKnown())
	{
		OccupancyMapDiffuse();
	}

	if (bDebugOccupancyMap)
	{
		AGAGridActor* Grid = GetGridActor();
		Grid->DebugGridMap = OccupancyMap;
		GridActor->RefreshDebugTexture();
		GridActor->DebugMeshComponent->SetVisibility(true);
	}
}


void UGATargetComponent::OccupancyMapSetPosition(const FVector& Position)
{
	// TODO PART 4

	// We've been observed to be in a given position
	// Clear out all probability in the omap, and set the appropriate cell to P = 1.0
	AGAGridActor* Grid = GetGridActor();
	FCellRef curPosCell = Grid->GetCellRef(Position);

	for (int32 Y = OccupancyMap.GridBounds.MinY; Y < OccupancyMap.GridBounds.MaxY; Y++)
	{
		for (int32 X = OccupancyMap.GridBounds.MinX; X < OccupancyMap.GridBounds.MaxX; X++)
		{
			if (Y == curPosCell.Y && X == curPosCell.X) {
				OccupancyMap.SetValue(curPosCell, 1.0f);
			}
			else {
				FCellRef CellRef(X, Y);
				OccupancyMap.SetValue(CellRef, 0.0f);
			}
		}
	}
}

// Function to check if a vector is within the vision angle
bool IsWithinVisionAngle(const FVector& OriginalVector, const FVector& TargetVector, float VisionAngle)
{
	// Normalize the vectors
	FVector NormalizedOriginal = OriginalVector.GetSafeNormal();
	FVector NormalizedTarget = TargetVector.GetSafeNormal();

	// Calculate the dot product between the normalized vectors
	float DotProduct = FVector::DotProduct((FVector&) NormalizedOriginal, (FVector&) NormalizedTarget);
	DotProduct = FMath::Clamp(DotProduct, -1.0f, 1.0f);

	// Calculate the angle between the vectors in radians
	float AngleRad = FMath::Acos(DotProduct);

	// Convert the angle to degrees
	float AngleDeg = FMath::RadiansToDegrees(AngleRad);

	// Check if the angle is within the vision angle
	return AngleDeg <= VisionAngle / 2.0f;
}

// Function to check if one FVector is within a certain distance from another FVector
bool IsWithinDistance(const FVector& VectorA, const FVector& VectorB, float DistanceThreshold)
{
	// Calculate the squared distance between the vectors
	float SquaredDistance = FVector::DistSquared(VectorA, VectorB);

	// Calculate the squared distance threshold
	float SquaredThreshold = FMath::Square(DistanceThreshold);

	// Check if the squared distance is less than or equal to the squared threshold
	return SquaredDistance <= SquaredThreshold;
}


void UGATargetComponent::OccupancyMapUpdate()
{
	AGAGridActor* Grid = GetGridActor();
	APawn* PlayerPawn = UGameplayStatics::GetPlayerPawn(this, 0);
	if (Grid)
	{
		FGAGridMap VisibilityMap(Grid, 0.0f);

		// TODO PART 4

		// STEP 1: Build the visibility map, based on the perception components of the AIs in the world

		UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
		if (PerceptionSystem)
		{
			TArray<TObjectPtr<UGAPerceptionComponent>>& PerceptionComponents = PerceptionSystem->GetAllPerceptionComponents();
			for (UGAPerceptionComponent* PerceptionComponent : PerceptionComponents)
			{
				APawn* OwnerPawn = PerceptionComponent->GetOwnerPawn();
				FVector Start = OwnerPawn->GetActorLocation();
				FVector ForwardVector = OwnerPawn->GetActorRotation().Vector();
				
				float angle = PerceptionComponent->VisionParameters.VisionAngle;
				float dist = PerceptionComponent->VisionParameters.VisionDistance;

				for (int32 Y = OccupancyMap.GridBounds.MinY; Y < OccupancyMap.GridBounds.MaxY; Y++)
				{
					for (int32 X = OccupancyMap.GridBounds.MinX; X < OccupancyMap.GridBounds.MaxX; X++)
					{

						FCellRef CellRef(X, Y);
						FVector End = Grid->GetCellPosition(CellRef);
						End.Z = Start.Z;

						bool flags = EnumHasAllFlags(Grid->GetCellData(CellRef), ECellData::CellDataTraversable);
						bool inAngle = IsWithinVisionAngle(ForwardVector, End - Start, angle);
						bool inDist = IsWithinDistance(Start, End, dist);
						bool hasFound = IsWithinDistance(Start, LastKnownState.Position, 200.0f);

						if ((flags && inAngle && inDist) || (hasFound)) { //check if the cell is within the VisionDistance before casting a ray because if it isnt theres no point since its not visible or if its reached the max cell

							UWorld* World = GetWorld();
							FHitResult HitResult;
							FCollisionQueryParams Params;
							Params.AddIgnoredActor(PlayerPawn);			// Probably want to ignore the player pawn
							Params.AddIgnoredActor(OwnerPawn);			// Probably want to ignore the AI themself
							bool bHitSomething = World->LineTraceSingleByChannel(HitResult, Start, End, ECollisionChannel::ECC_Visibility, Params);
							//If bHitSomething is false, then we have a clear LOS

							if (!bHitSomething) {
								VisibilityMap.SetValue(CellRef, 1.0f);
							}

						}	
					}
				}
			}
		}

		// STEP 2: Clear out the probability in the visible cells
		float oMapSum = 0.0f;
		float ocuVal;

		for (int32 Y = VisibilityMap.GridBounds.MinY; Y < VisibilityMap.GridBounds.MaxY; Y++)
		{
			for (int32 X = VisibilityMap.GridBounds.MinX; X < VisibilityMap.GridBounds.MaxX; X++)
			{
				FCellRef CellRef(X, Y);
				float visVal;
				VisibilityMap.GetValue(CellRef, visVal);
				if (visVal == 1.0f) {
					OccupancyMap.SetValue(CellRef, 0.0f);
				}

				OccupancyMap.GetValue(CellRef, ocuVal);
				oMapSum += ocuVal;
			}
		}

		// STEP 3: Renormalize the OMap, so that it's still a valid probability distribution
		float maxVal = 0.0f;
		FCellRef maxCell;

		for (int32 Y = OccupancyMap.GridBounds.MinY; Y < OccupancyMap.GridBounds.MaxY; Y++)
		{
			for (int32 X = OccupancyMap.GridBounds.MinX; X < OccupancyMap.GridBounds.MaxX; X++)
			{
				FCellRef CellRef(X, Y);

				bool flags = EnumHasAllFlags(Grid->GetCellData(CellRef), ECellData::CellDataTraversable);

				if (flags) {
					float curVal;
					OccupancyMap.GetValue(CellRef, curVal);
					float newVal = curVal / oMapSum;
					OccupancyMap.SetValue(CellRef, newVal);

					if (newVal > maxVal) {
						maxVal = newVal;
						maxCell = CellRef;
					}
				}
			}
		}

		// STEP 4: Extract the highest-likelihood cell on the omap and refresh the LastKnownState.
		FVector maxVec = Grid->GetCellPosition(maxCell);
		LastKnownState.Position = maxVec;
	}

}


void UGATargetComponent::OccupancyMapDiffuse()
{
	// TODO PART 4
	// Diffuse the probability in the OMAP
	float alpha = 0.75f;
	float r2 = FMath::Sqrt(2.0f);

	std::list<std::pair<int, int>> sides = {
			{0,1},
			{0,-1},
			{1,0},
			{-1,0},
	};

	std::list<std::pair<int, int>> diag = {
			{1,1},
			{1,-1},
			{-1,1},
			{-1,-1},
	};

	int32 maxX = OccupancyMap.GridBounds.MaxX;
	int32 maxY = OccupancyMap.GridBounds.MaxY;
	int32 minX = OccupancyMap.GridBounds.MinX;
	int32 minY = OccupancyMap.GridBounds.MinY;

	AGAGridActor* Grid = GetGridActor();

	for (int32 Y = minY; Y < maxY; Y++)	//Only diffuse cells with >0.0f value
	{
		for (int32 X = minX; X < maxX; X++)
		{
			FCellRef CellRef(X, Y);
			float curProb;
			OccupancyMap.GetValue(CellRef, curProb);

			if (curProb == 0.0f) {
				continue;
			}

			float sideProb = alpha * curProb;
			float diagProb = (alpha * curProb) / r2;

			
			bool flags = EnumHasAllFlags(Grid->GetCellData(CellRef), ECellData::CellDataTraversable);

			for (const auto& s : sides) {

				int32 newX = X + s.first;
				int32 newY = Y + s.second;

				if (flags && newX < maxX && newX >= minX && newY < maxY && newY >= minY) {
					FCellRef sideRef(newX, newY);
					float sideVal;
					OccupancyMap.GetValue(sideRef, sideVal);
					
					if (curProb > sideVal) { //Only diffuse larger probabilities to smaller probabilities
						OccupancyMap.SetValue(sideRef, sideProb);
					}
				}

			}

			for (const auto& d : diag) {

				int32 newX = X + d.first;
				int32 newY = Y + d.second;

				if (flags && newX < maxX && newX >= minX && newY < maxY && newY >= minY) {
					FCellRef diagRef(newX, newY);
					float diagVal;
					OccupancyMap.GetValue(diagRef, diagVal);

					if (curProb > diagVal) { //Only diffuse larger probabilities to smaller probabilities
						OccupancyMap.SetValue(diagRef, diagVal);
					}
				}
			}
		}
	}

	float oMapSum = 0.0f;
	float mapVal;

	//Get the sum of probabilities after diffusion
	for (int32 Y = OccupancyMap.GridBounds.MinY; Y < OccupancyMap.GridBounds.MaxY; Y++)
	{
		for (int32 X = OccupancyMap.GridBounds.MinX; X < OccupancyMap.GridBounds.MaxX; X++)
		{
			FCellRef CellRef(X, Y);

			OccupancyMap.GetValue(CellRef, mapVal);
			oMapSum += mapVal;
		}
	}

	//Renormalize probabilities after diffusion
	for (int32 Y = OccupancyMap.GridBounds.MinY; Y < OccupancyMap.GridBounds.MaxY; Y++)
	{
		for (int32 X = OccupancyMap.GridBounds.MinX; X < OccupancyMap.GridBounds.MaxX; X++)
		{
			FCellRef CellRef(X, Y);

			float curVal;
			OccupancyMap.GetValue(CellRef, curVal);
			float newVal = curVal / oMapSum;
			OccupancyMap.SetValue(CellRef, newVal);
		}
	}

}
