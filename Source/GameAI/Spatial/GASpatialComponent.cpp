#include "GASpatialComponent.h"
#include "GameAI/Pathfinding/GAPathComponent.h"
#include "GameAI/Grid/GAGridMap.h"
#include "Kismet/GameplayStatics.h"
#include "Math/MathFwd.h"
#include "GASpatialFunction.h"
#include "ProceduralMeshComponent.h"
#include <queue>
#include <vector>
#include <algorithm>
#include <map>
#include <tuple>
#include <utility>
#include <list>
#include "Containers/Map.h"
#include "Containers/Array.h"
#include "CoreMinimal.h"

using namespace std;


UGASpatialComponent::UGASpatialComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	SampleDimensions = 20000.0f;		// should cover the bulk of the test mapdd
}


const AGAGridActor* UGASpatialComponent::GetGridActor() const
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

const UGAPathComponent* UGASpatialComponent::GetPathComponent() const
{
	const UGAPathComponent* Result = PathComponent.Get();
	if (Result)
	{
		return Result;
	}
	else
	{
		AActor* Owner = GetOwner();
		if (Owner)
		{
			// Note, the UGAPathComponent and the UGASpatialComponent are both on the controller
			Result = Owner->GetComponentByClass<UGAPathComponent>();
			if (Result)
			{
				PathComponent = Result;
			}
		}
		return Result;
	}
}

APawn* UGASpatialComponent::GetOwnerPawn() const
{
	AActor* Owner = GetOwner();
	if (Owner)
	{
		APawn* Pawn = Cast<APawn>(Owner);
		if (Pawn)
		{
			return Pawn;
		}
		else
		{
			AController* Controller = Cast<AController>(Owner);
			if (Controller)
			{
				return Controller->GetPawn();
			}
		}
	}

	return NULL;
}

//Function that takes in the dijkstra distance map, the starting point, the max cell, and the grid and returns the shortest path between the starting point and the max cell via dijkstra
vector<FCellRef> getPositionPath(const FGAGridMap& DistanceMapOut, const FVector& StartPoint, const FCellRef& dest, const AGAGridActor* Grid) {

	FCellRef curCell = dest;
	FCellRef startCell = Grid->GetCellRef(StartPoint);
	vector<FCellRef> path;
	int iter = 0;

	list<pair<int, int>> directions = {
			{0,1},
			{0,-1},
			{1,0},
			{-1,0},
	};

	//Loop to start from the destination cell and follow the dijkstra neighbors back to the start cell to find the shortest path
	while (curCell != startCell && iter < 1000) {
		path.push_back(curCell);
		
		float cellVal = FLT_MAX;
		FCellRef nextCell = curCell;

		for (const auto& d : directions) {
			int newX = curCell.X + d.first;
			int newY = curCell.Y + d.second;
			FCellRef adjCell = FCellRef(newX, newY);

			//Find the smallest distance neighbor and set the current cell to that cell so it can follow the dijkstra path
			if (DistanceMapOut.GridBounds.IsValidCell(adjCell) && Grid->GetCellData(adjCell) == ECellData::CellDataTraversable) {

				float adjDist;
				DistanceMapOut.GetValue(adjCell, adjDist);

				if (adjDist < cellVal) {
					cellVal = adjDist;
					nextCell = adjCell;
				}

			}
		}

		curCell = nextCell;
		iter++;
	}

	//Reverses the order of the path since the path was constructed in reverse order
	reverse(path.begin(), path.end());

	return path;
}

//Helper function for getLineTrace that will get 80 points along the line created from the current FVector location to the FVector location of the goal cell
//Checks to see if there are any obstructions in the way that the robot could run into or if it is an open straight shot to the cell
bool getPointsAlongLine2(const FVector& start, const FVector& end, const AGAGridActor* Grid) {
	int steps = 80; //I Chose a step size of 80 because the grid is 80x80 and this makes it less likely that there would be an obstruction that wasnt caught by the interpolation

	for (int i = 0; i < steps; ++i) {
		// Calculate the interpolation factor (t) based on the current point index
		float t = static_cast<float>(i) / (steps - 1);

		// Interpolate between start and end using lerp
		float x = start.X + t * (end.X - start.X);
		float y = start.Y + t * (end.Y - start.Y);

		FCellRef lineCell = Grid->GetCellRef(FVector(x, y, 0.0f));
		if (Grid->GetCellData(lineCell) != ECellData::CellDataTraversable) { //Check to see if the Cell associated with the FVector coordinates found by lerp is not traversible and will return true meaning that cell is not reachable by a straight line
			return true;
		}
	}
	return false;
}

//LineTrace() function for path smoothing
//Function takes in the current path from robot to player, the location of the robot, and a reference to the grid
//Returns a tuple so I can easily access the FVector and FCellRef representation of the same location
tuple<FVector, FCellRef> getLineTrace2(const vector<FCellRef>& path, const FVector& origin, const AGAGridActor* Grid) {

	FCellRef originCell = Grid->GetCellRef(origin);
	FCellRef prev = originCell;

	//Loop through the A* path and check if cell in the path is directly reachable from the origin without hitting a wall by passing the cell vector to getPointsAlongLine
	for (const auto& element : path) {
		FVector destCellVec = Grid->GetCellPosition(element);

		if (getPointsAlongLine2(origin, destCellVec, Grid)) {
			return make_tuple(Grid->GetCellPosition(prev), prev); //If there is an obstruction found between the robot and the current cell in the loop, the function returns the previous cell that could be reached via a straight line
		}

		prev = element;
	}

	return make_tuple(Grid->GetCellPosition(path.back()), path.back()); //If all cells return false that means all are reachable via a straight line and it returns the end of the path (i.e. the player's cell) as the one to point towards
}

//Comparator struct to order the <dist, cell> pairs by lowest distance in Dijkstra priority queue
struct PairLess {
	bool operator()(const std::pair<float, FCellRef>& lhs, const std::pair<float, FCellRef>& rhs) const {
		return lhs.first > rhs.first;  // Compare based on the float values
	}
};

//Compare function to compare float of <cell val, cell> pair to order by max cell value from GridMap
bool comparePairs(const std::pair<float, FCellRef>& a, const std::pair<float, FCellRef>& b) {
	return a.first < b.first;
}


bool UGASpatialComponent::ChoosePosition(bool PathfindToPosition, bool Debug)
{
	bool Result = false;
	const APawn* OwnerPawn = GetOwnerPawn();
	const AGAGridActor* Grid = GetGridActor();

	const UGAPathComponent* pathComponent = GetPathComponent();
	UGAPathComponent* nonConstPathComponent = const_cast<UGAPathComponent*>(pathComponent);

	if (SpatialFunctionReference.Get() == NULL)
	{
		UE_LOG(LogTemp, Warning, TEXT("UGASpatialComponent has no SpatialFunctionReference assigned."));
		return false;
	}

	// Don't worry too much about the Unreal-ism below. Technically our SpatialFunctionReference is not ACTUALLY
	// a spatial function instance, rather it's a class, which happens to have a lot of data in it.
	// Happily, Unreal creates, under the hood, a default object for every class, that lets you access that data
	// as if it were a normal instance
	const UGASpatialFunction* SpatialFunction = SpatialFunctionReference->GetDefaultObject<UGASpatialFunction>();

	// The below is to create a GridMap (which you will fill in) based on a bounding box centered around the OwnerPawn
	FVector pawnLocation = OwnerPawn->GetActorLocation();
	FCellRef pawnCell = Grid->GetCellRef(pawnLocation);

	nonConstPathComponent->Steps.SetNum(1);

	//if (Grid->GetCellData(pawnCell) != ECellData::CellDataTraversable) {
	//	APawn* PlayerPawn = UGameplayStatics::GetPlayerPawn(this, 0);
	//	FVector playerVec = PlayerPawn->GetActorLocation();
	//	nonConstPathComponent->SetDestination(playerVec);
	//	nonConstPathComponent->Steps[0].Set(FVector2D(playerVec), Grid->GetCellRef(playerVec));
	//	nonConstPathComponent->State = GAPS_Active;
	//	return true;
	//}

	FBox2D Box(EForceInit::ForceInit);
	FIntRect CellRect;
	FVector2D PawnLocation(pawnLocation);
	Box += PawnLocation;
	Box = Box.ExpandBy(SampleDimensions / 2.0f);
	if (GridActor->GridSpaceBoundsToRect2D(Box, CellRect))
	{
		// Super annoying, by the way, that FIntRect is not blueprint accessible, because it forces us instead
		// to make a separate bp-accessible FStruct that represents _exactly the same thing_.
		FGridBox GridBox(CellRect);

		// This is the grid map I'm going to fill with values
		FGAGridMap GridMap(Grid, GridBox, 0.0f);

		// Fill in this distance map using Dijkstra!
		FGAGridMap DistanceMap(Grid, GridBox, FLT_MAX);


		// ~~~ STEPS TO FILL IN FOR ASSIGNMENT 3 ~~~

		// Step 1: Run Dijkstra's to determine which cells we should even be evaluating (the GATHER phase)
		// (You should add a Dijkstra() function to the UGAPathComponent())
		// I would recommend adding a method to the path component which looks something like
		// bool UGAPathComponent::Dijkstra(const FVector &StartPoint, FGAGridMap &DistanceMapOut) const;
		
		Dijkstra(pawnLocation, DistanceMap);

		// Step 2: For each layer in the spatial function, evaluate and accumulate the layer in GridMap
		// Note, only evaluate accessible cells found in step 1
		for (const FFunctionLayer& Layer : SpatialFunction->Layers)
		{
			// figure out how to evaluate each layer type, and accumulate the value in the GridMap
			EvaluateLayer(Layer, GridMap, DistanceMap);
		}

		// Step 3: pick the best cell in GridMap

		//Loop through all cells in the grid and add them and their value to a vector
		vector<std::pair<float, FCellRef>> cellValVec;

		for (int32 Y = GridMap.GridBounds.MinY; Y < GridMap.GridBounds.MaxY; Y++)
		{
			for (int32 X = GridMap.GridBounds.MinX; X < GridMap.GridBounds.MaxX; X++)
			{
				FCellRef CellRef(X, Y);
				float curVal;
				GridMap.GetValue(CellRef, curVal);

				cellValVec.push_back({ curVal, CellRef });
			}
		}

		//sort the vector by highest value
		sort(cellValVec.begin(), cellValVec.end(), comparePairs);

		//loop through vector to find the highest value cell that is also traversable in Dijkstra and set that as the max
		FCellRef maxCell;
		for (const auto& pair : cellValVec) {
			float curDistVal;
			DistanceMap.GetValue(pair.second, curDistVal);

			if (curDistVal >= 0 && curDistVal < FLT_MAX && trunc(curDistVal) == curDistVal) {
				maxCell = pair.second;
			}
		}

		// Let's pretend for now we succeeded.
		Result = true;

		if (PathfindToPosition)
		{
			// Step 4: Go there!
			// This will involve reconstructing the path and then getting it into the UGAPathComponent
			// Depending on what your cached Dijkstra data looks like, the path reconstruction might be implemented here
			// or in the UGAPathComponent

			//if ladder to get the path from current robot cell to max cell and follow the smoothed path towards the max until it reaches it and stops
			vector<FCellRef> path = getPositionPath(DistanceMap, pawnLocation, maxCell, Grid);
			if (path.size() > 1 && path.size() < 1000) {
				tuple<FVector, FCellRef> moveToTuple = getLineTrace2(path, pawnLocation, Grid);
				nonConstPathComponent->SetDestination(Grid->GetCellPosition(get<1>(moveToTuple)));
				nonConstPathComponent->Steps[0].Set(FVector2D(get<0>(moveToTuple)), get<1>(moveToTuple));
				nonConstPathComponent->State = GAPS_Active;
			}
			else if (path.size() == 1000) {
				nonConstPathComponent->SetDestination(Grid->GetCellPosition(maxCell));
				nonConstPathComponent->Steps[0].Set(Grid->GetCellGridSpacePosition(maxCell), maxCell);
				nonConstPathComponent->State = GAPS_Active;
			}
			else {
				nonConstPathComponent->State = GAPS_Finished;
			}
		}
		
		if (Debug)
		{
			// Note: this outputs (basically) the results of the position selection
			// However, you can get creative with the debugging here. For example, maybe you want
			// to be able to examine the values of a specific layer in the spatial function
			// You could create a separate debug map above (where you're doing the evaluations) and
			// cache it off for debug rendering. Ideally you'd be able to control what layer you wanted to 
			// see from blueprint

			GridActor->DebugGridMap = GridMap;
			GridActor->RefreshDebugTexture();
			GridActor->DebugMeshComponent->SetVisibility(true);		//cheeky!
		}
	}

	return Result;
}


void UGASpatialComponent::EvaluateLayer(const FFunctionLayer& Layer, FGAGridMap &GridMap, const FGAGridMap &DistanceMap) const
{
	AActor* OwnerPawn = GetOwnerPawn();
	const AGAGridActor* Grid = GetGridActor();

	APawn* PlayerPawn = UGameplayStatics::GetPlayerPawn(this, 0);
	FVector StartPoint = OwnerPawn->GetActorLocation();
	FVector End = PlayerPawn->GetActorLocation();
	
	//UGAPathComponent* nonConstPathComponent = const_cast<UGAPathComponent*>(pathComponent);

	for (int32 Y = GridMap.GridBounds.MinY; Y < GridMap.GridBounds.MaxY; Y++)
	{
		for (int32 X = GridMap.GridBounds.MinX; X < GridMap.GridBounds.MaxX; X++)
		{
			FCellRef CellRef(X, Y);
			//DistanceMap.GridBounds.IsValidCell(CellRef) && 
			if (EnumHasAllFlags(Grid->GetCellData(CellRef), ECellData::CellDataTraversable))
			{
				// evaluate me!

				// First step is determine input value. Remember there are three possible inputs to handle:
				// 	SI_None				UMETA(DisplayName = "None"),
				//	SI_TargetRange		UMETA(DisplayName = "Target Range"),
				//	SI_PathDistance		UMETA(DisplayName = "PathDistance"),
				//	SI_LOS				UMETA(DisplayName = "Line Of Sight")

				// Next, run it through the response curve using something like this
				// float Value = 4.5f;
				// float ModifiedValue = Layer.ResponseCurve.GetRichCurveConst()->Eval(Value, 0.0f);

				// Then add it's influence to the grid map, combining with the current value using one of the two operators
				//	SO_None				UMETA(DisplayName = "None"),
				//	SO_Add				UMETA(DisplayName = "Add"),			// add this layer to the accumulated buffer
				//	SO_Multiply			UMETA(DisplayName = "Multiply")		// multiply this layer into the accumulated buffer

				float ModifiedValue;
				float value;

				switch (Layer.Input) {
					case ESpatialInput::SI_None:
						value = 0.0f;
						break;
					case ESpatialInput::SI_TargetRange: 
						value = FVector::Dist(Grid->GetCellPosition(CellRef), End); //Get FVector dist from current cell to player cell
						break;
					case ESpatialInput::SI_PathDistance:
						float tempDist;
						DistanceMap.GetValue(CellRef, tempDist); //Get Dijkstra path distance value and makes sure it is traversable from Dijkstra
						value = (tempDist != FLT_MAX) ? tempDist : 0.0f;
						break;
					case ESpatialInput::SI_PERCEP:
						value = 0.0f;
						break;
					case ESpatialInput::SI_LOS:
						UWorld* World = GetWorld();
						FHitResult HitResult;
						FCollisionQueryParams Params;
						FVector Start = Grid->GetCellPosition(CellRef);
						Start.Z = End.Z;		// Hack: we don't have Z information in the grid actor -- take the player's z value and raycast against that
						Params.AddIgnoredActor(PlayerPawn);			// Probably want to ignore the player pawn
						Params.AddIgnoredActor(OwnerPawn);			// Probably want to ignore the AI themself
						bool bHitSomething = World->LineTraceSingleByChannel(HitResult, Start, End, ECollisionChannel::ECC_Visibility, Params);
						//If bHitSomething is false, then we have a clear LOS
						value = bHitSomething ? 0.0f : 1.0f;
						break;
				}

				ModifiedValue = Layer.ResponseCurve.GetRichCurveConst()->Eval(value, 0.0f);
				float curValue;
				GridMap.GetValue(CellRef, curValue);

				switch (Layer.Op) {
					case ESpatialOp::SO_None: 
						GridMap.SetValue(CellRef, 0.0f);
						break;
					case ESpatialOp::SO_Add: 
						GridMap.SetValue(CellRef, curValue + ModifiedValue);
						break;
					case ESpatialOp::SO_Multiply: 
						GridMap.SetValue(CellRef, curValue * ModifiedValue);
						break;
				}

				// HERE ARE SOME ADDITIONAL HINTS

				// Here's how to get the player's pawn
				// APawn *PlayerPawn = UGameplayStatics::GetPlayerPawn(this, 0);

				// Here's how to cast a ray

				// UWorld* World = GetWorld();
				// FHitResult HitResult;
				// FCollisionQueryParams Params;
				// FVector Start = Grid->GetCellPosition(CellRef);		// need a ray start
				// FVector End = PlayerPawn->GetActorLocation();		// need a ray end
				// Start.Z = End.Z;		// Hack: we don't have Z information in the grid actor -- take the player's z value and raycast against that
				// Add any actors that should be ignored by the raycast by calling
				// Params.AddIgnoredActor(PlayerPawn);			// Probably want to ignore the player pawn
				// Params.AddIgnoredActor(OwnerPawn);			// Probably want to ignore the AI themself
				// bool bHitSomething = World->LineTraceSingleByChannel(HitResult, Start, End, ECollisionChannel::ECC_Visibility, Params);
				// If bHitSomething is false, then we have a clear LOS
			}
		}
	}
}

//Dijkstra implementation to get distance from start to all traversible cells in the distance map
bool UGASpatialComponent::Dijkstra(const FVector& StartPoint, FGAGridMap& DistanceMapOut) const
{
	const AGAGridActor* Grid = GetGridActor();

	FCellRef startCell = Grid->GetCellRef(StartPoint);
	priority_queue<pair<float, FCellRef>, vector<pair<float, FCellRef>>, PairLess> pq;
	pq.push({ 0.0f, startCell });
	vector<FCellRef> visited;
	visited.push_back(startCell);
	FCellRef temp = FCellRef(10, 75);

	list<pair<int, int>> directions = {
			{0,1},
			{0,-1},
			{1,0},
			{-1,0},
	};

	//loops while there are still cells in the priority queue
	while (!pq.empty()) {
		pair<float, FCellRef> curCellPair = pq.top();
		pq.pop();

		float curDist = curCellPair.first;
		FCellRef curCell = curCellPair.second;
		DistanceMapOut.SetValue(curCell, curDist); //sets the value in the distance map to the current distance from start

		float newDist = curDist + 1.0f;

		//Loop through the left-right-up-down neighbor adjacent cells of the current cell 
		for (const auto& d : directions) {
			int newX = curCell.X + d.first;
			int newY = curCell.Y + d.second;
			FCellRef adjCell = FCellRef(newX, newY);

			//If the neighbor has not already been visisted, is valid, and is traversable, then it is added to visited and added to the priority queue
			auto inVisted = find(visited.begin(), visited.end(), adjCell);
			if (inVisted == visited.end() && DistanceMapOut.GridBounds.IsValidCell(adjCell) && Grid->GetCellData(adjCell) == ECellData::CellDataTraversable) {
				visited.push_back(adjCell);
				pq.push({ newDist, adjCell });
			}
		}
	}

	return false;
}