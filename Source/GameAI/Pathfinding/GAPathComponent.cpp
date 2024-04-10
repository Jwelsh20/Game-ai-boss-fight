#include "GAPathComponent.h"
#include "GameFramework/NavMovementComponent.h"
#include "Kismet/GameplayStatics.h"
#include <queue>
#include <vector>
#include <tuple>
#include <optional>

using namespace std;

UGAPathComponent::UGAPathComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	State = GAPS_None;
	bDestinationValid = false;
	ArrivalDistance = 100.0f;

	// A bit of Unreal magic to make TickComponent below get called
	PrimaryComponentTick.bCanEverTick = true;
}


const AGAGridActor* UGAPathComponent::GetGridActor() const
{
	if (GridActor.Get())
	{
		return GridActor.Get();
	}
	else
	{
		AGAGridActor* Result = NULL;
		AActor *GenericResult = UGameplayStatics::GetActorOfClass(this, AGAGridActor::StaticClass());
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

APawn* UGAPathComponent::GetOwnerPawn()
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


void UGAPathComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	if (bDestinationValid)
	{
		RefreshPath();

		if (State == GAPS_Active)
		{
			FollowPath();
		}
	}

	// Super important! Otherwise, unbelievably, the Tick event in Blueprint won't get called

	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

EGAPathState UGAPathComponent::RefreshPath()
{
	AActor* Owner = GetOwnerPawn();
	FVector StartPoint = Owner->GetActorLocation();

	check(bDestinationValid);

	float DistanceToDestination = FVector::Dist(StartPoint, Destination);

	if (DistanceToDestination <= ArrivalDistance)
	{
		// Yay! We got there!
		State = GAPS_Finished;
	}
	else
	{
		// Replan the path!
		State = AStar();
		//State = GAPS_Active;
	}

	return State;
}

//Helper function for getLineTrace that will get 80 points along the line created from the current FVector location to the FVector location of the goal cell
//Checks to see if there are any obstructions in the way that the robot could run into or if it is an open straight shot to the cell
bool getPointsAlongLine(const FVector& start, const FVector& end, const AGAGridActor* Grid) {
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
tuple<FVector, FCellRef> getLineTrace(const vector<FCellRef>& path, const FCellRef& origin, const AGAGridActor* Grid) {

	FVector originVec = Grid->GetCellPosition(origin);
	FCellRef prev = origin;
	
	//Loop through the A* path and check if cell in the path is directly reachable from the origin without hitting a wall by passing the cell vector to getPointsAlongLine
	for (const auto& element : path) {
		FVector destCellVec = Grid->GetCellPosition(element);

		if (getPointsAlongLine(originVec, destCellVec, Grid)) {
			return make_tuple(Grid->GetCellPosition(prev), prev); //If there is an obstruction found between the robot and the current cell in the loop, the function returns the previous cell that could be reached via a straight line
		}

		prev = element;
	}
	
	return make_tuple(Grid->GetCellPosition(path.back()), path.back()); //If all cells return false that means all are reachable via a straight line and it returns the end of the path (i.e. the player's cell) as the one to point towards
}

//Helper function for CompareCells operator
//Calculates the linear distance between two cells on the grid
double calculateDistance(const FCellRef& point1, const FCellRef& point2) {
	int deltaX = point2.X - point1.X;
	int deltaY = point2.Y - point1.Y;

	return sqrt(deltaX * deltaX + deltaY * deltaY);
}

//Struct to compare cells to be sorted in priority queue
struct CompareCells {
	const FCellRef& DestinationCell;

	CompareCells(const FCellRef& destinationCell) : DestinationCell(destinationCell) {}

	//operator to compare and sort cells based on distance to DestinationCell
	bool operator()(const tuple<FCellRef, vector<FCellRef>>& lhs, const tuple<FCellRef, vector<FCellRef>>& rhs) const {
		double lhsDist = calculateDistance(get<0>(lhs), DestinationCell);
		double rhsDist = calculateDistance(get<0>(rhs), DestinationCell);
		return lhsDist > rhsDist;  // Min-heap based on dist
	}
};

//A* search function
EGAPathState UGAPathComponent::AStar()
{
	const AGAGridActor* Grid = GetGridActor();

	Steps.SetNum(1);
	
	//Get the current location of the robot and set it as the startCell
	AActor* Owner = GetOwnerPawn();
	FVector StartPoint = Owner->GetActorLocation();
	FCellRef startCell = Grid->GetCellRef(StartPoint);

	//Makes a priority queue of tuples of <FCellRef, vector<FCellRef>>. This represents the current cell being searched and the path leading from the start to that current cell. And CompareCells is passed in to sort them based on distance to the player
	CompareCells compareCellsInstance(DestinationCell);
	priority_queue<tuple<FCellRef, vector<FCellRef>>, vector<tuple<FCellRef, vector<FCellRef>>>, CompareCells> pq(compareCellsInstance);
	vector<FCellRef> startVector;
	pq.push(make_tuple(startCell, startVector));
	vector<FCellRef> visited;
	visited.push_back(startCell);
	//Boolean to check if a path has been found. This is only relevenat if the player jumps ontop of either of the two square structures that have no ramp leading up to them so there is no possible path for the robot to reach the player. 
	bool pathFound = false;

	while (!pq.empty()) {
		tuple<FCellRef, vector<FCellRef>> curCellTuple = pq.top();
		pq.pop();

		FCellRef curCell = get<0>(curCellTuple);
		vector<FCellRef> curPath = get<1>(curCellTuple);

		//priority queue pops the top element which is the closest to the player and converts that cell and the destinationCell to FVector2D
		FVector2D curCell2D = Grid->GetCellGridSpacePosition(curCell);
		FVector2D destCell2D = Grid->GetCellGridSpacePosition(DestinationCell);
		
		//If the current cell is within the arrival distance we have found the optimal path
		if (FVector2D::Distance(curCell2D, destCell2D) <= ArrivalDistance) {

			if (curPath.size() >= 2) {
				UWorld* World = GetWorld();
				tuple<FVector, FCellRef> moveToTuple = getLineTrace(curPath, startCell, Grid);
				if (Grid->GetCellData(get<1>(moveToTuple)) == ECellData::CellDataTraversable) {
					Steps[0].Set(FVector2D(get<0>(moveToTuple)), get<1>(moveToTuple));
				}
			}
			else {
				//When the robot finds the player, the new paths generated will be only 1 cell in length and calling index 1 will cause a crash
				//This allows the player to start moving again and the robot will follow the player straight away then switch back to A* as soon as enough distance is created or an obstacle is in the way
				Steps[0].Set(FVector2D(StartPoint), startCell);
			}

			//Sets pathFound to true and breaks out of the while loop
			pathFound = true;
			break;
		}

		//If not path was found the current cell is added to the existing path
		curPath.push_back(curCell);

		list<list<int>> directions = {
			{0,1},
			{0,-1},
			{1,0},
			//{1,1},
			//{1,-1},
			//{-1,1},
			{-1,0},
			//{-1,-1},
		};

		//Loop through the left-right-up-down neighbor adjacent cells of the current cell 
		for (const list<int>& d : directions) {
			auto it2 = d.begin();
			int newX = curCell.X + *it2;
			++it2;
			int newY = curCell.Y + *it2;

			FCellRef adjCell = FCellRef(newX, newY);

			//If the neighbor has not already been visisted, is valid, and is traversable, then it is added to visited and added to the priority queue
			auto inVisted = find(visited.begin(), visited.end(), adjCell);
			if (inVisted == visited.end() && adjCell.IsValid() && Grid->GetCellData(adjCell) == ECellData::CellDataTraversable) {
				visited.push_back(adjCell);
				pq.push(make_tuple(adjCell, curPath));
			}
		}
	}

	//in the event the player is in a "glitch spot" where there is no possible path to be found as the player is standing in an area surrounded by non-traversible cells
	//in this case it seems reasonable to just point the robot at the player because without this the robot was standing in place without trying to locate the player
	if (!pathFound) {
		//Steps[0].Set(FVector2D(Destination), DestinationCell);

		//I was originially pointing the robot in the direction of the player, but that was causing some edge case issues if the robot got too close to the edge and would end up in an untraversible cell. Now the robot will just stop moving in this event.
	} 

	return GAPS_Active;
}

void UGAPathComponent::FollowPath()
{
	AActor* Owner = GetOwnerPawn();
	FVector StartPoint = Owner->GetActorLocation();

	check(State == GAPS_Active);
	check(Steps.Num() > 0);

	// Always follow the first step, assuming that we are refreshing the whole path every tick
	FVector V = FVector(Steps[0].Point, 0.0f) - StartPoint;
	V.Normalize();

	UNavMovementComponent* MovementComponent = Owner->FindComponentByClass<UNavMovementComponent>();
	if (MovementComponent)
	{
		MovementComponent->RequestPathMove(V);
	}
}


FVector UGAPathComponent::GetRandomAccessiblePosition()
{
	const AGAGridActor* Grid = GetGridActor();
	FVector CurrentLocation = GetOwnerPawn()->GetActorLocation();

	int maxAttempts = 10000;
	int attempt = 0;

	while (attempt < maxAttempts) {
		int32 RandomX = FMath::RandRange(CurrentLocation.X - 2000, CurrentLocation.X + 2000);
		int32 RandomY = FMath::RandRange(CurrentLocation.Y - 2000, CurrentLocation.Y + 2000);

		FVector Candidate = FVector(RandomX, RandomY, CurrentLocation.Z);

		FCellRef CellRef = Grid->GetCellRef(Candidate);

		if (Grid->GetCellData(CellRef) == ECellData::CellDataTraversable) {
			return Candidate;
		}
		attempt++;
	}

	return FVector();
}

EGAPathState UGAPathComponent::SetDestination(const FVector &DestinationPoint)
{
	Destination = DestinationPoint;

	State = GAPS_Invalid;
	bDestinationValid = true;

	const AGAGridActor* Grid = GetGridActor();
	if (Grid)
	{
		FCellRef CellRef = Grid->GetCellRef(Destination);
		if (CellRef.IsValid())
		{
			DestinationCell = CellRef;
			bDestinationValid = true;

			RefreshPath();
		}
	}

	return State;
}