#ifndef ASTAR_APPLICATION_H
#define ASTAR_APPLICATION_H
//-----------------------------------------------------------------
// Includes & Forward Declarations
//-----------------------------------------------------------------
#include "framework/EliteInterfaces/EIApp.h"
using namespace Elite;

//-----------------------------------------------------------------
// Application
//-----------------------------------------------------------------
class App_PathfindingAStar final : public IApp
{
public:
	//Constructor & Destructor
	App_PathfindingAStar() = default;
	virtual ~App_PathfindingAStar();

	//App Functions
	void Start() override;
	void Update(float deltaTime) override;
	void Render(float deltaTime) const override;

private:
	//Datamembers
	const bool ALLOW_DIAGONAL_MOVEMENT = true;
	bool m_PathFound = false;
	Elite::Vector2 m_StartPosition = Elite::ZeroVector2;
	Elite::Vector2 m_TargetPosition = Elite::ZeroVector2;

	//Grid datamembers
	static const int ROWS = 15;
	static const int COLUMNS = 30;
	unsigned int m_SizeCell = 5;
	Graph<Node>* m_pGridGraph = nullptr;
	Elite::Vector2 m_StartGrid = Elite::ZeroVector2;

	//Pathfinding datamembers
	std::vector<Elite::Vector2> m_vPath;

	//Debug rendering information
	bool m_bDrawGrid = false;
	bool m_bDrawConnections = false;
	int m_SelectedHeuristic = 4;
	Heuristic pHeuristicFunction = HeuristicFunctions::Chebyshev;

	//Functions
	void MakeGridGraph();
	void EvaluateGridGraph();

	//C++ make the class non-copyable
	App_PathfindingAStar(const App_PathfindingAStar&) = delete;
	App_PathfindingAStar& operator=(const App_PathfindingAStar&) = delete;
};
#endif