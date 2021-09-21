/*=============================================================================*/
// Copyright 2017-2018 Elite Engine
// Authors: Matthieu Delaere
/*=============================================================================*/
// ENavigationMeshPathfinder.h: A* Pathfinder implementation for Navigatoin Meshes using IPathfinder
/*=============================================================================*/
#ifndef ELITE_NAVIGATION_MESH_PATHFINDER
#define ELITE_NAVIGATION_MESH_PATHFINDER
namespace Elite
{
	//Portal struct (only contains line info atm, you can expand this if needed)
	struct Portal
	{
		Portal() {}
		explicit Portal(const Line& line) :
			Line(line)
		{}
		Line Line = {};
	};

	class NavigationMeshPathfinder final : public Pathfinder<NavigationMeshNode>
	{
	public:
		//--- Constructor & Destructor ---
		NavigationMeshPathfinder() :Pathfinder() {}
		~NavigationMeshPathfinder() = default; //Non virtual Destructor

		//--- Pathfinder Functions ---
		std::vector<Vector2> FindPath(
			Graph<NavigationMeshNode>* pGraph,
			NavigationMeshNode StartNode,
			NavigationMeshNode EndNode,
			Heuristic heuristicFunction);

		//--- DEBUG VARIABLES ---
#ifdef _DEBUG
		static bool sDrawDebugInfoQueries;
		static bool sDrawPortals;
		static bool sOptimization;
#endif

	private:
		//--- Private Pathfinder Functions ---
		std::vector<Portal> FindPortals(const NavigationMesh* pNavigationMesh, 
			const std::vector<NavigationMeshNode*>& nodePath, 
			const Vector2& startPos, const Vector2& endPos) const;
		std::vector<Vector2> OptimizePortals(const std::vector<Portal>& portals, 
			const Vector2& startPos) const;

		bool ContainsLineIndexTriangle(int lineIndex, const Triangle* pTriangle) const;
		bool Is1LinesOfTriangleReviewed(const Triangle* pTriangle, NavigationMesh* pNavigationMesh) const;
		void AddConnections(const Triangle* pTriangle, Node* pCurrentNode, Node* pStartNode, std::vector<Connection*>& openList, std::vector<Connection*>& closedList, NavigationMesh* pNavigationMesh, Heuristic heuristicFunction, Connection* pCurrentConnection = nullptr, float ProjectOnLineOffset = 1.f);
		bool LegGoingInwardsALT(Elite::Vector2& oldLeg, Elite::Vector2& newLeg, Elite::Vector2& otherLeg, Elite::Vector2& apex) const;
		bool CheckLeg(Elite::Vector2& currentLeg, Elite::Vector2& newLeg, Elite::Vector2& otherLeg, Elite::Vector2& apex, int& currentLegIndex, int& otherLegIndex, int& apexIndex, unsigned int& index, const std::vector<Portal>& portals, std::vector<Elite::Vector2>& vPath) const;

		//Connection Comparison function
		static bool ConnectionComparison(Connection* i, Connection* j) { return i->GetFCost() < j->GetFCost(); }
	};
}
#endif