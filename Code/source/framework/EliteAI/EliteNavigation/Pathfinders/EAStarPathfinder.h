/*=============================================================================*/
// Copyright 2017-2018 Elite Engine
// Authors: Matthieu Delaere
/*=============================================================================*/
// EAStarPathfinder.h: A* Pathfinder implementation using IPathfinder
/*=============================================================================*/
#ifndef ELITE_NAVIGATION_ASTARPATHFINDER
#define ELITE_NAVIGATION_ASTARPATHFINDER
namespace Elite
{
	class AStarPathfinder final : public Pathfinder<Node>
	{
	public:
		//--- Constructor & Destructor ---
		AStarPathfinder() :Pathfinder() {}
		~AStarPathfinder() = default; //Non virtual Destructor

		//--- Pathfinder Functions ---
		std::vector<Elite::Vector2> FindPath(
			Graph<Node>* pGraph,
			Node* pStartNode,
			Node* pEndNode,
			Heuristic heuristicFunction);

	private:
		void AddNodeConnectionsToOpenList(Node* node, Node* pStartNode, Node* pEndNode, Heuristic heuristic, std::vector<Connection*>& openList, std::vector<Connection*>& closedList, Connection** pLastConnection, Connection* pCurrentconnection = nullptr);
	
		static bool ConnectionComparison(Connection* i, Connection* j) { return i->GetFCost() < j->GetFCost(); }

		//Debugging
		void PrintConnectionList(std::vector<Connection*>& list)
		{
			for (unsigned int i{}; i < list.size(); i++)
			{
				std::cout << list[i]->GetFCost() << ", ";
			}
			std::cout << "\n";
		}
	};
}
#endif