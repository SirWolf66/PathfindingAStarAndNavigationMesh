//=== General Includes ===
#include "stdafx.h"
#include "EAStarPathfinder.h"
using namespace Elite;

//=== Extra Includes ===
#include <algorithm>  

//=== Pathfinder Functions ===
std::vector<Elite::Vector2> AStarPathfinder::FindPath(
	Graph<Node>* pGraph,
	Node* pStartNode,
	Node* pEndNode,
	Heuristic heuristicFunction)
{
	//Variables
	std::vector<Vector2> vPath; //Path from the start to the end node

	std::vector<Connection*> openList;
	std::vector<Connection*> closedList;

	Connection* pLastConnection = nullptr;

	//Add the connections of the start node to begin
	AddNodeConnectionsToOpenList(pStartNode, pStartNode, pEndNode, heuristicFunction, openList, closedList, &pLastConnection);

	//FindLoop
	while (!openList.empty())
	{
		AddNodeConnectionsToOpenList(openList[0]->GetEndNode(), pStartNode, pEndNode, heuristicFunction, openList, closedList, &pLastConnection, openList[0]);
	}

	if (pLastConnection != nullptr)
	{
		vPath.push_back(pLastConnection->GetEndNode()->GetPosition());
		while (pLastConnection->GetStartNode()->GetPosition() != pStartNode->GetPosition())
		{
			vPath.push_back(pLastConnection->GetStartNode()->GetPosition());
			pLastConnection = pLastConnection->GetHeadConnection();
		}
	}
	
	closedList = std::vector<Connection*>{};

	return vPath;
}

void AStarPathfinder::AddNodeConnectionsToOpenList(Node* node, Node* pStartNode, Node* pEndNode, Heuristic heuristicFunction, std::vector<Connection*>& openList, std::vector<Connection*>& closedList, Connection** pLastConnection, Connection* pCurrentconnection)
{
	std::vector<Connection*> NodeConnections{ node->GetConnections() };
	
	for (unsigned int i{ 0 }; i < NodeConnections.size(); i++)
	{
		//Check if the end is reached
		if (NodeConnections[i]->GetEndNode() == pEndNode)
		{
			openList = std::vector<Connection*>{};
			NodeConnections[i]->SetHeadConnection(pCurrentconnection);
			*pLastConnection = pCurrentconnection;
			return;
		}

		//Check if connection is not in closed or openlist, before adding to openlist
		if (std::find(closedList.begin(), closedList.end(), NodeConnections[i]) == closedList.end() && std::find(openList.begin(), openList.end(), NodeConnections[i]) == openList.end())
		{
			CalculateCosts(NodeConnections[i], pStartNode, pEndNode, heuristicFunction);
			NodeConnections[i]->SetHeadConnection(pCurrentconnection);

			openList.push_back(NodeConnections[i]);
		}
	}

	//If the end isn't reached, add current connection to closedlist and erase it from the openlist
	if (pCurrentconnection != nullptr)
	{
		closedList.push_back(pCurrentconnection);
		openList.erase(openList.begin());
	}

	//Sort the vector of Connection, this will make sure the connection with the lowest F-score is at front
	std::stable_sort(openList.begin(), openList.end(), ConnectionComparison); //Sort the vector, so the connection with the smallest F-score is put at front
}