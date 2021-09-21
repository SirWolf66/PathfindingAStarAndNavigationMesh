//=== General Includes ===
#include "stdafx.h"
#include "ENavigationMeshPathfinder.h"
using namespace Elite;

//=== Static Debug Variable Initialization ===
#ifdef _DEBUG
bool NavigationMeshPathfinder::sDrawDebugInfoQueries = false;
bool NavigationMeshPathfinder::sDrawPortals = false;
bool NavigationMeshPathfinder::sOptimization = false;
#endif

//=== Pathfinder Functions ===
std::vector<Elite::Vector2> NavigationMeshPathfinder::FindPath(
	Graph<NavigationMeshNode>* pGraph,
	NavigationMeshNode StartNode,
	NavigationMeshNode EndNode,
	Heuristic heuristicFunction)
{
	//Variables
	std::vector<Elite::Vector2> vPath;
	NavigationMesh* pNavigationMesh = reinterpret_cast<NavigationMesh*>(pGraph);
	if (pNavigationMesh == nullptr)
	{
		std::cout << "ERROR: searching path on NavigationMesh without passing a NavigationMesh Graph!" << std::endl;
		return vPath;
	}


	//--------------- STEP 1: PREPARATIONS ---------------
	// - Resetting the Graph, adding initial path nodes & setting up variables
	// - Kickstarting the while loop by creating nodes/connections from the startTriangle
	//--------------------------------------------
	//....


	//Reset navMesh
	pNavigationMesh->ResetNavigationMesh();

	//Add the start and endNode
	NavigationMeshNode* pStartNode = pNavigationMesh->AppendNode(StartNode);
	NavigationMeshNode* pEndNode = pNavigationMesh->AppendNode(EndNode);

	//A* containers and variables
	std::vector<Connection*> openList;
	std::vector<Connection*> closedList;
	Connection* pCurrentConnection = nullptr;

	//Find the triangles, where the Startnode and Endnode reside in
	const Triangle* pStartTriangle{ pNavigationMesh->GetCurrentTriangleFromPosition(pStartNode->GetPosition()) };
	const Triangle* pEndTriangle{ pNavigationMesh->GetCurrentTriangleFromPosition(pEndNode->GetPosition()) };

	//Check if the start and end triangles are found, if not get the closest triangle
	if (pStartTriangle == nullptr)
	{
		pStartTriangle = pNavigationMesh->GetClosestTriangle(pStartNode->GetPosition());
	}

	if (
		pEndTriangle == nullptr)
	{
		pEndTriangle = pNavigationMesh->GetClosestTriangle(pEndNode->GetPosition());
	}

	//Check again if the start and end triangles are found, if one of them isn't found, then return vPath
	if (pStartTriangle == nullptr || pEndTriangle == nullptr)
	{
		return vPath;
	}

	//Check if the start and end triangle are the same, if they are the same push the endNode's position and return the Path
	if (pStartTriangle == pEndTriangle)
	{
		vPath.push_back(pEndNode->GetPosition());
		return vPath;
	}

	//Kick starting the algoritmn by adding the first connections( lines ) to the open list
	AddConnections(pStartTriangle, pStartNode, pStartNode, openList, closedList, pNavigationMesh, heuristicFunction);

	/*//const std::vector<const Line*> StartConnectionsVector{ pNavigationMesh->GetPlausibleLines(pStartTriangle) };

	//for (int i{}; i < StartConnectionsVector.size(); i++)
	//{
	//	const Line* pSegment{ StartConnectionsVector[i] };

	//	Elite::Vector2 projectedPointOnLine{ ProjectOnLineSegment(pSegment->p1, pSegment->p2, pStartNode->GetPosition(), 1.f) };
	//	NavigationMeshNode* navigationMeshNode{ new NavigationMeshNode{projectedPointOnLine, pSegment->index, pStartTriangle} };

	//	pStartNode->AddConnection(static_cast<Node*>(navigationMeshNode));
	//	pNavigationMesh->SetLineReviewed(pSegment->index);

	//	// Turn the NavigationNode in a connection and calculate the G-and H-score pf the connection and add to the openlist	
	//	Connection* pCurrentConnection{ new Connection{pStartNode, static_cast<Node*>(navigationMeshNode) } };
	//	CalculateCosts(pCurrentConnection, pStartNode, pCurrentConnection->GetEndNode(), heuristicFunction);
	//	openList.push_back(pCurrentConnection);
	//}*/


	//--------------- CALCULATIONS ---------------
	//Do the actual path calculations. We generate our "graph" (== query) on the fly, 
	//using the A* (heuristics) algorithm to determine next connections.
	//Generate query on the fly - expanding based on the current closest node (through connection).
	//We loop while we still have nodes in the openlist.
	//We stop when we hit the triangle that holds the targetNode 
	while (!openList.empty())
	{
		//Get the first node in the openlist, the list is sorted, so this means that the node with lowest F-score is in front
		pCurrentConnection = openList[0];

		//Add the Node that you are evaluating to the closed list
		closedList.push_back(pCurrentConnection);
		openList.erase(openList.begin());

		//Check if the connection leads to the triangle where the Endnode resides in
		NavigationMeshNode* pCurrentConnectionEndNode{ static_cast<NavigationMeshNode*>(pCurrentConnection->GetEndNode()) };
		int endNodeLineIndex{ pCurrentConnectionEndNode->GetAssociatedLineIndex() };


		if (ContainsLineIndexTriangle(endNodeLineIndex, pEndTriangle))
		{
			pCurrentConnectionEndNode->Initialize(); //Initialize the connection pool
			Connection* pLastConnection{ pCurrentConnectionEndNode->AddConnection(pEndNode) };
			pLastConnection->SetHeadConnection(pCurrentConnection);
			openList = std::vector<Connection*>{};
			continue;
		}

		//Finding a new triangle that is connected to the End Node's lineIndex
		const Triangle* pCurrenTriangle{};
		const std::vector<const Triangle*> newTrianglesVector{ pNavigationMesh->GetTrianglesFromLineIndex(endNodeLineIndex) };

		if (newTrianglesVector.size() == 1)
		{
			pCurrenTriangle = newTrianglesVector[0];
		}
		else if (newTrianglesVector.size() == 2)
		{
			for (unsigned int i{}; i < newTrianglesVector.size(); i++)
			{
				if (Is1LinesOfTriangleReviewed(newTrianglesVector[0], pNavigationMesh))
				{
					pCurrenTriangle = newTrianglesVector[0];

				}
				else
				{
					pCurrenTriangle = newTrianglesVector[1];
				}
			}
		}
		else
		{
			std::cout << "Whoops, it appears no new triangles are found \n";

		}

		if ((pCurrenTriangle == nullptr))
		{
			closedList.push_back(pCurrentConnection);
			openList.erase(openList.begin());
			continue;
		}
		
		//2E
		//Projecting the current position(pCurrentPositionEndNode) to the newly found triangle edges
		AddConnections(pCurrenTriangle, pCurrentConnectionEndNode, pStartNode, openList, closedList, pNavigationMesh, heuristicFunction, pCurrentConnection);
	}

	//--- DEBUG RENDERING
#ifdef _DEBUG
	if (sDrawDebugInfoQueries)
	{
		auto pN = pGraph->GetNodes();
		for (auto pN : pGraph->GetNodes())
		{
			auto pC = pN->GetConnections();
			for (auto pC : pN->GetConnections())
			{
				DEBUGRENDERER2D->DrawPoint(pC->GetStartNode()->GetPosition(), 5.0f, { 0,1.f,1.f }, 0.3f);
				DEBUGRENDERER2D->DrawPoint(pC->GetEndNode()->GetPosition(), 5.0f, { 0,1.f,1.f }, 0.3f);
				DEBUGRENDERER2D->DrawSegment(pC->GetStartNode()->GetPosition(),
					pC->GetEndNode()->GetPosition(), { 0,1.f,1.f }, 0.3f);
			}
		}

	}
#endif

	//IMPORTANT: The code below is given as a gift. If you reach this point, you will get a path that is not the optimal path but the agent will already get to the end point.
	//Finish the OptimizePortals() functions to get an optimized path!!
	//--------------- RECONSTRUCT ---------------
	//Retracing the path and store this path in a container.
	//--------------------------------------------
	std::vector<NavigationMeshNode*> vPathNodes;
	//pCurrentConnection && is a safety check for when you run before implementing the actual pathfinder
	if (pCurrentConnection)
	{
		while (pCurrentConnection->GetStartNode() != pStartNode)
		{
			vPathNodes.push_back(static_cast<NavigationMeshNode*>(pCurrentConnection->GetEndNode()));
			pCurrentConnection = pCurrentConnection->GetHeadConnection();
		}
		vPathNodes.push_back(static_cast<NavigationMeshNode*>(pCurrentConnection->GetEndNode()));
		std::reverse(vPathNodes.begin(), vPathNodes.end());
	}
	//--------------- OPTIMIZE ---------------
	//Retrace the path and store this path in a container. Then optimize it using the funnel algorithm and
	//return the optimized path! 
	//--------------------------------------------
	//COMMENT THE FOLLOWING LINE OF CODE (#define NOT_OPTIMIZED) TO ENABLE THE OPTIMIZED PART!
	//Otherwise it will just copy over the nodes so you can test earlier functionality
//#define NOT_OPTIMIZED
#/*ifdef NOT_OPTIMIZED
	auto const portals = FindPortals(pNavigationMesh, vPathNodes, pStartNode->GetPosition(), pEndNode->GetPosition());
	for (auto n : vPathNodes)
		vPath.push_back(n->GetPosition());
#else
	//Optimize our path using funnel algorithm!
	auto const portals = FindPortals(pNavigationMesh, vPathNodes, pStartNode->GetPosition(), pEndNode->GetPosition());
	vPath = OptimizePortals(portals, pStartNode->GetPosition());
	vPath.push_back(vPathNodes[vPathNodes.size() - 1]->GetPosition()); //Push goal as part of the path
#endif
	return vPath;*/

	//Moved the optimizzation but to an if statement coupled to a bool that is controlled by a checkbox
	if (sOptimization)
	{
		//Optimize our path using funnel algorithm!
		auto const portals = FindPortals(pNavigationMesh, vPathNodes, pStartNode->GetPosition(), pEndNode->GetPosition());
		vPath = OptimizePortals(portals, pStartNode->GetPosition());
		vPath.push_back(vPathNodes[vPathNodes.size() - 1]->GetPosition()); //Push goal as part of the path
	}
	else
	{
	
		auto const portals = FindPortals(pNavigationMesh, vPathNodes, pStartNode->GetPosition(), pEndNode->GetPosition());
		for (auto n : vPathNodes)
			vPath.push_back(n->GetPosition());
	}

	return vPath;
}

//=== Private Pathfinder Functions ===
//--- References ---
//http://digestingduck.blogspot.be/2010/03/simple-stupid-funnel-algorithm.html
//https://gamedev.stackexchange.com/questions/68302/how-does-the-simple-stupid-funnel-algorithm-work
std::vector<Elite::Vector2> NavigationMeshPathfinder::OptimizePortals(const std::vector<Portal>& portals,
	const Elite::Vector2& startPos) const
{
	//P1 == right point of portal, P2 == left point of portal
	std::vector<Vector2> vPath = {};
	Vector2 apex = startPos;
	int apexIndex = 0, leftLegIndex = 0, rightLegIndex = 0;
	Vector2 rightLeg = portals[rightLegIndex].Line.p1 - apex;
	Vector2 leftLeg = portals[leftLegIndex].Line.p2 - apex;

	for (unsigned int i = 1; i < static_cast<unsigned int>(portals.size()); ++i)
	{
		//--- GET CURRENT PORTAL ---
		Portal pCurrentPortal{ portals[i]};

		//--- RIGHT CHECK ---
		Vector2 newRightLeg{ pCurrentPortal.Line.p1 - apex };

		//LegGoingInwardsALT(rightLeg, newRightLeg, leftLeg, apex);

		if (CheckLeg(rightLeg, newRightLeg, leftLeg, apex, rightLegIndex, leftLegIndex, apexIndex, i, portals, vPath))
		{
			rightLeg = portals[rightLegIndex].Line.p1 - apex;
			leftLeg = portals[leftLegIndex].Line.p2 - apex;
			continue;
		}

		//--- LEFT CHECK ---
		Vector2 newLeftLeg{ pCurrentPortal.Line.p2 - apex };
		if (CheckLeg( leftLeg, newLeftLeg, rightLeg, apex, leftLegIndex, rightLegIndex, apexIndex, i, portals, vPath))
		{
			rightLeg = portals[rightLegIndex].Line.p1 - apex;
			leftLeg = portals[leftLegIndex].Line.p2 - apex;
			continue;
		}

	}
	return vPath;
}

std::vector<Portal> NavigationMeshPathfinder::FindPortals(const NavigationMesh* pNavigationMesh,
	const std::vector<NavigationMeshNode*>& nodePath,
	const Elite::Vector2& startPos, const Elite::Vector2& endPos) const
{
	//Container
	std::vector<Portal> vPortals = {};

	//For each node received, get it's corresponding line
	const Triangle* pTriangle = nullptr;
	for (size_t i = 0; i < nodePath.size() - 1; ++i)
	{
		//Local variables
		auto pNode = nodePath[i]; //Store node, except last node, because this is our target node!
		auto pLine = pNavigationMesh->GetPolygon()->GetLines()[pNode->GetAssociatedLineIndex()];

		//Redetermine it's "orientation" based on the required path (left-right vs right-left) - p1 should be right point
		auto centerLine = (pLine->p1 + pLine->p2) / 2.0f;
		//Find the corresponding triangled based on node linking
		pTriangle = pNode->GetTraversedTriangle();
		auto centerTriangle = pTriangle->GetCenter();
		auto cp = Cross((centerLine - centerTriangle), (pLine->p1 - centerTriangle));
		Line portalLine = {};
		if (cp > 0)//Left
			portalLine = Line(pLine->p2, pLine->p1);
		else //Right
			portalLine = Line(pLine->p1, pLine->p2);

		//Store portal
		vPortals.push_back(Portal(portalLine));
	}
	//Add degenerate portal to force end evaluation
	vPortals.push_back(Portal(Line(endPos, startPos)));

#ifdef _DEBUG
	if (sDrawPortals)
	{
		for (auto portal : vPortals)
			DEBUGRENDERER2D->DrawSegment(portal.Line.p1, portal.Line.p2, Color(1.f, .5f, 0.f));
	}
#endif

	return vPortals;
}


#pragma region Private helper functions
bool NavigationMeshPathfinder::ContainsLineIndexTriangle(int lineIndex, const Triangle* pTriangle) const
{
	const int triangleNodesAmount{ 3 };
	bool lineIndexFound{ false };

	//Using OR gates
	lineIndexFound = lineIndexFound || (lineIndex == pTriangle->metaData.IndexLine1);
	lineIndexFound = lineIndexFound || (lineIndex == pTriangle->metaData.IndexLine2);
	lineIndexFound = lineIndexFound || (lineIndex == pTriangle->metaData.IndexLine3);

	return lineIndexFound;
}

bool NavigationMeshPathfinder::Is1LinesOfTriangleReviewed(const Triangle* pTriangle, NavigationMesh* pNavigationMesh) const
{
	bool linesarereviewed{ false };
	bool firstLineReviewed{ false };

	//Using XOR gates
	firstLineReviewed = firstLineReviewed ^ pNavigationMesh->IsLineReviewed(pTriangle->metaData.IndexLine1);
	linesarereviewed = firstLineReviewed ^ pNavigationMesh->IsLineReviewed(pTriangle->metaData.IndexLine2); //XOR gates work until here, but in the third case it will  fuck up if it is set to false here

	linesarereviewed = (linesarereviewed || firstLineReviewed) ^ pNavigationMesh->IsLineReviewed(pTriangle->metaData.IndexLine3); //To combat the problem, use an OR between the first and the second line, so if either of them is turned to true, the XOR gate will get 2 true's, resulting into false;

	//Using some if statements
	/*if (pNavigationMesh->IsLineReviewed(pTriangle->metaData.IndexLine1))
	{
		linesarereviewed = true;
	}

	if (pNavigationMesh->IsLineReviewed(pTriangle->metaData.IndexLine2) && !linesarereviewed)
	{
		linesarereviewed = true;
	}
	else if(pNavigationMesh->IsLineReviewed(pTriangle->metaData.IndexLine2) && linesarereviewed)
	{
		return false;
	}

	if (pNavigationMesh->IsLineReviewed(pTriangle->metaData.IndexLine3) && !linesarereviewed)
	{
		linesarereviewed = true;
	}
	else if (pNavigationMesh->IsLineReviewed(pTriangle->metaData.IndexLine3) && linesarereviewed)
	{
		return false;
	}*/

	return linesarereviewed;
}

void NavigationMeshPathfinder::AddConnections(const Triangle* pTriangle, Node* pCurrentNode, Node* pStartNode, std::vector<Connection*>& openList, std::vector<Connection*>& closedList, NavigationMesh* pNavigationMesh, Heuristic heuristicFunction, Connection* pCurrentConnection, float ProjectOnLineOffset)
{
	const std::vector<const Line*> ConnectionsVector{ pNavigationMesh->GetPlausibleLines(pTriangle) };
	//NavigationMeshNode* navigationMeshNode{};

	for (unsigned int i{}; i < ConnectionsVector.size(); i++)
	{
		const Line* pSegment{ ConnectionsVector[i] };

		if (!pNavigationMesh->IsLineReviewed(pSegment->index))
		{
			Elite::Vector2 projectedPointOnLine{ ProjectOnLineSegment(pSegment->p1, pSegment->p2, pCurrentNode->GetPosition(), ProjectOnLineOffset) };
			NavigationMeshNode navigationMeshNode{ projectedPointOnLine, pSegment->index, pTriangle };
			pNavigationMesh->AppendNode(navigationMeshNode);

			pCurrentNode->Initialize(); //Initialize the connection pool
			pCurrentNode->AddConnection( pNavigationMesh->GetNodes()[ pNavigationMesh->GetNodes().size() - 1 ] );

			pCurrentNode->GetConnections()[pCurrentNode->GetConnections().size() - 1]->SetHeadConnection(pCurrentConnection);

			pNavigationMesh->SetLineReviewed(pSegment->index);
		}
	}

	//A*star part
	std::vector<Connection*> pCurrenNodeConnections{ pCurrentNode->GetConnections() };

	for (unsigned int i{}; i < pCurrenNodeConnections.size(); i++)
	{
		if (std::find(closedList.begin(), closedList.end(), pCurrenNodeConnections[i]) == closedList.end() )
		{
			//Set the head Connection and Calculate the G-and H-score of the connection and add to the openlist	
			CalculateCosts(pCurrenNodeConnections[i], pStartNode, pCurrenNodeConnections[i]->GetEndNode(), heuristicFunction);

			//pCurrenNodeConnections[i]->SetHeadConnection(pCurrentConnection);

			openList.push_back(pCurrenNodeConnections[i]);
		}
	}

	//If the end isn't reached, add current connection to closedlist and erase it from the openlist
	if (pCurrentConnection != nullptr)
	{
		//closedList.push_back(pCurrentConnection);
		//openList.erase(openList.begin());
	}

	//Sort the Connections, so the Connection with the lowest F-Score is put at the front
	std::stable_sort(openList.begin(), openList.end(), ConnectionComparison);
}

//An alternative calculation to using cross, example: normalise the old right leg and the new right leg and see which one is closer to the leftleg
bool NavigationMeshPathfinder::LegGoingInwardsALT(Elite::Vector2& oldLeg, Elite::Vector2& newLeg, Elite::Vector2& otherLeg, Elite::Vector2& apex) const
{
	/*//float crossProduct{ Cross(v1, v2) };

	//float angleInDegrees = asinf(crossProduct);

	//return asinf(crossProduct);*/
	
	Elite::Vector2 NormalOld{ oldLeg.GetNormalized() };
	Elite::Vector2 NormalNew{ newLeg.GetNormalized() };
	Elite::Vector2 NormalOther{ otherLeg.GetNormalized() };

	Elite::Vector2 OtherNewDifference{ NormalOther - NormalNew };
	Elite::Vector2 OtherOldDifference{ NormalOther - NormalOld };

	float DistanceOtherNew{ sqrtf(powf(OtherNewDifference.x, 2) + powf(OtherNewDifference.y, 2)) };
	float DistanceOtherOld{ sqrtf(powf(OtherOldDifference.x, 2) + powf(OtherOldDifference.y, 2)) };

	if (DistanceOtherNew < DistanceOtherOld)
	{
		return true;
	}

	return false;
}

bool NavigationMeshPathfinder::CheckLeg(Elite::Vector2& currentLeg, Elite::Vector2& newLeg, Elite::Vector2& otherLeg, Elite::Vector2& apex, int& currentLegIndex, int& otherLegIndex, int& apexIndex, unsigned int& index, const std::vector<Portal>& portals, std::vector<Elite::Vector2>& vPath) const
{
	//Problem lies with the crossing i thinl, then again with the alternative caclculations i get the exact same result
	//if( LegGoingInwardsALT(currentLeg, newLeg, otherLeg, apex )
	if (currentLeg.Cross(newLeg) > 0.f) // If the angle between the currenleg and the newleg is positive, then the newleg is inside the funnel
	{
		//if (LegGoingInwardsALT(otherLeg, newLeg, currentLeg, apex)
		if ( newLeg.Cross(otherLeg) > 0.f ) // If the angle between the newleg and the otherleg is positive, then the newleg does not cross the other leg 
		{
			currentLeg = newLeg;
			currentLegIndex = index;
		}
		else
		{
			apex += otherLeg;
			apexIndex = otherLegIndex;
			int newIterator{ otherLegIndex + 1 };

			//Setting new iterator
			index = newIterator;
			otherLegIndex = newIterator;
			currentLegIndex = newIterator;

			//Store current apexpoint as part of the pathd
			//vPath.push_back(apex);

			if ( newIterator < static_cast<int>(portals.size() ))
			{
				//return true;
			}
		}
	}

	return false;
}
#pragma endregion