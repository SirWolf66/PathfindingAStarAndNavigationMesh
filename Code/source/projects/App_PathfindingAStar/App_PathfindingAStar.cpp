//Precompiled Header [ALWAYS ON TOP IN CPP]
#include "stdafx.h"

//Includes
#include "App_PathfindingAStar.h"
#include "../App_Steering/SteeringAgent.h"

//Destructor
App_PathfindingAStar::~App_PathfindingAStar()
{
	SAFE_DELETE(m_pGridGraph);
}

//Functions
void App_PathfindingAStar::Start()
{
	//Set Camera
	DEBUGRENDERER2D->GetActiveCamera()->SetZoom(39.0f);
	DEBUGRENDERER2D->GetActiveCamera()->SetCenter(Elite::Vector2(73.0f, 35.0f));
	DEBUGRENDERER2D->GetActiveCamera()->SetMoveLocked(true);
	DEBUGRENDERER2D->GetActiveCamera()->SetZoomLocked(true);

	//Create and Evaluate Graph
	MakeGridGraph();
	EvaluateGridGraph();
}

void App_PathfindingAStar::Update(float deltaTime)
{
	UNREFERENCED_PARAMETER(deltaTime);

	//INPUT
	bool const leftMouseButtonPressed = INPUTMANAGER->IsMouseButtonUp(InputMouseButton::eLeft);
	bool const rightMouseButtonPressed = INPUTMANAGER->IsMouseButtonUp(InputMouseButton::eRight);
	if (leftMouseButtonPressed || rightMouseButtonPressed)
	{
		MouseData mouseData = {};
		if (leftMouseButtonPressed)
			mouseData = INPUTMANAGER->GetMouseData(Elite::InputType::eMouseButton, Elite::InputMouseButton::eLeft);
		else if (rightMouseButtonPressed)
			mouseData = INPUTMANAGER->GetMouseData(Elite::InputType::eMouseButton, Elite::InputMouseButton::eRight);
		else
			return;
		Elite::Vector2 const pos = Elite::Vector2(static_cast<float>(mouseData.X), static_cast<float>(mouseData.Y));

		//Find closest node for target as well
		auto const closestNode = m_pGridGraph->GetClosestNode(
			DEBUGRENDERER2D->GetActiveCamera()->ConvertScreenToWorld(pos),
			Elite::HeuristicFunctions::SqrtEuclidean);

		//Set target or start position based on mouse button
		if (leftMouseButtonPressed)
		{
			m_TargetPosition = closestNode->GetPosition();
			m_PathFound = false;
		}

		else if (rightMouseButtonPressed)
		{
			m_StartPosition = closestNode->GetPosition();
			m_PathFound = false;
		}
	}

#ifdef PLATFORM_WINDOWS
#pragma region UI
	//UI
	{
		//Setup
		int menuWidth = 115;
		int const width = DEBUGRENDERER2D->GetActiveCamera()->GetWidth();
		int const height = DEBUGRENDERER2D->GetActiveCamera()->GetHeight();
		bool windowActive = true;
		ImGui::SetNextWindowPos(ImVec2((float)width - menuWidth - 10, 10));
		ImGui::SetNextWindowSize(ImVec2((float)menuWidth, (float)height - 20));
		ImGui::Begin("Gameplay Programming", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
		ImGui::PushAllowKeyboardFocus(false);

		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: target");
		ImGui::Text("RMB: start");
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("A* Pathfinding");
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Checkbox("Grid", &m_bDrawGrid);
		ImGui::Checkbox("Connections", &m_bDrawConnections);
		if (ImGui::Combo("", &m_SelectedHeuristic, "Manhattan\0Euclidean\0SqrtEuclidean\0Octile\0Chebyshev", 4))
		{
			switch (m_SelectedHeuristic)
			{
			case 0:
				pHeuristicFunction = HeuristicFunctions::Manhattan;
				break;
			case 1:
				pHeuristicFunction = HeuristicFunctions::Euclidean;
				break;
			case 2:
				pHeuristicFunction = HeuristicFunctions::SqrtEuclidean;
				break;
			case 3:
				pHeuristicFunction = HeuristicFunctions::Octile;
				break;
			case 4:
				pHeuristicFunction = HeuristicFunctions::Chebyshev;
				break;
			default:
				pHeuristicFunction = HeuristicFunctions::Chebyshev;
				break;
			}
		}
		ImGui::Spacing();

		//End
		ImGui::PopAllowKeyboardFocus();
		ImGui::End();
	}
#pragma endregion
#endif

	//Finding Nodes
	Node* pTargetNode = m_pGridGraph->GetClosestNode(m_TargetPosition, Elite::HeuristicFunctions::SqrtEuclidean);
	Node* pClosestStartNode = m_pGridGraph->GetClosestNode(m_StartPosition,
		Elite::HeuristicFunctions::SqrtEuclidean);

	//If we have nodes and the target is not the startNode, find a path!
	if ((pTargetNode && pClosestStartNode) &&  pTargetNode != pClosestStartNode && !m_PathFound)
	{
		//Pathfinding
		Elite::AStarPathfinder pathFinder = {};
		m_vPath = pathFinder.FindPath(m_pGridGraph, pClosestStartNode, pTargetNode,
			pHeuristicFunction);
		m_PathFound = true;
	}
}

void App_PathfindingAStar::Render(float deltaTime) const
{
	UNREFERENCED_PARAMETER(deltaTime);

	//Render the start position
	Elite::Vector2 vertsStart[4]
	{
		Elite::Vector2(m_StartPosition.x - m_SizeCell / 2.0f, m_StartPosition.y - m_SizeCell / 2.0f),
		Elite::Vector2(m_StartPosition.x - m_SizeCell / 2.0f, m_StartPosition.y + m_SizeCell / 2.0f),
		Elite::Vector2(m_StartPosition.x + m_SizeCell / 2.0f, m_StartPosition.y + m_SizeCell / 2.0f),
		Elite::Vector2(m_StartPosition.x + m_SizeCell / 2.0f, m_StartPosition.y - m_SizeCell / 2.0f)
	};
	DEBUGRENDERER2D->DrawSolidPolygon(&vertsStart[0], 4, Color(0.0f, 1.f, 0.0f, 1.0f), 0.7f);

	//Render the target
	Elite::Vector2 vertsTarget[4]
	{
		Elite::Vector2(m_TargetPosition.x - m_SizeCell / 2.0f, m_TargetPosition.y - m_SizeCell / 2.0f),
		Elite::Vector2(m_TargetPosition.x - m_SizeCell / 2.0f, m_TargetPosition.y + m_SizeCell / 2.0f),
		Elite::Vector2(m_TargetPosition.x + m_SizeCell / 2.0f, m_TargetPosition.y + m_SizeCell / 2.0f),
		Elite::Vector2(m_TargetPosition.x + m_SizeCell / 2.0f, m_TargetPosition.y - m_SizeCell / 2.0f)
	};
	DEBUGRENDERER2D->DrawSolidPolygon(&vertsTarget[0], 4, Color(1.0f, 0.f, 0.0f, 1.0f), 0.7f);

	//Render non-walkable grid
	auto nodes = m_pGridGraph->GetNodes();
	for (auto pNode : nodes)
	{
		auto pos = pNode->GetPosition();
		Elite::Vector2 verts[4]
		{
			Elite::Vector2(pos.x - m_SizeCell / 2.0f, pos.y - m_SizeCell / 2.0f),
			Elite::Vector2(pos.x - m_SizeCell / 2.0f, pos.y + m_SizeCell / 2.0f),
			Elite::Vector2(pos.x + m_SizeCell / 2.0f, pos.y + m_SizeCell / 2.0f),
			Elite::Vector2(pos.x + m_SizeCell / 2.0f, pos.y - m_SizeCell / 2.0f)
		};
		if (!pNode->IsWalkable())
		{
			DEBUGRENDERER2D->DrawSolidPolygon(&verts[0], 4, Color(1.f, 0.f, 0.f, 1.0f), 0.8f);
			DEBUGRENDERER2D->DrawSolidCircle(pos, 0.1f, Elite::ZeroVector2, Color(1.f, 0.f, 0.f, 1.0f), 0.8f);
		}
	}

	//Render the grid
	if (m_bDrawGrid)
	{
		for (auto pNode : nodes)
		{
			auto pos = pNode->GetPosition();
			Elite::Vector2 verts[4]
			{
				Elite::Vector2(pos.x - m_SizeCell / 2.0f, pos.y - m_SizeCell / 2.0f),
				Elite::Vector2(pos.x - m_SizeCell / 2.0f, pos.y + m_SizeCell / 2.0f),
				Elite::Vector2(pos.x + m_SizeCell / 2.0f, pos.y + m_SizeCell / 2.0f),
				Elite::Vector2(pos.x + m_SizeCell / 2.0f, pos.y - m_SizeCell / 2.0f)
			};
			if (pNode->IsWalkable())
			{
				DEBUGRENDERER2D->DrawSolidPolygon(&verts[0], 4, Color(0.8f, 0.8f, 0.8f, 1.0f), 0.9f);
				DEBUGRENDERER2D->DrawSolidCircle(pos, 0.1f, Elite::ZeroVector2, Color(0.8f, 0.8f, 0.8f, 1.0f), 0.9f);
			}
		}
	}

	//Render Connections
	if (m_bDrawConnections)
	{
		for (auto pNode : nodes)
		{
			auto connections = pNode->GetConnections();
			for (auto pConnection : connections)
			{
				DEBUGRENDERER2D->DrawSegment(
					pConnection->GetStartNode()->GetPosition(),
					pConnection->GetEndNode()->GetPosition(),
					Color(0.f, 0.f, 1.f));
			}
		}
	}

	//Render the path
	for(auto p : m_vPath)
	{
		Elite::Vector2 verts[4]
		{ 
			Elite::Vector2(p.x - m_SizeCell / 2.0f, p.y - m_SizeCell / 2.0f),
			Elite::Vector2(p.x - m_SizeCell / 2.0f, p.y + m_SizeCell / 2.0f),
			Elite::Vector2(p.x + m_SizeCell / 2.0f, p.y + m_SizeCell / 2.0f),
			Elite::Vector2(p.x + m_SizeCell / 2.0f, p.y - m_SizeCell / 2.0f) };
		DEBUGRENDERER2D->DrawSolidPolygon(&verts[0], 4, Color(0.8f, 0.45f, 0.f, 1.0f), 0.8f);
	}
}

void App_PathfindingAStar::MakeGridGraph()
{
	//Create Graph with Grid Layout
	m_pGridGraph = new Graph<Node>(450);
	for (auto r = 0; r < ROWS; ++r)
	{
		for (auto c = 0; c < COLUMNS; ++c)
		{
			m_pGridGraph->AppendNode(Node(Elite::Vector2(c * m_SizeCell + m_StartGrid.x, r * m_SizeCell + m_StartGrid.y)));
		}
	}
}

void App_PathfindingAStar::EvaluateGridGraph()
{
	//Reset
	auto nodes = m_pGridGraph->GetNodes();
	if (nodes.empty())
		return;
	for (auto pNode : nodes)
	{
		pNode->SetWalkable(true);
	}

	//Determine if nodes are walkable, based on LevelObstacles
	//(Row * amountOfNodesPerRow) + Column
	//--- Obstacle 1 ---
	nodes[(3 * 30) + 5]->SetWalkable(false);
	nodes[(3 * 30) + 6]->SetWalkable(false);
	nodes[(3 * 30) + 7]->SetWalkable(false);
	nodes[(3 * 30) + 8]->SetWalkable(false);
	nodes[(3 * 30) + 9]->SetWalkable(false);
	nodes[(3 * 30) + 10]->SetWalkable(false);

	//--- Obstacle 2 ---
	nodes[(7 * 30) + 7]->SetWalkable(false);
	nodes[(8 * 30) + 7]->SetWalkable(false);
	nodes[(9 * 30) + 7]->SetWalkable(false);
	nodes[(10 * 30) + 7]->SetWalkable(false);
	nodes[(11 * 30) + 7]->SetWalkable(false);
	nodes[(12 * 30) + 7]->SetWalkable(false);

	//--- Obstacle 3 ---
	nodes[(2 * 30) + 15]->SetWalkable(false);
	nodes[(3 * 30) + 15]->SetWalkable(false);
	nodes[(4 * 30) + 15]->SetWalkable(false);
	nodes[(5 * 30) + 15]->SetWalkable(false);
	nodes[(6 * 30) + 15]->SetWalkable(false);
	nodes[(7 * 30) + 15]->SetWalkable(false);
	nodes[(8 * 30) + 15]->SetWalkable(false);
	nodes[(9 * 30) + 15]->SetWalkable(false);
	nodes[(10 * 30) + 15]->SetWalkable(false);
	nodes[(11 * 30) + 15]->SetWalkable(false);
	nodes[(12 * 30) + 15]->SetWalkable(false);

	//--- Obstacle 4 ---
	nodes[(8 * 30) + 20]->SetWalkable(false);
	nodes[(8 * 30) + 21]->SetWalkable(false);
	nodes[(8 * 30) + 22]->SetWalkable(false);
	nodes[(8 * 30) + 23]->SetWalkable(false);
	nodes[(8 * 30) + 24]->SetWalkable(false);
	nodes[(8 * 30) + 25]->SetWalkable(false);

	//Create Connections
	for (auto r = 0; r < ROWS; ++r)
	{
		for (auto c = 0; c < COLUMNS; ++c)
		{
			//Index = (Row * COLUMNS) + Column
			//Reset connections
			auto pCurrentNode = nodes[(r * COLUMNS) + c];
			pCurrentNode->RemoveConnections();

			if (!pCurrentNode->IsWalkable())
				continue;

			// Up
			if (r > 0 && nodes[((r-1) * COLUMNS) + c]->IsWalkable())
				pCurrentNode->AddConnection(nodes[((r - 1) * COLUMNS + c)]);
			// Right
			if (c < COLUMNS - 1 && nodes[(r * COLUMNS) + (c + 1)]->IsWalkable())
				pCurrentNode->AddConnection(nodes[(r * COLUMNS) + (c + 1)]);
			// Down
			if (r < ROWS - 1 && nodes[((r + 1) * COLUMNS) + c]->IsWalkable())
				pCurrentNode->AddConnection(nodes[((r + 1) * COLUMNS + c)]);
			// Left
			if (c > 0 && nodes[(r * COLUMNS) + (c - 1)]->IsWalkable())
				pCurrentNode->AddConnection(nodes[(r * COLUMNS) + (c - 1)]);

			if (ALLOW_DIAGONAL_MOVEMENT)
			{
				// Bottom Left
				if (c > 0 && r > 0 && nodes[((r - 1) * COLUMNS) + (c - 1)]->IsWalkable())
				{
					//And nothing is blocking diagonal path
					if (nodes[(r * COLUMNS) + (c - 1)]->IsWalkable() && nodes[((r - 1) * COLUMNS) + c]->IsWalkable())
						pCurrentNode->AddConnection(nodes[((r - 1) * COLUMNS) + (c - 1)]);
				}
				// Bottom Right
				if (r > 0 && c < COLUMNS - 1 && nodes[((r - 1) * COLUMNS) + (c + 1)]->IsWalkable())
				{
					//And nothing is blocking diagonal path
					if (nodes[(r * COLUMNS) + (c + 1)]->IsWalkable() && nodes[((r - 1) * COLUMNS) + c]->IsWalkable())
						pCurrentNode->AddConnection(nodes[((r - 1) * COLUMNS) + (c + 1)]);
				}
				// Top Left
				if (c > 0 && r < ROWS - 1 && nodes[((r + 1) * COLUMNS) + (c - 1)]->IsWalkable())
				{
					//And nothing is blocking diagonal path
					if (nodes[(r * COLUMNS) + (c - 1)]->IsWalkable() && nodes[((r + 1) * COLUMNS) + c]->IsWalkable())
						pCurrentNode->AddConnection(nodes[((r + 1) * COLUMNS) + (c - 1)]);
				}
				// Top Right
				if (r < ROWS - 1 && c < COLUMNS - 1 && nodes[((r + 1) * COLUMNS) + (c + 1)]->IsWalkable())
				{
					//And nothing is blocking diagonal path
					if (nodes[(r * COLUMNS) + (c + 1)]->IsWalkable() && nodes[((r + 1) * COLUMNS) + c]->IsWalkable())
						pCurrentNode->AddConnection(nodes[((r + 1) * COLUMNS) + (c + 1)]);
				}
			}
		}
	}
}