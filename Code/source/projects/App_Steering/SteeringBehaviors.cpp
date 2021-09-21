//Precompiled Header [ALWAYS ON TOP IN CPP]
#include "stdafx.h"

//Includes
#include "SteeringBehaviors.h"
#include "SteeringAgent.h"

//SEEK
//****
SteeringOutput Seek::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};
	steering.LinearVelocity = m_Target.Position - pAgent->GetPosition();
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	//Debug rendering
	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, steering.LinearVelocity.Magnitude(), { 0.1f, 1.f, 0.f, 0.5f }, 0.4f);
	}

	return steering;
}

//WANDER (base > SEEK)
//******
SteeringOutput Wander::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	const float halfJitter = m_MaxJitterOffset / 2;
	const Elite::Vector2 randomOffset = Elite::Vector2{ randomFloat(-halfJitter, halfJitter), randomFloat(-halfJitter, halfJitter) };
	m_WanderingTarget += randomOffset;
	m_WanderingTarget.Normalize();
	m_WanderingTarget *= m_Radius;

	//Add offset
	Elite::Vector2 offset{ pAgent->GetLinearVelocity() };
	offset.Normalize();
	offset *= m_Offset;

	m_Target = TargetData{ pAgent->GetPosition() + offset + m_WanderingTarget };

	if (pAgent->CanRenderBehavior())
	{
		Elite::Vector2 pos{ pAgent->GetPosition() };
		DEBUGRENDERER2D->DrawSegment(pos, pos + offset, { 0.f, 0.f, 1.f, 0.5f }, 0.4f);
		DEBUGRENDERER2D->DrawCircle(pos + offset, m_Radius, { 0.f, 0.f, 1.f, 0.5f }, { 0.4f });
		DEBUGRENDERER2D->DrawSolidCircle(pos + offset + m_WanderingTarget, 0.5f, { 0,0 }, { 0.f, 1.f, 0.f, 0.5f }, 0.3f);

	};

	return Seek::CalculateSteering(deltaT, pAgent);
}

//Flee 
//******
Flee::Flee(float evasionRadius /*15.f*/)
	:m_EvasionRadius(evasionRadius)
{

}

SteeringOutput Flee::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	auto distanceToTarget = Elite::Distance(pAgent->GetPosition(), m_Target.Position);
	SteeringOutput steering{};

	if (distanceToTarget < m_EvasionRadius)
	{
		steering.LinearVelocity = pAgent->GetPosition() - m_Target.Position;
		steering.LinearVelocity.Normalize();
		steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();
	}
	else
	{
		steering.IsValid = false;
	}

	//Debug rendering
	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, steering.LinearVelocity.Magnitude(), { 0.1f, 1.f, 0.f, 0.5f }, 0.4f);
	}

	return steering;
}

//Arrive
//******
SteeringOutput Arrive::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	const float maxSpeed = 10.f;
	const float arrivalRadius = 1.f;
	const float slowRadius = 15.f;

	SteeringOutput steering{ Seek::CalculateSteering(deltaT, pAgent) };

	Elite::Vector2 toTarget = m_Target.Position - pAgent->GetPosition();
	const float distance = toTarget.Magnitude();

	//Stop the actor if it is within acceptable range of the target
	if (distance < arrivalRadius)
	{
		pAgent->SetLinearVelocity(Elite::ZeroVector2);
		//return;
	}

	toTarget.Normalize();

	if (distance < slowRadius)
	{
		toTarget *= maxSpeed * (distance / slowRadius);
	}
	else   
	{
		toTarget *= maxSpeed;
	}

	//pAgent->SetLinearVelocity(toTarget);
	steering.LinearVelocity = toTarget;
	//steering.LinearVelocity
	return steering; 
}

//Face 
//******
SteeringOutput Face::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};
	steering.LinearVelocity = m_Target.Position - pAgent->GetPosition();
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= 0.000000001f;

	//Debug rendering
	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, steering.LinearVelocity.Magnitude(), { 0.1f, 1.f, 0.f, 0.5f }, 0.4f);
	}

	return steering;
}

//Evade 
//******
SteeringOutput Evade::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};
	steering.LinearVelocity = m_Target.Position - pAgent->GetPosition();
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= 0.000000001f;

	//Debug rendering
	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, steering.LinearVelocity.Magnitude(), { 0.1f, 1.f, 0.f, 0.5f }, 0.4f);
	}

	return steering;
}

//Pursuit 
//******
SteeringOutput Pursuit::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	//m_Target.LinearVelocity;

	SteeringOutput steering{};
	steering.LinearVelocity = (m_Target.Position + m_Target.LinearVelocity) - (pAgent->GetPosition() + pAgent->GetLinearVelocity());
	/*if (m_Target.LinearVelocity.x != 0.f && m_Target.LinearVelocity.y != 0.f)
	{
		steering.LinearVelocity + pAgent->GetLinearVelocity();
	}*/
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	//Debug rendering
	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, steering.LinearVelocity.Magnitude(), { 0.1f, 1.f, 0.f, 0.5f }, 0.4f);
	}

	return steering;
}

/*
If implementiing steering behaviour, you might want to false the autoRotation in app_SteeringBehaviours.cpp
*/