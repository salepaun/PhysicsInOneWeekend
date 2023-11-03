//
//  Body.cpp
//
#include "Body.h"

/*
====================================================
Body::Body
====================================================
*/
Body::Body() :
	m_position(0.0f),
	m_orientation(0.0f, 0.0f, 0.0f, 1.0f),
	m_LinearVelocity(0.0f),
	m_invMass(0),
	m_shape(nullptr)
{
}

Vec3 Body::GetCenterOfMassWorldSpace() const
{
	const Vec3 centerOfMass = m_shape->GetCenterOfMass();
	const Vec3 pos = m_position + m_orientation.RotatePoint(centerOfMass);
	return pos;
}

Vec3 Body::GetCenterOfMassModelSpace() const
{
	const Vec3 centerOfMass = m_shape->GetCenterOfMass();
	return centerOfMass;
}

Vec3 Body::WorldSpaceToBodySpace(const Vec3& pt) const
{
	Vec3 centerOfMassToPointWorldSpace = pt - GetCenterOfMassWorldSpace();
	Quat inverseOrient = m_orientation.Inverse();
	Vec3 bodySpace = inverseOrient.RotatePoint(centerOfMassToPointWorldSpace);
	return bodySpace;
}

Vec3 Body::BodySpaceToWorldSpace(const Vec3& pt) const
{
	Vec3 worldSpace = GetCenterOfMassWorldSpace() + m_orientation.RotatePoint(pt);
	return worldSpace;
}

void Body::ApplyLinearImpulse(const Vec3& impulse)
{
	if (m_invMass == 0.0f) return;

	m_LinearVelocity += impulse * m_invMass;
}
