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
	m_linearVelocity(0.0f),
	m_angularVelocity(0.0f),
	m_invMass(0),
	m_elasticity(1),
	m_friction(0),
	m_shape(nullptr)
{
}

void Body::Update(const float dt_sec)
{
	m_position += m_linearVelocity * dt_sec;
	// okay, we have an angular velocity around the center of mass, this needs to be
// converted somehow to relative to model position.  This way we can properly update
// the orientation of the model.
	Vec3 positionCM = GetCenterOfMassWorldSpace();
	Vec3 cmToPos = m_position - positionCM;

	// Total Torque is equal to external applied torques + internal torque (precession)
	// T = T_external + omega x I * omega
	// T_external = 0 because it was applied in the collision response function
	// T = Ia = w x I * w
	// a = I^-1 ( w x I * w )
	Mat3 orientation = m_orientation.ToMat3();
	Mat3 inertiaTensor = orientation * m_shape->InertiaTensor() * orientation.Transpose();
	Vec3 alpha = inertiaTensor.Inverse() * (m_angularVelocity.Cross(inertiaTensor * m_angularVelocity));
	m_angularVelocity += alpha * dt_sec;

	// Update orientation
	Vec3 dAngle = m_angularVelocity * dt_sec;
	Quat dq = Quat(dAngle, dAngle.GetMagnitude());
	m_orientation = dq * m_orientation;
	m_orientation.Normalize();

	// Now get the new model position
	m_position = positionCM + dq.RotatePoint(cmToPos);
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

Mat3 Body::GetInverseInertiaTensorBodySpace() const
{
	Mat3 inertiaTensor = m_shape->InertiaTensor();
	Mat3 invInertiaTensor = inertiaTensor.Inverse() * m_invMass;
	return invInertiaTensor;
}

Mat3 Body::GetInverseInertiaTensorWorldSpace() const
{
	Mat3 inertiaTensor = m_shape->InertiaTensor();
	Mat3 invInertiaTensor = inertiaTensor.Inverse() * m_invMass;
	Mat3 orient = m_orientation.ToMat3();
	invInertiaTensor = orient * invInertiaTensor * orient.Transpose();
	return invInertiaTensor;
}

void Body::ApplyImpulse(const Vec3& impulsePoint, const Vec3& impulse)
{
	if (m_invMass == 0.0f) return;

	ApplyLinearImpulse(impulse);
	Vec3 position = GetCenterOfMassWorldSpace();
	Vec3 r = impulsePoint - position;
	Vec3 dl = r.Cross(impulse);
	ApplyAngularImpulse(dl);
}

void Body::ApplyLinearImpulse(const Vec3& impulse)
{
	if (m_invMass == 0.0f) return;

	m_linearVelocity += impulse * m_invMass;
}

void Body::ApplyAngularImpulse(const Vec3& impulse)
{
	if (m_invMass == 0.0f) return;

	m_angularVelocity += GetInverseInertiaTensorWorldSpace() * impulse;
	const float maxAngularSpeed = 30.0f;
	const float maxAngularSpeedSqr = maxAngularSpeed * maxAngularSpeed;
	if (m_angularVelocity.GetLengthSqr() > maxAngularSpeedSqr) {

		m_angularVelocity.Normalize();
		m_angularVelocity *= maxAngularSpeed;
	}
}
