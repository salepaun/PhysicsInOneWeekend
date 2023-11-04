//
//	Body.h
//
#pragma once
#include "../Math/Vector.h"
#include "../Math/Quat.h"
#include "../Math/Matrix.h"
#include "../Math/Bounds.h"
#include "Shapes.h"

#include "../Renderer/model.h"
#include "../Renderer/shader.h"

/*
====================================================
Body
====================================================
*/
class Body {
public:
	Body();

	Vec3		m_position;
	Quat		m_orientation;
	Vec3		m_LinearVelocity;
	float		m_invMass;
	Shape *		m_shape;

	bool IsStatic() const { return m_invMass == 0.0f; }

	Vec3 GetCenterOfMassWorldSpace() const;
	Vec3 GetCenterOfMassModelSpace() const;

	Vec3 WorldSpaceToBodySpace(const Vec3& pt) const;
	Vec3 BodySpaceToWorldSpace(const Vec3& pt) const;

	void ApplyLinearImpulse(const Vec3& impulse);
};