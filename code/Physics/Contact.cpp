//
//  Contact.cpp
//
#include "Contact.h"

/*
====================================================
ResolveContact
====================================================
*/
void ResolveContact( contact_t & contact ) {
	Body* bodyA = contact.bodyA;
	Body* bodyB = contact.bodyB;

	const float invMassA = bodyA->m_invMass;
	const float invMassB = bodyB->m_invMass;
	const float invMassSum = invMassA + invMassB;

	const float elasticityA = bodyA->m_elasticity;
	const float elasticityB = bodyB->m_elasticity;
	const float elasticity = elasticityA * elasticityB;

	const Vec3 normal = contact.normal;
	const Vec3 vab = bodyA->m_LinearVelocity - bodyB->m_LinearVelocity;
	const float impulseJ = -(1.0f + elasticity) * vab.Dot(normal) / invMassSum;
	const Vec3 vectorImpulse = normal * impulseJ;

	bodyA->ApplyLinearImpulse(vectorImpulse);
	bodyB->ApplyLinearImpulse(vectorImpulse * -1.0);

	const float tA = invMassA / (invMassSum);
	const float tB = invMassB / (invMassSum);

	const Vec3 ds = contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace;
	bodyA->m_position += ds * tA;
	bodyB->m_position -= ds * tB;
}