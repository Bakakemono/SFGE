#include "..\include\p2collider.h"


p2Collider::p2Collider(p2ColliderDef* colliderDef)
{
	userData = colliderDef->userData;
	isSensor = colliderDef->isSensor;
	shape = colliderDef->shape;
}


bool p2Collider::IsSensor()
{
	return this->isSensor;
}

void * p2Collider::GetUserData()
{
	return userData;
}

p2Collider p2Collider::GetCollider()
{
	return *this;
}

p2Shape p2Collider::GetShape()
{
	return *shape;
}
