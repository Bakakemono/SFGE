#include "..\include\p2collider.h"


p2Collider::p2Collider(p2ColliderDef* colliderDef)
{
	userData = (*colliderDef).userData;
}


bool p2Collider::IsSensor()
{
	return false;
}

void * p2Collider::GetUserData()
{
	return userData;
}
