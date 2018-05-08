/*
MIT License

Copyright (c) 2017 SAE Institute Switzerland AG

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include <p2body.h>
#include <iostream>


p2Body::p2Body(p2BodyDef bodyDef)
{
	this->position = bodyDef.position;
	this->linearVelocity = bodyDef.linearVelocity;
	this->type = bodyDef.type;
	this->gravityScale = bodyDef.gravityScale;
}

p2Vec2 p2Body::GetLinearVelocity()
{
	return linearVelocity;
}

void p2Body::SetLinearVelocity(p2Vec2 velocity)
{
	linearVelocity += velocity;
}
float p2Body::GetAngularVelocity()
{
	return angularVelocity;
}

p2Collider* p2Body::GetCollider()
{
	return &(*collider);
}

p2Vec2 p2Body::GetPosition()
{
	return position;
}

p2AABB p2Body::GetAABB()
{
	return p2AABB(this->aabb);
}

void p2Body::SetPosition(p2Vec2 v)
{
	position = position + v;
	aabb.SetPosition(v);
}

float p2Body::GetGarvityScale()
{
	if (this->gravityScale == 0)
		return gravityScale;
	else
		return 1.0f;
}

void p2Body::SetMass(float Mass)
{
	mass = Mass;
}

float p2Body::GetMass()
{
	return mass;
}

void p2Body::SetAcceleration()
{
	acceleration = p2Vec2((rand() % 10) + 1, 0);
	
}

p2Vec2 p2Body::GetAcceleration()
{
	return acceleration;
}

p2BodyType p2Body::GetType()
{
	return this->type;
}

p2Collider * p2Body::CreateCollider(p2ColliderDef * colliderDef)
{
	if (p2CircleShape* d = dynamic_cast<p2CircleShape*>(colliderDef->shape))
	{
		this->aabb = p2AABB(GetPosition(), d->GetRadius());
	}

	else if (p2RectShape* d = dynamic_cast<p2RectShape*>(colliderDef->shape))
	{
		this->aabb = p2AABB(GetPosition(), d->GetSize());
	}

	this->collider = new p2Collider(colliderDef);
	this->SetAcceleration();
	return (this->collider);
}

bool p2Body::CheckContactAABB(p2Body* body)
{
	p2Vec2 body1 = this->aabb.GetCenter() - p2Vec2(this->aabb.GetExtends().x, this->aabb.GetExtends().y);
	p2Vec2 body2 = body->aabb.GetCenter() - p2Vec2(body->aabb.GetExtends().x, body->aabb.GetExtends().y);


	if ((body2.x >= body1.x + this->aabb.GetExtends().x*2)
		|| (body2.x + body->aabb.GetExtends().x * 2 <= body1.x)
		|| (body2.y >= body1.y + this->aabb.GetExtends().y * 2)
		|| (body2.y + body->aabb.GetExtends().y * 2 <= body1.y))
	{
		return false;
	}
	else
	{
		return true;
	}
}
