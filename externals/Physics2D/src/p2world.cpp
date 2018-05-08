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
#include <p2world.h>
#include <iostream>


p2World::p2World(p2Vec2 gravity) : m_Gravity(gravity)
{
}

void p2World::Step(float dt)
{
	DeltaTime += dt;

	for (auto p2Body_it = m_p2Bodys.begin(); p2Body_it != m_p2Bodys.end(); p2Body_it++)
	{
		if (p2Body_it->GetType() == p2BodyType::DYNAMIC)
		{
			p2Body_it->SetPosition(p2Vec2(p2Body_it->GetLinearVelocity().x * DeltaTime, (m_Gravity.y * (0.5) * DeltaTime * DeltaTime)* p2Body_it->GetGarvityScale() + p2Body_it->GetLinearVelocity().y * DeltaTime));
		}

		for (auto p2Body_it2 = m_p2Bodys.begin(); p2Body_it2 != m_p2Bodys.end(); p2Body_it2++)
		{
			if (p2Body_it != p2Body_it2)
			{
				if (Test_planets)
				{
					auto p2Body_Sun_it = m_p2Bodys.begin();
						SetMass(100000000.0f, 10000.0f);
						if(p2Body_Sun_it != p2Body_it)
						UniversGravity(&(*p2Body_it), &(*p2Body_Sun_it), dt);
				}

				if (p2Body_it->CheckContactAABB(&(*p2Body_it2)))
				{
					std::cout << "can touch\n";
				}
				else
				{
					std::cout << "can not touch\n";
				}
			}
		}
	}
}

p2Body * p2World::CreateBody(p2BodyDef* bodyDef)
{
	p2Body body(*bodyDef);
	m_p2Bodys.push_front(body);
	std::list<p2Body>::iterator it = m_p2Bodys.begin();
	return &(*it);
}

void p2World::SetContactListener(p2ContactListener * contactListener)
{
}

void p2World::UniversGravity(p2Body * a, p2Body * Sun, float dt)
{
	p2Vec2 VecDirGravity = Sun->GetPosition() - a->GetPosition();
	VecDirGravity.Normalized();
	p2Vec2 VecPerpenicular = p2Vec2(VecDirGravity.y, -(VecDirGravity.x));
	VecDirGravity = VecDirGravity * (6.67 * pow(10, -11) * a->GetMass() * Sun->GetMass() / pow((Sun->GetPosition() - a->GetPosition()).GetMagnitude(), 2));
	VecPerpenicular.Normalized();
	VecPerpenicular = VecPerpenicular + a->GetAcceleration() * dt;
	std::cout << a->GetAcceleration().x << "\n";
	a->SetPosition(VecDirGravity + VecPerpenicular / 50);
}

void p2World::SetMass(float Msun, float Mplanets)
{
	for (auto p2Body_it = m_p2Bodys.begin(); p2Body_it != m_p2Bodys.end(); p2Body_it++)
	{
		p2Body_it->SetMass(Mplanets);
		
		if (p2Body_it == m_p2Bodys.end())
		{
			p2Body_it->SetMass(Msun);
		}
	}
}
