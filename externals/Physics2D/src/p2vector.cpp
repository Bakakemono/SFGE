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

#include <p2vector.h>
#include <cmath>

p2Vec2::p2Vec2()
{
}

p2Vec2::p2Vec2(float x, float y)
{
	this->x = x;
	this->y = y;
}

p2Vec2 p2Vec2::operator+(p2Vec2 v)
{
	return p2Vec2(this->x + v.x, this->y + v.y);
}

p2Vec2 p2Vec2::operator+=(p2Vec2 v)
{
	return p2Vec2(*this + v);
}

p2Vec2 p2Vec2::operator-(p2Vec2 v)
{
	return p2Vec2(this->x - v.x, this->y - v.y);
}

p2Vec2 p2Vec2::operator-=(p2Vec2 v)
{
	return p2Vec2(*this - v);

}

p2Vec2 p2Vec2::operator/(float f)
{
	return p2Vec2(this->x / f, this->y / f);

}

p2Vec2 p2Vec2::operator*(float f)
{
	return p2Vec2(this->x * f, this->y * f);

}

float p2Vec2::Dot(p2Vec2 v1, p2Vec2 v2)
{
	return v1.x*v2.x + v1.y*v2.y;
}

p2Vec3 p2Vec2::Cross(p2Vec2 v1, p2Vec2 v2)
{
	return p2Vec3(v1.y*0.0f - v2.y * 0.0f, 0.0f*v2.x - 0.0f * v1.x, v1.x*v2.y - v2.x * v1.y);

}

float p2Vec2::GetMagnitude()
{
	return sqrt(pow(this->x, 2) + pow(this->y, 2));
}

p2Vec2 p2Vec2::Normalized()
{
	return p2Vec2(*this / this->GetMagnitude());
}

void p2Vec2::Normalize()
{
	*this = *this / this->GetMagnitude();
	x /= GetMagnitude();
	y /= GetMagnitude();
}

p2Vec3 p2Vec2::to3()
{
	return p2Vec3(this->x, this->y, 0.0f);
}

p2Vec3::p2Vec3()
{
}

p2Vec3::p2Vec3(float x, float y, float z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

p2Vec3 p2Vec3::operator+(p2Vec3 v)
{
	return p2Vec3(this->x + v.x, this->y + v.y, this->z + v.z);
}

p2Vec3 p2Vec3::operator+=(p2Vec3 v)
{
	return p2Vec3(*this + v);
}

p2Vec3 p2Vec3::operator-(p2Vec3 v)
{
	return p2Vec3(this->x - v.x, this->y - v.y, this->z - v.z);
}

p2Vec3 p2Vec3::operator-=(p2Vec3 v)
{
	return p2Vec3(*this - v);
}

p2Vec3 p2Vec3::operator/(float f)
{
	return p2Vec3(this->x / f, this->y / f, this->z / f);
}

p2Vec3 p2Vec3::operator*(float f)
{
	return p2Vec3(this->x * f, this->y * f, this->z * f);
}

float p2Vec3::Dot(p2Vec3 v1, p2Vec3 v2)
{
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

p2Vec3 p2Vec3::Cross(p2Vec3 v1, p2Vec3 v2)
{
	return p2Vec3(v1.y*v2.z - v2.y * v1.z, v1.z*v2.x - v2.z * v1.x, v1.x*v2.y - v2.x * v1.y);
}
