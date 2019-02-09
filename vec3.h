#pragma once

#ifndef _vec3_h_
#define _vec3_h_

class vec3 
{
public:

	float x, y, z;

	vec3() { x = y = z = 0; }
	vec3(float x_, float y_, float z_) { x = x_; y = y_; z = z_; }
	vec3 operator+(vec3 t) { return vec3(x + t.x, y + t.y, z + t.z); }
	vec3 operator-(vec3 t) { return vec3(x - t.x, y - t.y, z - t.z); }
	vec3 operator^(vec3 t) { return vec3(y * t.z - z * t.y, z * t.x - x * t.z, x * t.y - y * t.x); }
	vec3 operator*(float t) { return vec3(x*t, y*t, z*t); }
	vec3 operator/(vec3 t) { return vec3(x / t.x, y / t.y, z / t.z); }
	vec3 operator/(float t) { return vec3(x / t, y / t, z / t); }

	float operator*(vec3 t) { return x * t.x + y * t.y + z * t.z; }

	float norm() { return sqrtf(x*x + y * y + z * z); }
	vec3 normalize() { float n = norm();  x /= n; y /= n; z /= n; return *this; }

	float vectorDegreeBetween(vec3 t) {
		float cosalpha = ((*this) * t) / (norm() * t.norm());
		if (cosalpha < -1) cosalpha = -1; if (cosalpha > 1) cosalpha = 1;
		return acos(cosalpha);
	}

};

class int2 {
public:
	int2() { x = y = 0; }
	int2(int x_, int y_) { x = x_; y = y_; }
	int x, y;
};

#endif