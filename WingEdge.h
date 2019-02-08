#pragma once

#ifndef _WingEdge_h
#define _WingEdge_h

#include <vector>
#include <string>
#include "vec3.h"

class Wvertex;
class Wface;

class Wedge
{
public:
	Wedge() 
	{ 
		v_start = v_end = NULL; 
		e_left_pre = e_left_nex = e_right_pre = e_right_nex = NULL;
		f_left = f_right = NULL;
	}

	Wedge(Wvertex *vs, Wvertex *ve)
	{
		v_start = vs;
		v_end = ve;
		e_left_pre = e_left_nex = e_right_pre = e_right_nex = NULL;
		f_left = f_right = NULL;
	}

	Wvertex *v_start, *v_end;
	Wedge *e_left_pre, *e_left_nex;
	Wedge *e_right_pre, *e_right_nex;
	Wface *f_left, *f_right;
};

class Wvertex
{
public:
	Wvertex() 
	{
		oneEdge = NULL;
	}
	Wvertex(vec3 position, vec3 norm)
	{
		this->position = position;
		this->norm = norm;
	}
	vec3 position;
	vec3 norm;
	Wedge *oneEdge;
};

class Wface 
{
public:
	Wface() { oneEdge = NULL; }
	Wedge *oneEdge;
};

class WingEdge
{
public:
	bool loadOBJfile(std::string filename);
	bool saveToOBJfile(std::string fname);
	vec3 getVertex(Wvertex * v) { return v->position; };
	vec3 getNorm(Wvertex * v) { return v->norm; };
	std::vector<std::vector<Wvertex *>> extractVerticesOfFaces();

	float maxdimlength;
	vec3 center;

private:
	void convertOBJToWingedEdgeMesh(std::vector<vec3> &vertices, std::vector<vec3> &normals, std::vector<std::vector<int>> &faces);
	std::vector<Wface *> faceList;
	std::vector<Wvertex *> vertexList;
	std::vector<Wedge *> edgeList;
};

#endif