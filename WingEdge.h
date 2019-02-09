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
	Wvertex(vec3 position, vec3 norm, int idx)
	{
		this->position = position;
		this->norm = norm;
		this->idx = idx;
	}
	Wvertex(vec3 position)
	{
		this->position = position;
	}
	vec3 position;
	vec3 norm;
	Wedge *oneEdge;
	int idx;
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
	bool subdivision(std::string method, int level);

	float maxdimlength;
	vec3 center;

private:
	void convertOBJToWingedEdgeMesh(std::vector<vec3> &vertices, std::vector<vec3> &normals, std::vector<std::vector<int>> &faces);
	void normalizeEdge(std::vector<vec3> &vertices, std::vector<vec3> &normals, std::vector<std::vector<int>> &faces);
	vec3 computeButterfly(Wedge *edge);
	bool reconstructMesh(std::vector<std::vector<int>> newFaces);
	int countFace(Wedge *e);
	vec3 computeButterflyPts(Wedge *e0, int fnum, bool left);
	vec3 computeLoopEdge(Wedge *edge);
	vec3 computeLoopVectex(Wedge *edge);

	std::vector<Wface *> faceList;
	std::vector<Wvertex *> vertexList;
	std::vector<Wedge *> edgeList;
};

#endif