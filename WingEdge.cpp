#include <fstream>
#include <sstream>
#include <map>
#include <iostream>
#include <algorithm>

#include "WingEdge.h"

bool WingEdge::loadOBJfile(std::string filename)
{
	
	std::vector<vec3> vertices;
	std::vector<vec3> normals;
	std::vector<std::vector<int>> faces;
	
	std::ifstream ifstr(filename);
	if (!ifstr)
		return false;

	vertices.clear();
	faces.clear();
	normals.clear();

	// load vertices and faces
	std::string line;
	while (std::getline(ifstr, line)) 
	{
		if (line.length() < 1 || line[0] == '#')
			continue;

		else if (line[0] == 'v') 
		{

			// load a vertex
			line.erase(line.begin());
			std::istringstream iss(line);
			vec3 v;
			iss >> v.x >> v.y >> v.z;
			vertices.push_back(v);

		}
		else if (line[0] == 'f') 
		{

			// load a face
			line.erase(line.begin());
			faces.resize(faces.size() + 1);
			std::istringstream iss(line);
			int idx;
			while (iss >> idx)
				faces.back().push_back(idx - 1);
		}
	}

	// Estimate normals of vertices with the weights defined by angles around them
	normals = std::vector<vec3>(vertices.size(), vec3(0, 0, 0));
	for (int i = 0; i < faces.size(); i++) 
	{
		int vnum = faces[i].size();
		for (int j = 0; j < vnum; j++) {
			vec3 v = vertices[faces[i][j]];
			vec3 v_pre = vertices[faces[i][(j - 1 + vnum) % vnum]];
			vec3 v_nxt = vertices[faces[i][(j + 1) % vnum]];
			double weight = (v_pre - v).vectorDegreeBetween(v_nxt - v);
			normals[faces[i][j]] = normals[faces[i][j]] + ((v - v_pre) ^ (v_nxt - v)).normalize() * weight;
		}
	}
	for (int i = 0; i < normals.size(); ++i)
		normals[i].normalize();

	// find bbox of the mesh
	vec3 min(0, 0, 0);
	vec3 max(0, 0, 0);
	for (int i = 0; i < vertices.size(); i++)
	{
		vec3 v = vertices[i];
		if (v.x < min.x) min.x = v.x;
		if (v.y < min.y) min.y = v.y;
		if (v.z < min.z) min.z = v.z;

		if (v.x > max.x) max.x = v.x;
		if (v.y > max.y) max.y = v.y;
		if (v.z > max.z) max.z = v.z;
	}

	center = (max + min) / 2;
	vec3 dimlengths = max - min;

	maxdimlength = dimlengths.x;
	if (dimlengths.y > maxdimlength) maxdimlength = dimlengths.y;
	if (dimlengths.z > maxdimlength) maxdimlength = dimlengths.z;

	// normalize bounding box of the vertices
	for (int i = 0; i < vertices.size(); i++)
		vertices[i] = (vertices[i] - center) / maxdimlength;

	// convert the mesh to winged-edge form
	convertOBJToWingedEdgeMesh(vertices, normals, faces);

	return true;
}

void WingEdge::convertOBJToWingedEdgeMesh(std::vector<vec3> &vertices, std::vector<vec3> &normals, std::vector<std::vector<int>> &faces)
{
	// create wvertex and save to vertexlist
	vertexList.clear();
	for (int i = 0; i < vertices.size(); i++)
		vertexList.push_back(new Wvertex(vertices[i], normals[i]));

	// create wedge and save to edge
	edgeList.clear();
	for (int i = 0; i < faces.size()*3; i++)
		edgeList.push_back(new Wedge());

	// create wface and save to face
	faceList.clear();
	for (int i = 0; i < faces.size(); i++)
		faceList.push_back(new Wface());

	int vnum = faces[0].size();

	// use edgetable to record indices of start and end vertex, edge index
	int m = vertices.size();
	std::vector<std::vector<int>> edgeTable(m, std::vector<int>(m, -1));

	for (int i = 0; i < faces.size(); i++) 
	{
		Wface *f = faceList[i];
		
		for (int v = 0; v < vnum; v++) 
		{
			int v0 = faces[i][v];
			Wvertex *v_start = vertexList[v0];

			int v1 = faces[i][(v+1)%vnum];
			Wvertex *v_end = vertexList[v1];

			Wedge *edge = edgeList[vnum*i + v];
			edge->v_start = v_start;
			edge->v_end = v_end;

			Wedge *edge_pre = edgeList[vnum*i + (v - 1 + vnum) % vnum];
			edge->e_right_pre = edge_pre;

			Wedge *edge_nex = edgeList[vnum*i + (v + 1) % vnum];
			edge->e_right_nex = edge_nex;

			edgeTable[v0][v1] = vnum * i + v;
			edge->f_right = f;

			f->oneEdge = edge;
			v_start->oneEdge = edge;
			v_end->oneEdge = edge; 
		}
	}

	// save left according the table
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < m; j++)
		{
			int idx = edgeTable[i][j];
			if (idx != -1)
			{
				Wedge *edge = edgeList[idx];
				edge->e_left_pre = edgeList[(idx - 1 + vnum) % vnum];
				edge->e_left_nex = edgeList[(idx + 1) % vnum];
				edge->f_left = edge->e_left_pre->f_right;
			}
		}
	}
}

std::vector<std::vector<Wvertex *>> WingEdge::extractVerticesOfFaces()
{
	// the vertices index
	std::vector<std::vector<Wvertex *>> vertices(faceList.size());

	for (int i = 0; i < faceList.size(); i++)
	{
		Wface *f = faceList[i];
		Wedge *e0 = f->oneEdge;
		Wedge *edge = e0;

		// find all the edges related to the face
		do
		{
			if (edge->f_right == f)
				edge = edge->e_right_pre;
			vertices[i].push_back(edge->v_start);
		} 
		while (edge != e0);
	}

	return vertices;
}

bool WingEdge::saveToOBJfile(std::string fname)
{
	// check whether we could create the file
	std::ofstream ofstr(fname);
	if (!ofstr)
		return false;

	// extract the face list from winged-edge mesh
	std::vector<std::vector<Wvertex *>> faces = extractVerticesOfFaces();

	// export vertices
	for (int i = 0; i < vertexList.size(); i++) 
	{
		vec3 v = (vertexList[i]->position* maxdimlength) + center;
		ofstr << "v " << v.x
			<< " " << v.y
			<< " " << v.z << std::endl;
	}

	std::vector<Wvertex *>::iterator idx;

	// export faces
	for (int i = 0; i < faces.size(); i++) 
	{
		ofstr << "f";

		for (int j = 0; j < faces[i].size(); j++)
		{
			idx = std::find(vertexList.begin(), vertexList.end(), faces[i][j]);
			ofstr << " " << std::distance(vertexList.begin(), idx) + 1;
		}
		ofstr << std::endl;
	}

	return true;
}