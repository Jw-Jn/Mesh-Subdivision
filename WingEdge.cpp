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

	normalizeEdge(vertices, normals, faces);

	// convert the mesh to winged-edge form
	convertOBJToWingedEdgeMesh(vertices, normals, faces);

	return true;
}

void WingEdge::normalizeEdge(std::vector<vec3> &vertices, std::vector<vec3> &normals, std::vector<std::vector<int>> &faces)
{
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
}

void WingEdge::convertOBJToWingedEdgeMesh(std::vector<vec3> &vertices, std::vector<vec3> &normals, std::vector<std::vector<int>> &faces)
{
	// create wvertex and save to vertexlist
	vertexList.clear();
	for (int i = 0; i < vertices.size(); i++)
		vertexList.push_back(new Wvertex(vertices[i], normals[i], i));

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
			Wvertex *v_end = vertexList[v0];

			int v1 = faces[i][(v+1)%vnum];
			Wvertex *v_start = vertexList[v1];

			Wedge *edge = edgeList[vnum*i + v];
			edge->v_start = v_start;
			edge->v_end = v_end;

			Wedge *edge_nex = edgeList[vnum*i + (v - 1 + vnum) % vnum];
			edge->e_right_nex = edge_nex;

			Wedge *edge_pre = edgeList[vnum*i + (v + 1) % vnum];
			edge->e_right_pre = edge_pre;

			edgeTable[v1][v0] = vnum * i + v;
			edge->f_right = f;

			f->oneEdge = edge;
			v_start->oneEdge = edge;
		   // v_end->oneEdge = edge; 
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
				int flipIdx = edgeTable[j][i];
				Wedge *edgeFlip = edgeList[flipIdx];

				edge->e_left_pre = edgeFlip->e_right_pre;
				edge->e_left_nex = edgeFlip->e_right_nex; 

				edge->f_left = edgeFlip->f_right;
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

	int idx;

	// export faces
	for (int i = 0; i < faces.size(); i++) 
	{
		ofstr << "f";

		for (int j = 0; j < faces[i].size(); j++)
		{
			idx = faces[i][j]->idx;
			ofstr << " " << idx + 1;
		}
		ofstr << std::endl;
	}

	return true;
}

// for map
bool operator<(const int2 a, const int2 b) {
	if (a.x < b.x) return true;
	if (a.x > b.x) return false;
	if (a.y < b.y) return true;
	return false;
}
bool operator==(const int2 a, const int2 b) {
	return (a.x == b.x) && (a.y == b.y);
}

bool WingEdge::subdivision(std::string method, int level)
{
	// new face list
	std::vector<std::vector<int>> newFaces;

	std::cout << method << std::endl;
	if (method.compare("Butterfly")==0)
	{
		while (level--)
		{
			newFaces.clear();
			int faceNum = faceList.size();
			std::map<int2, int> cutMap;

			std::vector<int> evenVertices;
			std::vector<int> oddVertices;

			// for all faces
			for (int i = 0; i < faceNum; i++)
			{
				evenVertices.clear();
				oddVertices.clear();

				Wedge *e0 = faceList[i]->oneEdge;
				Wedge *edge = e0;

				// while !=e0
				do
				{
					// edge->end()->idx, save to even
					int startIdx = edge->v_start->idx;
					int endIdx = edge->v_end->idx;

					evenVertices.push_back(endIdx);

					// check edge devided?
					int2 edgeIdx = int2(startIdx, endIdx);
					if (startIdx > endIdx)
						edgeIdx = int2(endIdx, startIdx);

					int vertexIdx;

					std::pair<std::map<int2, int >::iterator, bool> res = cutMap.insert(std::pair<int2, int>(edgeIdx, -1));
					// not devided
					if (res.second) {
						// compute vertex
						Wvertex *v = new Wvertex();
						v->position = computeButterfly(edge);

						// insert to vertexlist
						vertexList.push_back(v);

						// save idx to map
						cutMap[edgeIdx] = vertexList.size() - 1;

						vertexIdx = vertexList.size() - 1;

					}
					// devided
					else {
						// get idx
						vertexIdx = res.first->second;
					}

					//save to odd
					oddVertices.push_back(vertexIdx);

					edge = edge->e_right_nex;

				} while (edge != e0);

				// odd[0,1,2]; odd[0,1]even[0]; odd[1,2]even[1]; odd[2,0]even[2];
				// face list insert vertexidx -> face
				int num = oddVertices.size();
				std::vector<int> face;

				for (int j = 0; j < num; j++)
				{
					face.clear();
					face.push_back(oddVertices[j]);
					face.push_back(oddVertices[(j + 1) % num]);
					face.push_back(evenVertices[j]);
					newFaces.push_back(face);
				}

				face.clear();
				for (int j = 0; j < num; j++)
					face.push_back(oddVertices[num - 1 - j]);
				newFaces.push_back(face);

			}

			bool result = reconstructMesh(newFaces);
			return result;
		}
	}

	else
	{
		std::cout << "loop" << std::endl;

		while (level--)
		{
			std::vector<Wvertex *> vertexList_newPosition;

			for (int i = 0; i < vertexList.size(); i++)
			{
				Wvertex *v = new Wvertex;
				vertexList_newPosition.push_back(v);
			}

			newFaces.clear();
			int faceNum = faceList.size();
			std::map<int2, int> cutMap;

			std::vector<int> evenVertices;
			std::vector<int> oddVertices;

			// for all faces
			for (int i = 0; i < faceNum; i++)
			{
				evenVertices.clear();
				oddVertices.clear();

				Wedge *e0 = faceList[i]->oneEdge;
				Wedge *edge = e0;

				// while !=e0
				do
				{
					// edge->end()->idx, save to even
					int startIdx = edge->v_start->idx;
					int endIdx = edge->v_end->idx;

					evenVertices.push_back(endIdx);
					vec3 newPosition = computeLoopVectex(edge);
					// compute again
					vertexList_newPosition[endIdx]->position = newPosition;

					// check edge devided?
					int2 edgeIdx = int2(startIdx, endIdx);
					if (startIdx > endIdx)
						edgeIdx = int2(endIdx, startIdx);

					int vertexIdx;

					std::pair<std::map<int2, int >::iterator, bool> res = cutMap.insert(std::pair<int2, int>(edgeIdx, -1));
					// not devided
					if (res.second) {
						// compute vertex
						Wvertex *v = new Wvertex();
						v->position = computeLoopEdge(edge);

						// insert to vertexlist
						vertexList.push_back(v);

						// save idx to map
						cutMap[edgeIdx] = vertexList.size() - 1;

						vertexIdx = vertexList.size() - 1;

					}
					// devided
					else {
						// get idx
						vertexIdx = res.first->second;
					}

					//save to odd
					oddVertices.push_back(vertexIdx);

					edge = edge->e_right_nex;

				} while (edge != e0);

				// odd[2,1,0]; odd[0,1]even[0]; odd[1,2]even[1]; odd[2,0]even[2];
				// face list insert vertexidx -> face
				int num = oddVertices.size();
				std::vector<int> face;

				for (int j = 0; j < num; j++)
				{
					face.clear();
					face.push_back(oddVertices[j]);
					face.push_back(oddVertices[(j + 1) % num]);
					face.push_back(evenVertices[j]);
					newFaces.push_back(face);
				}

				face.clear();
				for (int j = 0; j < num; j++)
					face.push_back(oddVertices[num - 1 - j]);
				newFaces.push_back(face);
			}

			for (int i = 0; i < vertexList_newPosition.size(); i++)
				vertexList[i]->position = vertexList_newPosition[i]->position;

			bool result = reconstructMesh(newFaces);
			return result;
		}
	}
	return false;
}

vec3 WingEdge::computeLoopEdge(Wedge *e)
{
	vec3 position(0, 0, 0);
	position = position + e->v_end->position*(3.0 / 8.0);
	position = position + e->v_start->position*(3.0 / 8.0);
	position = position + e->e_right_nex->v_end->position*(1.0 / 8.0);
	position = position + e->e_left_nex->v_end->position*(1.0 / 8.0);

	return position;
}

vec3 WingEdge::computeLoopVectex(Wedge *e)
{
	int k = 0;
	std::vector<vec3> vertices;
	vec3 position = e->v_end->position;
	Wedge *e0 = e->e_left_pre->e_right_nex;
	Wedge *edge = e0;
	vertices.clear();
	do
	{
		vertices.push_back(edge->v_end->position);
		edge = edge->e_left_nex;
		k += 1;
	} while (edge != e0);

	float beta = (1.0 / k)*(5.0 / 8.0 - (3.0 / 8.0 + 0.25*cos(6.28 / k))*(3.0 / 8.0 + 0.25*cos(6.28 / k)));

	position = position*(1.0 - k * beta);

	for (int i = 0; i < vertices.size(); i++)
		position = position + vertices[i] * beta;

	return position;
}

bool WingEdge::reconstructMesh(std::vector<std::vector<int>> newFaces)
{
	std::vector<vec3> vertices;
	std::vector<vec3> normals;

	vertices.clear();
	normals.clear();

	for (int i=0; i<vertexList.size(); i++)
	{
		// load a vertex
		vec3 v = vertexList[i]->position;
		vertices.push_back(v);
	}

	// Estimate normals of vertices with the weights defined by angles around them
	normals = std::vector<vec3>(vertices.size(), vec3(0, 0, 0));
	for (int i = 0; i < newFaces.size(); i++)
	{
		int vnum = newFaces[i].size();
		for (int j = 0; j < vnum; j++) {
			vec3 v = vertices[newFaces[i][j]];
			vec3 v_pre = vertices[newFaces[i][(j - 1 + vnum) % vnum]];
			vec3 v_nxt = vertices[newFaces[i][(j + 1) % vnum]];
			double weight = (v_pre - v).vectorDegreeBetween(v_nxt - v);
			normals[newFaces[i][j]] = normals[newFaces[i][j]] + ((v - v_pre) ^ (v_nxt - v)).normalize() * weight;
		}
	}
	for (int i = 0; i < normals.size(); ++i)
		normals[i].normalize();

	normalizeEdge(vertices, normals, newFaces);

	// convert the mesh to winged-edge form
	convertOBJToWingedEdgeMesh(vertices, normals, newFaces);

	return true;
}

vec3 WingEdge::computeButterfly(Wedge *e0)
{
	int leftD = countFace(e0->e_left_pre->e_right_nex);
	int rightD = countFace(e0);

	vec3 position;
	std::vector<vec3> pts;

	if (leftD == rightD && rightD == 6)
	{
		pts.clear();
		pts.push_back(vec3(e0->v_end->position));
		pts.push_back(vec3(e0->v_start->position));

		Wedge *edge = e0->e_right_nex;
		pts.push_back(vec3(edge->v_end->position));
		pts.push_back(vec3(edge->e_left_pre->v_start->position));

		edge = edge->e_right_nex;
		pts.push_back(vec3(edge->e_left_nex->v_end->position));

		edge = e0->e_left_nex;
		pts.push_back(vec3(edge->v_end->position));
		pts.push_back(vec3(edge->e_left_pre->v_start->position));

		edge = edge->e_right_nex;
		pts.push_back(vec3(edge->e_left_nex->v_end->position));

		position = pts[0] * 0.5 + pts[1] * 0.5
			+ pts[2] * 0.125 - pts[3] * 0.0625 - pts[4] * 0.0625
			+ pts[5] * 0.125 - pts[6] * 0.0625 - pts[7] * 0.0625;

		return position;
	}
	else
	{
		if (leftD == 6)
			position = computeButterflyPts(e0, rightD, false);
		else if (rightD == 6)
			position = computeButterflyPts(e0, leftD, true);
		else
		{
			vec3 p1 = computeButterflyPts(e0, leftD, true);
			vec3 p2 = computeButterflyPts(e0, rightD, false);
			position = (p1 + p2) / 2;
		}
		return position;
	}
}

vec3 WingEdge::computeButterflyPts(Wedge *e0, int k, bool left)
{
	Wedge *edge = e0;

	std::vector<vec3> s(k);
	vec3 position;

	if (left)
	{
		vec3 q = edge->v_end->position;
		if (k == 3)
		{
			s[0] = edge->v_start->position;
			edge = edge->e_right_nex;
			s[1] = edge->v_end->position;
			edge = edge->e_left_pre;
			s[2] = edge->v_start->position;
			position = s[2] * (-1.0 / 12.0) + (s[0] + s[1])*(5.0 / 12.0) + q * (1 - (-1.0 / 12) - (5.0 / 12.0) * 2);
		}
		else if (k == 4)
		{
			s[0] = edge->v_start->position;
			edge = edge->e_right_nex->e_left_pre;
			s[1] = edge->v_start->position;
			position = s[1] * (-1.0 / 8.0) + s[0] * (3.0 / 8.0) + q * (1 - (-1.0 / 8.0) - (3.0 / 8.0));
		}
		else
		{
			int i = 0;
			e0 = e0->e_left_pre->e_right_nex; // flip
			edge = e0;
			do
			{
				s[i] = edge->v_end->position;
				edge = edge->e_left_nex;
				i++;
			} while (edge != e0);

			float sum = 0.0;
			float sj = 0.0;

			for (i = 0; i < k; i++)
			{
				sj = (1.0 / k)*(1.0 / 4.0 + cos(2.0 * i*3.14 / k) + (1.0 / 2.0)*cos(4.0 * i*3.14 / k));
				position = position + s[i] * sj;
				sum += sj;
			}
			position = position + q * (1.0 - sum);
		}
	}
	else
	{
		vec3 q = edge->v_start->position;

		if (k == 3)
		{
			s[0] = edge->v_end->position;
			edge = edge->e_right_pre;
			s[1] = edge->v_start->position;
			edge = edge->e_left_nex;
			s[2] = edge->v_end->position;
			position = s[2] * (-1.0 / 12.0) + (s[0] + s[1])*(5.0 / 12.0) + q * (1 - (-1.0 / 12) - (5.0 / 12.0) * 2);
		}
		else if (k == 4)
		{
			s[0] = edge->v_end->position;
			edge = edge->e_right_pre->e_left_nex;
			s[1] = edge->v_end->position;
			position = s[1] * (-1.0 / 8.0) + s[0] * (3.0 / 8.0) + q * (1 - (-1.0 / 8.0) - (3.0 / 8.0));
		}
		else
		{
			int i = 0;
			edge = e0;
			do
			{
				s[i] = edge->v_end->position;
				edge = edge->e_left_nex;
				i++;
			} while (edge != e0);

			float sum = 0.0;
			float sj = 0.0;

			for (i = 0; i < k; i++)
			{
				sj = (1.0 / k)*(1.0 / 4.0 + cos(2.0 * i*3.14 / k) + (1.0 / 2.0)*cos(4.0 * i*3.14 / k));
				position = position + s[i] * sj;
				sum += sj;
			}
			position = position + q * (1.0 - sum);
		}
	}
	
	
	return position;
}

int WingEdge::countFace(Wedge *e)
{
	int count = 0;
	Wedge *edge = e;
	do
	{
		edge = edge->e_left_nex;
		count+=1;
	} while (edge != e);

	return count;
}