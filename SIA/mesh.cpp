#include "mesh.h"
#ifdef WIN32
	#include <windows.h>
#endif
#include <GL/gl.h>

void Mesh::draw() 
{
	bool normalsOk = (_normals.size()==_points.size());
	bool colorsOk = (_colors.size()==_points.size());
	glEnable(GL_NORMALIZE);
	glColor4f(_color.x, _color.y, _color.z, _color.w);
	switch (_nbEdges) {
		case 3 :
			glBegin(GL_TRIANGLES);
			for (unsigned int i = 0 ; i < _triangles.size() ; i++) {
				if (normalsOk) glNormal3f(_normals[_triangles[i]].x,_normals[_triangles[i]].y,_normals[_triangles[i]].z);
				if (colorsOk) glColor3f(_colors[_triangles[i]].x,_colors[_triangles[i]].y,_colors[_triangles[i]].z);
				glVertex3f(_points[_triangles[i]].x,_points[_triangles[i]].y,_points[_triangles[i]].z);
			}
			glEnd();
			break;
		case 4 :
			glBegin(GL_QUADS);
			for (unsigned int i = 0 ; i < _triangles.size() ; i++) {
				if (normalsOk) glNormal3f(_normals[_triangles[i]].x,_normals[_triangles[i]].y,_normals[_triangles[i]].z);
				glVertex3f(_points[_triangles[i]].x,_points[_triangles[i]].y,_points[_triangles[i]].z);
			}
			glEnd();
			break;
	}

}

void Mesh::load(const char* fileName) {
	_points.clear();
	_normals.clear();
	_faces.clear();
	_triangles.clear();
	_nbEdges = 0;

	FILE *fdat; fopen_s(&fdat, fileName, "r");
	
	char line[300];
	char *next_token;
	while(fgets(line, 300, fdat)) {
		if (line[0]=='#')
			continue;
		if (strstr(line, "g ")) {
			continue;
		} else {
			char* key = strtok_s(line, " \t\n\r", &next_token);
			if (!key)
				continue;
			if (key[0]=='v' && key[1]=='n') {
				glm::vec3 vi;
				char* valx = strtok_s(NULL, " \t\n\r", &next_token);
				char* valy = strtok_s(NULL, " \t\n\r", &next_token);
				char* valz = strtok_s(NULL, " \t\n\r", &next_token);
				vi.x = (float)atof(valx);
				vi.y = (float)atof(valy);
				vi.z = (float)atof(valz);
				_normals.push_back(vi);
			} else if (key[0]=='v' && key[1]=='t') {
			} else if (key[0]=='v') {
				glm::vec4 vi(0,0,0,1);
				char* valx = strtok_s(NULL, " \t\n\r", &next_token);
				char* valy = strtok_s(NULL, " \t\n\r", &next_token);
				char* valz = strtok_s(NULL, " \t\n\r", &next_token);
				vi.x = (float)atof(valx);
				vi.y = (float)atof(valy);
				vi.z = (float)atof(valz);
				_points.push_back(vi);
			} else if (key[0]='f') {
				char* val = strtok_s(NULL, " \t\n\r", &next_token);
				std::vector<unsigned int> face;
				for(;val!=NULL;val = strtok_s(NULL, " \t\n\r", &next_token)) {
					unsigned int iv[3] = { 0, 0, 0 };	// 0:vtx, 1:tex, 2:nrm
					if (sscanf_s(val, "%d/%d/%d", iv+0,iv+1,iv+2)==3) {
						_triangles.push_back(iv[0]-1);
						face.push_back(iv[0]-1);
					} else if (sscanf_s(val, "%d/%d", iv+0,iv+1)==2) {
						_triangles.push_back(iv[0]-1);
						face.push_back(iv[0]-1);
					} else if (sscanf_s(val, "%d//%d", iv+0,iv+2)==2) {
						_triangles.push_back(iv[0]-1);
						face.push_back(iv[0]-1);
					} else if (sscanf_s(val, "%d", iv+0)==1) {
						_triangles.push_back(iv[0]-1);
						face.push_back(iv[0]-1);
					}
				}
				_faces.push_back(face);
				_nbEdges = face.size();
			}
		}
	}
	fclose(fdat);
}