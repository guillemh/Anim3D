#include "skinning.h"

using namespace std;

void Skinning::init() {
	if (_skin==NULL) return;
	if (_skel==NULL) return;

	// Compute number of joints :
	getJoints(_skel);
	_nbJoints = _joints.size();

	// Get mesh info :
	_nbVtx = _skin->_points.size();
	_weights.resize(_nbVtx);
	_pointsInit.resize(_nbVtx);
	for (int iv = 0 ; iv < _nbVtx ; iv++) {
		_weights[iv].resize(_nbJoints, 0);
		_pointsInit[iv] = _skin->_points[iv];
		_pointsInit[iv][3] = 1.0;
	}

	// Get transfo joint info :
	_transfoInit.resize(_nbJoints);
	_transfoInitInv.resize(_nbJoints);
	_transfoCurr.resize(_nbJoints);
	int idx = 0;
	glPushMatrix();
	glLoadIdentity();
	computeTransfo(_skel, &idx);
	glPopMatrix();
	for (unsigned int i = 0 ; i < _transfoCurr.size() ; i++) {
		_transfoInit[i] = _transfoCurr[i];
		_transfoInitInv[i] = glm::inverse(_transfoInit[i]);
	}

	// Get bones pose info :
	idx = 0;
	_posBonesInit.resize(_nbJoints);
	getBonesPos(_skel, &idx);

	// Compute weights :
	if (_meth)
		computeWeights();
	else 
		loadWeights("data/skinning.txt");

	// Test skinning :
	animate();
}

void Skinning::recomputeWeights() {
	if (_skin==NULL) return;
	if (_skel==NULL) return;

	// Compute weights :
	if (_meth) {
		cout << "computing weights\n";
		computeWeights();
		cout << "-- weights computed" << endl;
	} else {
		cout << "loading weights\n";
		loadWeights("data/skinning.txt");
		cout << "-- weights loaded" << endl;
	}

	// Test skinning :
	animate();
}

void Skinning::getJoints(Skeleton *skel) {
	_joints.push_back(skel);
	for (unsigned int ichild = 0 ; ichild < skel->_children.size() ; ichild++) {
		getJoints(skel->_children[ichild]);
	}
}
void Skinning::getBonesPos(Skeleton *skel, int *idx) {
	int i0 = (*idx);
	qglviewer::Vec pos(_transfoInit[i0][3][0], _transfoInit[i0][3][1], _transfoInit[i0][3][2]);
	for (unsigned int ichild = 0 ; ichild < skel->_children.size() ; ichild++) {
		(*idx)++;
		pos+=qglviewer::Vec(_transfoInit[(*idx)][3][0], _transfoInit[(*idx)][3][1], _transfoInit[(*idx)][3][2]);
		getBonesPos(skel->_children[ichild], idx);
	}
	pos/=(float)(skel->_children.size()+1);
	_posBonesInit[i0] = glm::vec4(pos.x, pos.y, pos.z, 1.0);
}

void Skinning::computeTransfo(Skeleton *skel, int *idx) {
	int i0 = (*idx);
	glPushMatrix();
	{
		glTranslatef(skel->_offX, skel->_offY, skel->_offZ);
		glTranslatef(skel->_curTx, skel->_curTy, skel->_curTz);
		skel->rotateSkeleton();

		float ptr[16];
		glGetFloatv(GL_MODELVIEW_MATRIX, ptr);
		int i = 0;
		for (int j = 0 ; j < 4 ; j++) {
			for (int k = 0 ; k < 4 ; k++) {
				_transfoCurr[(*idx)][k][j] = ptr[i];
				i++;
			}
		}
		for (unsigned int ichild = 0 ; ichild < skel->_children.size() ; ichild++) {
			(*idx)++;
			computeTransfo(skel->_children[ichild], idx);
		}
	}
	glPopMatrix();
	_transfoCurr[i0] = glm::transpose(_transfoCurr[i0]);
}


void Skinning::computeWeights() {
	if (_skin==NULL) return;
	if (_skel==NULL) return;
	// Compute the distance of each vertex of the mesh to all the bones
	// The nearest bone will be the one to influence the vertex
	for (int i = 0; i < _nbVtx; i++) {
		glm::vec4 vertex = _pointsInit[i];
		double minDist = numeric_limits<double>::infinity();
		int minBone;
		for (int j = 0; j < _nbJoints; j++) {
			// Re-initialize the influence to 0
			_weights[i][j] = 0;
			glm::vec4 bone = _posBonesInit[j];
			double dist = glm::distance(vertex, bone);
			if (dist < minDist) {
				minDist = dist;
				minBone = j;
			}
		}
		_weights[i][minBone] = 1;
	}
}

void Skinning::loadWeights(std::string filename) {
	std::vector<float> bone_indexA;
	std::vector<float> bone_weightA;
	FILE *file; fopen_s(&file, filename.data(), "r");
	if (!file) return;
	char * pch, *next_token;
	const int line_size = 600;
	char line[line_size];
	int iV = 0;
	while (!feof(file)) {
		// for each line i.e. for each vertex :
		if (fgets(line, line_size, file)) {
			int iJt = 0;
			float x;
			pch = strtok_s(line," ", &next_token);
			while (pch != NULL) {
				// for each number i.e. for each joint :
				if (pch[0]=='\n'){
				} else {
					x = (float)atof(pch);
					_weights[iV][iJt] = x;
				}
				pch = strtok_s(NULL, " ", &next_token);
				iJt++;
			}
			iV++;
		}		
	}
	fclose(file);
}

void Skinning::paintWeights(std::string jointName) {
	if (_skin==NULL) return;
	if (_skel==NULL) return;
	// Get the index of the corresponding joint in _joints
	int jointIndex;
	for (int j = 0; j < _nbJoints; j++) {
		if (!_joints.at(j)->_name.compare(jointName)) {
			jointIndex = j;
			break;
		}
	}

	_skin->_colors = std::vector<glm::vec4>(_nbVtx);
	// Color the vertices depending on the joint's weight on them
	for (int i = 0; i < _nbVtx; i++) {
		double weight = _weights[i][jointIndex];
		if (weight == 0) {
			_skin->_colors[i] = (glm::vec4(0., 0., 0., 1.));
		} else {
			_skin->_colors[i] = (glm::vec4(weight, 0., 0., 1.));
		}
	}
}

void Skinning::animate() {
	if (_skin==NULL) return;
	if (_skel==NULL) return;

	// Animate bones :
	int idx = 0;
	glPushMatrix();
	glLoadIdentity();
	computeTransfo(_skel, &idx);
	glPopMatrix();

	// Animate skin :
#if _SKINNING_GPU
#else
	applySkinning();
#endif
}

void Skinning::applySkinning() {
	// Loop on all the vertices, and change their position
	// according to the corresponding weights
	for (int i = 0; i < _nbVtx; i++) {
		glm::vec4 newPos;
		for (int j = 0; j < _nbJoints; j++) {
			newPos += _weights[i][j] * _transfoCurr[j] * _transfoInitInv[j] * _pointsInit[i];
		}
		_skin->_points[i] = newPos;
	}
}