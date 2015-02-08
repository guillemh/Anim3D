#include "skinning.h"
#include <glm/gtx/norm.hpp>

using namespace std;

void Skinning::init() {
	if (_skin==NULL) return;
	if (_skel==NULL) return;

	// Compute number of joints :
	_joints.clear();
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
	// Dual quaternions transform
	_dualQuatTransfoInit.resize(_nbJoints);
	_dualQuatTransfoCurr.resize(_nbJoints);
	for (unsigned int j = 0; j < _transfoInit.size(); j++) {
		_dualQuatTransfoInit[j] = _dualQuatTransfoCurr[j];
		DualQuaternion inv = _dualQuatTransfoInit[j];
	}

	// Get bones pose info :
	idx = 0;
	_posBonesInit.resize(_nbJoints);
	getBonesPos(_skel, &idx);

	// Compute weights :
	if (_meth == 1) {
		computeWeights(_skinningTypes::rigide);
		cout << "-- rigid skinning weights computed" << endl;
	} else if (_meth == 2) {
		computeWeights(_skinningTypes::lisse);
		cout << "-- smooth skinning weights computed" << endl;
	} else {
		loadWeights("data/skinning.txt");
		cout << "-- weights loaded" << endl;
	}

	// Test skinning :
	animate();
}

void Skinning::recomputeWeights() {
	if (_skin==NULL) return;
	if (_skel==NULL) return;

	// Compute weights :
	if (_meth == 1) {
		cout << "computing weights rigid skinning\n";
		computeWeights(_skinningTypes::rigide);
		cout << "-- rigid skinning weights computed" << endl;
	} else if (_meth == 2) {
		cout << "computing weights smooth skinning\n";
		computeWeights(_skinningTypes::lisse);
		cout << "-- smooth skinning weights computed" << endl;
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

double Skinning::geodesDistance(glm::vec3 vertex, int boneIndex) {
	glm::vec3 dist;
	glm::vec3 joint1 = glm::vec3(_transfoInit[boneIndex][3][0], _transfoInit[boneIndex][3][1], _transfoInit[boneIndex][3][2]);
	int secondIndex;
	if (boneIndex == _transfoInit.size()-1) {
		secondIndex = boneIndex-1;
	} else {
		secondIndex = boneIndex+1;
	}
	glm::vec3 joint2 = glm::vec3(_transfoInit[secondIndex][3][0], _transfoInit[secondIndex][3][1], _transfoInit[secondIndex][3][2]);
	float norm = glm::gtx::norm::l2Norm(joint2-joint1)*glm::gtx::norm::l2Norm(joint2-joint1);
	float p;
	if (norm != 0) {
		p = glm::dot((vertex-joint1), (joint2-joint1))/norm;
	} else {
		return glm::distance(vertex, joint1);
	}
	if (p < 0) {
		dist = joint1;
	} else if (p > 1) {
		dist = joint2;
	} else {
		dist = joint1 + p*(joint2-joint1);
	}
	return glm::distance(vertex, dist);
}

void Skinning::computeWeights(_skinningTypes skinningType) {
	if (_skin==NULL) return;
	if (_skel==NULL) return;

	switch (skinningType) {
	case _skinningTypes::rigide :
		// Skinning rigide
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
				glm::vec3 vert = glm::vec3(vertex);
				//double dist = geodesDistance(vert, j);
				double dist = glm::distance(vertex, bone);
				if (dist < minDist) {
					minDist = dist;
					minBone = j;
				}
			}
			_weights[i][minBone] = 1;
		}
		break;		
	case _skinningTypes::lisse :
		// Skinning lisse
		for (int i = 0; i < _nbVtx; i++) {
			glm::vec4 vertex = _pointsInit[i];
			double sumDist = 0;
			for (int j = 0; j < _nbJoints; j++) {
				glm::vec4 bone = _posBonesInit[j];
				glm::vec3 vert = glm::vec3(vertex);
				double dist = geodesDistance(vert, j);
				//double dist = glm::distance(vertex, bone);
				double weight = exp(-dist);
				_weights[i][j] = weight;
				sumDist += weight;
			}
			for (int j = 0; j < _nbJoints; j++) {
				_weights[i][j] /= sumDist;
			}
		}
		break;
	default:
		break;
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
		_skin->_colors[i] = (glm::vec4(weight, 0., 0., 1.));
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
	computeDualQuaternionTransform();
	glPopMatrix();

	// Animate skin :
#if _SKINNING_GPU
#else
	applySkinning();
#endif
}

// Question : bon algorithme de mise à jour pour les DQ?
void Skinning::applySkinning() {
	// Loop on all the vertices, and change their position
	// according to the corresponding weights
	for (int i = 0; i < _nbVtx; i++) {
		if (!_DUAL_QUAT) {
			glm::vec4 newPos;
			for (int j = 0; j < _nbJoints; j++) {
				newPos += _weights[i][j] * _transfoCurr[j] * _transfoInitInv[j] * _pointsInit[i];
			}
			_skin->_points[i] = newPos;
		} else {
			// With dual quaternions
			DualQuaternion dq;
			for (int j = 0; j < _nbJoints; j++) {
				dq += (_weights[i][j] * (_dualQuatTransfoCurr[j]*_dualQuatTransfoInit[j]));
			}
			// cf algo1 from paper
			Quaternion c0 = dq._quat;
			double norm = c0.normalize();
			Quaternion cE = Quaternion(dq._dual[0] / norm, dq._dual[1] / norm, dq._dual[2] / norm, dq._dual[3] / norm);
			qglviewer::Vec d0 = qglviewer::Vec(c0[0], c0[1], c0[2]);
			qglviewer::Vec dE = qglviewer::Vec(cE[0], cE[1], cE[2]);
			double a0 = c0[3];
			double aE = cE[3];

			qglviewer::Vec pos = qglviewer::Vec(_pointsInit[i].x, _pointsInit[i].y, _pointsInit[i].z);
			pos += 2*d0 ^ (d0^pos + a0*pos) + 2 * (a0*dE - aE*d0 + d0^dE);
			_skin->_points[i] = glm::vec4(pos.x, pos.y, pos.z, 1.0);
		}
	}
}

// To execute AFTER computeTransfo
void Skinning::computeDualQuaternionTransform() {
	for (int jointIndex = 0; jointIndex < _nbJoints; jointIndex++) {
		// Get the rotation matrix
		glm::mat3 R = glm::mat3(_transfoCurr[jointIndex]);
		// Compute the corresponding rotation quaternion
		Quaternion quat;
		_skel->matrixToQuaternion(R, &quat);
		quat.normalize();

		// Get the translation vector
		qglviewer::Vec translation = qglviewer::Vec(_transfoCurr[jointIndex][3][0], _transfoCurr[jointIndex][3][1], _transfoCurr[jointIndex][3][2]);

		// Compute the corresponding dual quaternion
		Quaternion dual = Quaternion(translation, 0.0) * quat;
		for (int i = 0; i < 4; i++) {
			dual[i] *= 0.5f;
		}
		
		_dualQuatTransfoCurr[jointIndex] = DualQuaternion(quat, dual);
	}
}

