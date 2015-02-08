#include "skeleton.h"
#include <skeletonIO.h>
#include <qglviewer.h>

using namespace std;

Skeleton* Skeleton::createFromFile(std::string fileName) {
	Skeleton* root = NULL;
	cout << "Loading from " << fileName << endl;

	ifstream inputfile(fileName.data());
	if(inputfile.good()) {
		while(!inputfile.eof()) {
			string buf;	
			inputfile >> buf;
			if(!buf.compare("HIERARCHY")) {
				root = readHierarchy(inputfile);
			}
		}
		inputfile.close();
	} else {
		std::cerr << "Failed to load the file " << fileName.data() << std::endl;
		fflush(stdout);
	}

	cout << "file loaded" << endl;

	return root;
}


void drawBone(Skeleton *child) 
{
	qglviewer::Vec v0(0,0,1);
	qglviewer::Vec v1(child->_offX, child->_offY, child->_offZ);
	qglviewer::Vec vRot = v0^v1; vRot.normalize();
	float angle = acosf((v0*v1)/(v0.norm()*v1.norm()))*180.0/M_PI;
	float height = (v1-v0).norm();
	float radius = 0.1f;
	glPushMatrix();
	{
		glRotatef(angle, vRot.x, vRot.y, vRot.z);
		gluCylinder(gluNewQuadric(), 0.1, 0.1, height, 5, 5);
	}
	glPopMatrix();

}

void Skeleton::rotateSkeleton() {
	switch (_rorder) {
	case roXYZ :
		glRotatef(_curRx, 1, 0, 0);
		glRotatef(_curRy, 0, 1, 0);
		glRotatef(_curRz, 0, 0, 1);
		break;
	case roYZX :
		glRotatef(_curRy, 0, 1, 0);
		glRotatef(_curRz, 0, 0, 1);
		glRotatef(_curRx, 1, 0, 0);
		break;
	case roZXY :
		glRotatef(_curRz, 0, 0, 1);
		glRotatef(_curRx, 1, 0, 0);
		glRotatef(_curRy, 0, 1, 0);
		break;
	case roXZY :
		glRotatef(_curRx, 1, 0, 0);
		glRotatef(_curRz, 0, 0, 1);
		glRotatef(_curRy, 0, 1, 0);
		break;
	case roYXZ :
		glRotatef(_curRy, 0, 1, 0);
		glRotatef(_curRx, 1, 0, 0);
		glRotatef(_curRz, 0, 0, 1);
		break;
	case roZYX :
		glRotatef(_curRz, 0, 0, 1);
		glRotatef(_curRy, 0, 1, 0);
		glRotatef(_curRx, 1, 0, 0);
		break;
	}
}
void Skeleton::draw() 
{
	glPushMatrix();
	{
		// Set good reference frame :
		glTranslatef(_offX, _offY, _offZ);
		// Use current value of dofs :
		glTranslatef(_curTx, _curTy, _curTz);
		rotateSkeleton();
		// Draw articulation :
		glColor3f(1,0,0),
			gluSphere(gluNewQuadric(), 0.25, 10, 10);
		// Draw bone and children :
		glColor3f(0,0,1);
		for (unsigned int ichild = 0 ; ichild < _children.size() ; ichild++) {
			drawBone(_children[ichild]);
			_children[ichild]->draw();
		}
	}
	glPopMatrix();
}

void Skeleton::animate(int iframe) 
{
	// Update dofs :
	_curTx = 0; _curTy = 0; _curTz = 0;
	_curRx = 0; _curRy = 0; _curRz = 0;
	for (unsigned int idof = 0 ; idof < _dofs.size() ; idof++) {
		if(!_dofs[idof].name.compare("Xposition")) _curTx = _dofs[idof]._values[iframe];
		if(!_dofs[idof].name.compare("Yposition")) _curTy = _dofs[idof]._values[iframe];
		if(!_dofs[idof].name.compare("Zposition")) _curTz = _dofs[idof]._values[iframe];
		if(!_dofs[idof].name.compare("Zrotation")) _curRz = _dofs[idof]._values[iframe];
		if(!_dofs[idof].name.compare("Yrotation")) _curRy = _dofs[idof]._values[iframe];
		if(!_dofs[idof].name.compare("Xrotation")) _curRx = _dofs[idof]._values[iframe];
	}	
	// Animate children :
	for (unsigned int ichild = 0 ; ichild < _children.size() ; ichild++) {
		_children[ichild]->animate(iframe);
	}
}

void Skeleton::eulerToMatrix(double rx, double ry, double rz, int rorder, glm::mat3 *R)
{
	glm::mat3 mx, my, mz;
	mx = glm::mat3(
		1, 0, 0,
		0, cos(rx), -sin(rx),
		0, sin(rx), cos(rx)
		);
	my = glm::mat3(
		cos(ry), 0, sin(ry),
		0, 1, 0,
		-sin(ry), 0, cos(ry)
		);
	mz = glm::mat3(
		cos(rz), -sin(rz), 0,
		sin(rz), cos(rz), 0,
		0, 0, 1
		);
	switch (rorder) {
	case roXYZ :
		*R = mx*my*mz;
		break;
	case roYZX :
		*R = my*mz*mx;
		break;
	case roZXY :
		*R = mz*mx*my;
		break;
	case roXZY :
		*R = mx*mz*my;
		break;
	case roYXZ :
		*R = my*mx*mz;
		break;
	case roZYX :
		*R = mz*my*mx;
		break;
	}
}
void Skeleton::matrixToQuaternion(glm::mat3 R, qglviewer::Quaternion *q)
{
	double q0 = sqrt(1.0 + R[0][0] + R[1][1] + R[2][2]) / 2.0;
	double q1 = (R[2][1] - R[1][2])/(4 * q0);
	double q2 = (R[0][2] - R[2][0])/(4 * q0);
	double q3 = (R[1][0] - R[0][1])/(4 * q0);
	q->setValue(q1, q2, q3, q0);
	/*
	qglviewer::Quaternion qglQuat;
	double m[3][3] = {
	{R[0][0], R[0][1], R[0][2]},
	{R[1][0], R[1][1], R[1][2]},
	{R[2][0], R[2][1], R[2][2]}
	};
	qglQuat.setFromRotationMatrix(m);
	*/
}
void Skeleton::quaternionToAxisAngle(qglviewer::Quaternion q, qglviewer::Vec *vaa)
{
	//float phi = atan2(q[0]*q[2] + q[1]*q[3], -(q[1]*q[2] - q[0]*q[3]));
	//float theta = acos(-q[0]*q[0] - q[1]*q[1] + q[2]*q[2] +q[3]*q[3]);
	//float psi = atan2(q[0]*q[2] - q[1]*q[3], q[1]*q[2] + q[0]*q[3]);
	if (q[3] > 1) {
		q.normalize(); // if w>1 acos and sqrt will produce errors, this cant happen if quaternion is normalised
	}
	double angle = 2 * acos(q[3]);
	double s = sqrt(1-q[3]*q[3]); // assuming quaternion normalised then w is less than 1, so term always positive.

	double ex;
	double ey;
	double ez;
	if (s < 0.001) { // test to avoid divide by zero, s is always positive due to sqrt
		// if s close to zero then direction of axis not important
		ex = q[0]; // if it is important that axis is normalised then replace with x=1; y=z=0;
		ey = q[1];
		ez = q[2];
	} else {
		ex = q[0] / s; // normalise axis
		ey = q[1] / s;
		ez = q[2]/ s;
	}
	ex *= angle;
	ey *= angle;
	ez *= angle;
	vaa->setValue(ex, ey, ez);
	/*
	qglviewer::Vec v;
	float f;
	q.getAxisAngle(v, f);
	*/
}
void Skeleton::eulerToAxisAngle(double rx, double ry, double rz, int rorder, qglviewer::Vec *vaa)
{
	// Euler -> matrix :
	glm::mat3 R;
	eulerToMatrix(M_PI*rx/180.0, M_PI*ry/180.0, M_PI*rz/180.0, rorder, &R);
	// matrix -> quaternion :
	qglviewer::Quaternion q;
	matrixToQuaternion(R, &q);
	// quaternion -> axis/angle :
	quaternionToAxisAngle(q, vaa);
}

void Skeleton::nbDofs() {
	if (_dofs.empty()) return;

	// Increase tol?
	double tol = 5e-2;

	int nbDofsR = 0;

	// TO COMPLETE :
	int isImplemented = 1;
	int nbFrames = _dofs[0]._values.size();
	qglviewer::Vec vaaPrec, vaa;
	double angle, anglePrec;
	//animate(0);
	eulerToAxisAngle(_curRx, _curRy, _curRz, _rorder, &vaaPrec);
	anglePrec = vaaPrec.norm();
	vaaPrec.normalize();

	int i = 0;
	while (nbDofsR < 2 && i < nbFrames) {
		double rx, ry, rz;
		switch (_rorder) {
		case roXYZ :
			rx = _dofs[_dofs.size()-3]._values[i];
			ry = _dofs[_dofs.size()-2]._values[i];
			rz = _dofs[_dofs.size()-1]._values[i];
			break;
		case roYZX :
			rx = _dofs[_dofs.size()-1]._values[i];
			ry = _dofs[_dofs.size()-3]._values[i];
			rz = _dofs[_dofs.size()-2]._values[i];
			break;
		case roZXY :
			rx = _dofs[_dofs.size()-2]._values[i];
			ry = _dofs[_dofs.size()-1]._values[i];
			rz = _dofs[_dofs.size()-3]._values[i];
			break;
		case roXZY :
			rx = _dofs[_dofs.size()-3]._values[i];
			ry = _dofs[_dofs.size()-1]._values[i];
			rz = _dofs[_dofs.size()-2]._values[i];
			break;
		case roYXZ :
			rx = _dofs[_dofs.size()-2]._values[i];
			ry = _dofs[_dofs.size()-3]._values[i];
			rz = _dofs[_dofs.size()-1]._values[i];
			break;
		case roZYX :
			rx = _dofs[_dofs.size()-1]._values[i];
			ry = _dofs[_dofs.size()-2]._values[i];
			rz = _dofs[_dofs.size()-3]._values[i];
			break;
		default :
			rx = 0;
			ry = 0;
			rz = 0;
			break;
		}
		eulerToAxisAngle(rx, ry, rz, _rorder, &vaa);

		// Compare vaa and vaaPrec axes
		angle = vaa.norm();
		vaa.normalize();

		double val = (vaaPrec - vaa).norm();
		if (val > tol) {
			nbDofsR = 2;
		} else if ((anglePrec - angle) > tol) {
			nbDofsR = 1;
		}

		// Update vaaPrec
		vaaPrec = vaa;
		anglePrec = angle;

		i++;
	}

	if (!isImplemented) return;
	cout << _name << " : " << nbDofsR << " degree(s) of freedom in rotation\n";

	// Propagate to children :
	for (unsigned int ichild = 0 ; ichild < _children.size() ; ichild++) {
		_children[ichild]->nbDofs();
	}

}

void Skeleton::getRotations(vector<vector<double> >& rot) {
	if (_dofs.empty()) return;
	size_t nbFrames = _dofs[0]._values.size();
	for (size_t f = 0; f < rot.size(); f++) {
		for (size_t i = 0; i < _dofs.size(); i++) {
			rot[f].push_back(_dofs[i]._values[f]);
		}
	}

	for (unsigned int ichild = 0 ; ichild < _children.size() ; ichild++) {
		_children[ichild]->getRotations(rot);
	}
}

int Skeleton::getMotionPeriod() {
	size_t nbFrames = _dofs[0]._values.size();
	vector<vector<double> > rot(nbFrames);

	// Get all the rotation values for all joints at all frames
	getRotations(rot);

	// Initialization
	// What is the first frame we consider
	int frame0 = 0;
	// Difference of rotation btw second and first frames
	double angleDiffInit = 0;
	// Difference of rotation btw third and first frames
	double angleDiffPrec = 0;
	for (size_t i = 0; i < rot[frame0+2].size(); i++) {
		angleDiffInit += (rot[frame0+1][i]-rot[frame0+0][i])*(rot[frame0+1][i]-rot[frame0+0][i]);
		angleDiffPrec += (rot[frame0+2][i]-rot[frame0+0][i])*(rot[frame0+2][i]-rot[frame0+0][i]);
	}
	// How the difference of rotations evolves : are we going farther from the first rotation
	// or closer to it?
	double sign = angleDiffPrec - angleDiffInit;

	// Comparison for all the other frames

	// Frames at which a change of sign is detected
	vector<int> signChanges;
	for (size_t frame = frame0+3; frame < nbFrames; frame++) {
		double angleDiff = 0;
		for (size_t i = 0; i < rot[frame].size(); i++) {
			angleDiff += (rot[frame][i]-rot[0][i])*(rot[frame][i]-rot[0][i]);
		}

		// Detection of the change of sign
		if ((angleDiff-angleDiffPrec) * sign < 0) {
			// There's been a change of sign
			if (signChanges.empty()) {
				// First change
				signChanges.push_back(frame);
				sign = angleDiff - angleDiffPrec;
			} else {
				int lastChange = signChanges[signChanges.size()-1];
				if (frame-lastChange < 3) {
					// If change too soon, it's noise :
					// the last change was false
					signChanges.pop_back();
					sign *= -1;
				} else {
					// Change of sign detected
					signChanges.push_back(frame);
					sign = angleDiff - angleDiffPrec;
				}
			}

		}
		angleDiffPrec = angleDiff;
	}

	if (signChanges.size() < 2) {
		// Not enough changes detected
		return 0;
	} else if (signChanges.size() < 3) {
		// Semi-period = btw two following changes
		return 2*(signChanges[1] - signChanges[0]);
	} else {
		// Period = btw first and third changes
		return (signChanges[2] - signChanges[0]);
	}
}

int Skeleton::getMotionBegin() {
	// Look at the positions of the left feet, take the minimal one
	size_t nbFrames = _dofs[0]._values.size();

	// Get all the offsets until the foot
	std::vector<glm::vec4> footOffsets;
	Skeleton* cur = this;
	while (cur->_name.compare("End")) {
		footOffsets.push_back(glm::vec4(cur->_offX, cur->_offY, cur->_offZ, 1.0));
		cur = cur->_children[0];
	}

	size_t frame0 = 0;
	// Compute the position of the foot for each frame
	std::vector<glm::vec4> footPositions;
	int jointIndex = 0;
	for (size_t f = frame0; f < nbFrames; f++) {
		cur = this;
		jointIndex = 0;
		glm::vec4 initPos = glm::vec4(0.0, 0.0, 0.0, 1.0);
		double rx, ry, rz;
		for (size_t i = 0; i < cur->_dofs.size(); i++) {
			if(!cur->_dofs[i].name.compare("Zrotation")) rz = cur->_dofs[i]._values[f];
			if(!cur->_dofs[i].name.compare("Yrotation")) ry = cur->_dofs[i]._values[f];
			if(!cur->_dofs[i].name.compare("Xrotation")) rx = cur->_dofs[i]._values[f];
			if(!cur->_dofs[i].name.compare("Zposition")) initPos.z = cur->_dofs[i]._values[f];
			if(!cur->_dofs[i].name.compare("Yposition")) initPos.y = cur->_dofs[i]._values[f];
			if(!cur->_dofs[i].name.compare("Xposition")) initPos.x = cur->_dofs[i]._values[f];
		}
		glm::vec4 footPos(0.0, 0.0, 0.0, 1.0);// = initPos;
		glm::mat3 R;
		eulerToMatrix(M_PI*rx/180.0, M_PI*ry/180.0, M_PI*rz/180.0, RotateOrder::roXYZ, &R);
		glm::mat4 transfo = glm::mat4(R);
		transfo[3] = footOffsets[jointIndex];
		jointIndex++;
		footPos = transfo*footPos;
		cur = cur->_children[0];
		while (cur->_name.compare("End")) {
			for (size_t i = 0; i < cur->_dofs.size(); i++) {
				if(!cur->_dofs[i].name.compare("Zrotation")) rz = cur->_dofs[i]._values[f];
				if(!cur->_dofs[i].name.compare("Yrotation")) ry = cur->_dofs[i]._values[f];
				if(!cur->_dofs[i].name.compare("Xrotation")) rx = cur->_dofs[i]._values[f];
			}
			glm::mat3 R;
			eulerToMatrix(M_PI*rx/180.0, M_PI*ry/180.0, M_PI*rz/180.0, RotateOrder::roXYZ, &R);
			glm::mat4 transfo = glm::mat4(R);
			transfo[3] = footOffsets[jointIndex];
			jointIndex++;
			footPos = transfo*footPos;
			cur = cur->_children[0];
		}
		footPositions.push_back(footPos);
	}

	// Search the minimum of the Y position of the foot over all the frames
	int frameMinPos = 0;
	double minY = std::numeric_limits<double>::max();
	for (size_t f = 0; f < nbFrames-frame0; f++) {
		double yPos = footPositions[f].y;
		cout << f+frame0 << " : " << yPos << "; ";
		if (footPositions[f].y < minY) {
			minY = footPositions[f].y;
			frameMinPos = f+frame0;
			cout << endl;
		}
	}
	cout << endl;

	// Look for the first frame close enough to the min
	double tol = 0.05;
	for (size_t f = 0; f < nbFrames-frame0; f++) {
		if (abs(footPositions[f].y - minY) < tol) {
			return f+frame0;
		}
	}
	return frameMinPos;
}