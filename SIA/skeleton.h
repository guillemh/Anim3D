#ifndef _SKELETON_H_
#define _SKELETON_H_

#include <string>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <qglviewer.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

using std::vector;

class AnimCurve {
public :
	AnimCurve() {};
	~AnimCurve() {
		_values.clear();
	}
public :
	std::string name;					// name of dof
	std::vector<double> _values;		// different keyframes = animation curve
};


enum RotateOrder {roXYZ=0, roYZX, roZXY, roXZY, roYXZ, roZYX};

class Skeleton {
public :
	std::string _name;					// name of joint
	double _offX;						// initial offset in X
	double _offY;						// initial offset in Y
	double _offZ;						// initial offset in Z
	std::vector<AnimCurve> _dofs;		// keyframes : _animCurves[i][f] = i-th dof at frame f;
	double _curTx;						// current value of translation on X
	double _curTy;						// current value of translation on Y
	double _curTz;						// current value of translation on Z
	double _curRx;						// current value of rotation about X (deg)
	double _curRy;						// current value of rotation about Y (deg)
	double _curRz;						// current value of rotation about Z (deg)
	int _rorder;						// order of euler angles to reconstruct rotation
	std::vector<Skeleton*> _children;	// children of the current joint


public :
	// Constructor :
	Skeleton() {};
	// Destructor :
	~Skeleton() {
		_dofs.clear();
		_children.clear();
	}

	// Create from data :
	static Skeleton* create(std::string name, double offX, double offY, double offZ, Skeleton* parent) {
		Skeleton* child = new Skeleton();
		child->_name = name;
		child->_offX = offX;
		child->_offY = offY;
		child->_offZ = offZ;
		child->_curTx = 0;
		child->_curTy = 0;
		child->_curTz = 0;
		child->_curRx = 0;
		child->_curRy = 0;
		child->_curRz = 0;
		if(parent != NULL) {
			parent->_children.push_back(child);
		}
		return child;
	}

	// Load from file (.bvh) :	
	static Skeleton* createFromFile(std::string fileName);

	// Viewer methods :
	void draw();
	void rotateSkeleton();
	void animate(int iframe=0);

	// Analysis of degrees of freedom :
	static void eulerToMatrix(double rx, double ry, double rz, int rorder, glm::mat3 *R);
	static void matrixToQuaternion(glm::mat3 R, qglviewer::Quaternion *q);
	static void quaternionToAxisAngle(qglviewer::Quaternion q, qglviewer::Vec *vaa);
	static void eulerToAxisAngle(double rx, double ry, double rz, int rorder, qglviewer::Vec *vaa);
	void nbDofs();

	// Recursive : set the matrix of all rotations on the joints at all frames
	void getRotations(vector<vector<double> >& rot);
	// Determines the period of the movement in number of frames
	int getMotionPeriod();
	// Determines the frame at which a period begins
	int getMotionBegin();
};


#endif