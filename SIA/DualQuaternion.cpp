#include "DualQuaternion.h"


DualQuaternion::DualQuaternion(void) {
	_quat = Quaternion(0.0, 0.0, 0.0, 1.0);
	_dual = Quaternion(0.0, 0.0, 0.0, 0.0);
}

DualQuaternion::~DualQuaternion(void) {}

DualQuaternion::DualQuaternion(const Quaternion& quat, const Quaternion& dual) {
	_quat = quat;
	_dual = dual;
}

DualQuaternion::DualQuaternion(const DualQuaternion& dq) {
	_quat = dq._quat;
	_dual = dq._dual;
}

void DualQuaternion::normalize() {
	double norm2 = Quaternion::dot(_quat, _quat);
	double norm = _quat.normalize();
	_dual = Quaternion(_dual[0]/norm, _dual[1]/norm, _dual[2]/norm, _dual[3]/norm);
}

void DualQuaternion::conjugate() {
	_quat.invert();
	_dual.invert();
}

void DualQuaternion::dualConjugate() {
	_dual.negate();
}

void DualQuaternion::operator += (const DualQuaternion& dq) {
	Quaternion quat = Quaternion(_quat[0] + dq._quat[0], _quat[1] + dq._quat[1], _quat[2] + dq._quat[2], _quat[3] + dq._quat[3]);
	Quaternion dual = Quaternion(_dual[0] + dq._dual[0], _dual[1] + dq._dual[1], _dual[2] + dq._dual[2], _dual[3] + dq._dual[3]);
	_quat = quat;
	_dual = dual;
}

DualQuaternion operator+(const DualQuaternion & dq1, const DualQuaternion & dq2) {
	Quaternion quat = Quaternion(dq1._quat[0] + dq2._quat[0], dq1._quat[1] + dq2._quat[1], dq1._quat[2] + dq2._quat[2], dq1._quat[3] + dq2._quat[3]);
	Quaternion dual = Quaternion(dq1._dual[0] + dq2._dual[0], dq1._dual[1] + dq2._dual[1], dq1._dual[2] + dq2._dual[2], dq1._dual[3] + dq2._dual[3]);
	return DualQuaternion(quat, dual);
}

DualQuaternion operator*(double scalar, const DualQuaternion & dq) {
	DualQuaternion res = DualQuaternion(dq);
	res._quat = Quaternion(scalar * res._quat[0], scalar * res._quat[1], scalar * res._quat[2], scalar * res._quat[3]);
	res._dual = Quaternion(scalar * res._dual[0], scalar * res._dual[1], scalar * res._dual[2], scalar * res._dual[3]);
	return res;
}

DualQuaternion operator*(const DualQuaternion & dq1, const DualQuaternion & dq2) {
	Quaternion quat(dq2._quat*dq1._quat);
	Quaternion dual((dq2._dual*dq1._quat)[0] + (dq2._quat*dq1._dual)[0], (dq2._dual*dq1._quat)[1] + (dq2._quat*dq1._dual)[1], 
		(dq2._dual*dq1._quat)[2] + (dq2._quat*dq1._dual)[2], (dq2._dual*dq1._quat)[3] + (dq2._quat*dq1._dual)[3]);
	return DualQuaternion(quat, dual);
}
