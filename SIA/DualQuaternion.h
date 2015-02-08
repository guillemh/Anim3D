#include <qglviewer.h>

using qglviewer::Quaternion;

class DualQuaternion
{
public:
	DualQuaternion(void);
	~DualQuaternion(void);

	DualQuaternion(const Quaternion& quat, const Quaternion& dual);
	DualQuaternion(const DualQuaternion& dq);

	void normalize();
	void conjugate();
	void dualConjugate();

	void operator += (const DualQuaternion& dq);

	Quaternion _quat;
	Quaternion _dual;
};

DualQuaternion operator+(const DualQuaternion & dq1, const DualQuaternion & dq2);
DualQuaternion operator*(double scalar, const DualQuaternion & dq);
DualQuaternion operator*(const DualQuaternion & dq1, const DualQuaternion & dq2);
