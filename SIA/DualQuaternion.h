#include <qglviewer.h>

using qglviewer::Quaternion;

class DualQuaternion
{
public:
	DualQuaternion(void);
	~DualQuaternion(void);

	DualQuaternion(Quaternion quat, Quaternion dual);
	DualQuaternion(const DualQuaternion& dq);

	void normalize();
	void conjugate();
	void dualConjugate();

	void operator += (const DualQuaternion& dq);

	Quaternion _quat;
	Quaternion _dual;
};

DualQuaternion operator+(const DualQuaternion & a, const DualQuaternion & b);
DualQuaternion operator*(double scalar, const DualQuaternion & dq);
