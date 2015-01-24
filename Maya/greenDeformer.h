#include <maya/MPxNode.h>
#include <maya/MTypeId.h> 
#include <maya/MPointArray.h>

class greenDeformer
  : public MPxNode
{
public:
    greenDeformer();
    ~greenDeformer();

    static void*        creator();
    static MStatus      initialize();

    //----------------------------------------
    // MPxNode overrides

    virtual MStatus     compute(const MPlug& plug, MDataBlock& data);

    //----------------------------------------
    // Helper methods

    // ...

public:
    static MTypeId      id;

    //----------------------------------------
    // Attributes

    static MObject      s_input_geom;
    static MObject      s_input_cage;
    static MObject      s_output_geom;

private:
	bool weight_initialized;
	double ***phi;
	double **psi;
	MVector *u;
	MVector *v;
	double *area;
	MPointArray vertex_array;

	MObject createCube(MObject& outData, MStatus& stat);
};

