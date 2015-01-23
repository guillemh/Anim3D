#include <greenDeformer.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnMesh.h>
#include <maya/MFloatPointArray.h>

#define SQRT_8 2.82842712475  

#define MCHECKERROR(STAT, MSG)      \
	if (MS::kSuccess != STAT)       \
		{                               \
		std::cout << MSG << std::endl;	\
		return MS::kFailure;        \
		}

#define ADD_ATTRIBUTE(ATTR)                         \
	MStatus ATTR##_stat;                            \
	ATTR##_stat = addAttribute(ATTR);               \
	MCHECKERROR(ATTR##_stat, "addAttribute: ATTR")

float epsilon = 0.0001;

MTypeId greenDeformer::id(0x60500);

MObject greenDeformer::s_input_geom;
MObject greenDeformer::s_input_cage;
MObject greenDeformer::s_output_geom;

greenDeformer::greenDeformer() {weight_initialized = false;}
greenDeformer::~greenDeformer() {
	//TODO : Free tous les pointeurs
}

void* greenDeformer::creator()
{
	return new greenDeformer();
}

MStatus greenDeformer::initialize()
{
	MFnTypedAttribute       t_attr;
	MFnNumericAttribute     n_attr;
	MStatus                 stat;

	// Input attributes

	s_input_geom = t_attr.create("inMesh", "i", MFnData::kMesh);
	MCHECKERROR(stat, " failed to create s_input_geom attribute");
	//t_attr.setReadable(false);
	ADD_ATTRIBUTE(s_input_geom);

	s_input_cage = t_attr.create("inCage", "ic", MFnData::kMesh);
	MCHECKERROR(stat, " failed to create s_input_cage attribute");
	//t_attr.setReadable(false);
	ADD_ATTRIBUTE(s_input_cage);
	std::cout << "lol" << std::endl;
	// Output attributes

	s_output_geom = t_attr.create("outMesh", "o", MFnData::kMesh);
	MCHECKERROR(stat, " failed to create s_output_geom attribute");
	t_attr.setStorable(false);
	ADD_ATTRIBUTE(s_output_geom);

	// Set the attribute dependencies

	attributeAffects(s_input_geom, s_output_geom);
	attributeAffects(s_input_cage, s_output_geom);

	return MS::kSuccess;
}

int sgn(float val) {
	return (0 < val) - (val < 0);
}

float GCTriInt(MVector p, MVector v1, MVector v2, MPoint eta) {
	float alpha = acos(((v2 - v1).normalize()*(p - v1).normalize()));
	float beta = acos(((v1 - p).normalize()*(v2 - p).normalize()));
	float lambda = pow(((p - v1).length()*sin(alpha)), 2);
	float c = pow((p - eta).length(), 2);
	float theta = M_PI - alpha;
	float I[2];
	for (int i = 0; i < 2; i++) {
		theta -= i*beta;
		float S = sin(theta);
		float C = cos(theta);

		I[i] = -sgn(S)*0.5*(2 * sqrt(c)*atan((sqrt(c)*C) / sqrt(lambda + pow(S, 2)*c))
			+ sqrt(lambda)*log(2 * sqrt(lambda)*pow(S, 2) / pow(1 - C, 2) * (1 - 2 * c*C / (c*(1 + C) + lambda + sqrt(pow(lambda, 2) + lambda*c*pow(S, 2))))));
	}
	return -1 / (4 * M_PI) * abs(I[0] - I[1] - sqrt(c)*beta);
}

//----------------------------------------
// MPxNode overrides

MStatus greenDeformer::compute(const MPlug& plug, MDataBlock& data)
{
	MStatus stat = MS::kUnknownParameter;
	std::cout << "lancement de la computation" << std::endl;

	if (plug == s_output_geom)
	{
		MDataHandle inputGeomData = data.inputValue(s_input_geom, &stat);
		MCHECKERROR(stat, " ERROR getting Geometry data");
		MDataHandle inputCageData = data.inputValue(s_input_cage, &stat);
		MCHECKERROR(stat, " ERROR getting cage data");
		MFnMesh geomMesh = inputGeomData.asMesh();
		MFnMesh cageMesh = inputCageData.asMesh();

		MIntArray triangles;
		MIntArray triangleCounts;
		stat = cageMesh.getTriangles(triangleCounts, triangles);
		MCHECKERROR(stat, " ERROR getting cage triangles");

		std::cout << "Il se passe des trucs de base" << std::endl;

		MFloatPointArray cagePoints;
		cageMesh.getPoints(cagePoints);

		const int numPoly = cageMesh.numPolygons();


		MFloatPointArray geomPoints;
		geomMesh.getPoints(geomPoints);
		int numPoints = geomPoints.length();

		MIntArray vertexList;
		MPoint zero(0, 0, 0, 0);
		MVector zerov();
		if (!weight_initialized) {


			u = new MVector[numPoly];
			v = new MVector[numPoly];
			area = new double[numPoly];


			phi = new float**[numPoly];
			psi = new float*[numPoly];
			for (int j = 0; j < numPoly; j++) {
				phi[j] = new float*[numPoints];
				psi[j] = new float[numPoints]();
			}
			for (int i = 0; i < numPoints; i++) {
				MPoint eta = geomPoints[i];
				for (int j = 0; j < numPoly; j++) {
					phi[j][i] = 0;
					cageMesh.getPolygonVertices(j, vertexList);
					MVector v[3];
					for (int l = 0; l < 3; l++) {
						v[l] = cagePoints[vertexList[l]] - eta;
					}
					MVector n;
					cageMesh.getPolygonNormal(j, n);
					MVector p = (v[0] * n)*n;
					int sign[3];
					float I[3];
					float II[3];
					MVector N[3];

					for (int l = 0; l < 3; l++) {
						sign[l] = sgn(((v[l] - p) ^ (v[(l + 1) % 3] - p))*n);

						I[l] = GCTriInt(p, v[l], v[(l + 1) % 3], zero);
						II[l] = GCTriInt(zero, v[(l + 1) % 3], v[l], zero);
						N[l] = v[(l + 1) % 3] ^ v[l];
						N[l].normalize();
					}
					float Isum = 0;
					for (int k = 0; k < 3; k++)
						Isum += sign[k] * I[k];
					Isum = -abs(Isum);
					psi[j][i] = -Isum;

					MVector w = Isum * n;
					for (int k = 0; k < 3; k++)
						w += II[k] * N[k];
					phi[j][i] = new float[3]();
					if (w.length() > epsilon)
						for (int l = 0; l < 3; l++)
							phi[j][i][l] = phi[j][i][l] + (N[(l + 1) % 3] * w) / (N[(l + 1) % 3] * v[l]);
				}
			}

			for (int i = 0; i < numPoly; i++){
				cageMesh.getPolygonVertices(i, vertexList);
				u[i] = cagePoints[vertexList[1]] - cagePoints[vertexList[0]];
				v[i] = cagePoints[vertexList[2]] - cagePoints[vertexList[1]];
				area[i] = 0.5*(u[i] ^ v[i]).length();
			}
			weight_initialized = true;
		}

		double *s = new double[numPoly];
		MVector *up = new MVector[numPoly];
		MVector *vp = new MVector[numPoly];

		for (int i = 0; i < numPoly; i++) {
			cageMesh.getPolygonVertices(i, vertexList);
			up[i] = cagePoints[vertexList[1]] - cagePoints[vertexList[0]];
			vp[i] = cagePoints[vertexList[2]] - cagePoints[vertexList[1]];
			s[i] = sqrt(pow(up[i].length()*v[i].length(), 2) + pow(u[i].length()*vp[i].length(), 2) - 2 * (up[i] * vp[i]) * (u[i] * v[i])) / (SQRT_8 * area[i]);
		}
		MVector *eta = new MVector[numPoints]();
		for (int i = 0; i < numPoints; i++) {
			eta[i] = MVector();
			for (int j = 0; j < numPoly; j++) {
				MVector n = up[j] ^ vp[j];
				n.normalize();
				eta[i] += psi[j][i] * s[j] * n;
				for (int l = 0; l < 3; l++) {
					eta[i] += phi[j][i][l] * vp[j];
				}
			}
		}
		std::cout << "ATTENTION JE VAIS CHARGER L'OUTPUT!" << std::endl;
		MDataHandle outputGeomData = data.outputValue(s_output_geom, &stat);
		MCHECKERROR(stat, " ERROR getting Output Geometry data");
		MFnMesh outputGeomMesh;
		MFloatPointArray vertex_array(geomPoints);
		for (int i = 0; i < numPoints; i++) {
			vertex_array.insert(MFloatPoint(eta[i]), i);
		}
		std::cout << "J'ai sette les points sur ma copie de geometrie finale" << std::endl;
		std::cout << "Coucou, on y est presque" << std::endl;
		outputGeomData.setMObject(outputGeomMesh.create(outputGeomMesh.numVertices(), outputGeomMesh.numPolygons(), vertex_array, triangleCounts,
			triangles));
		// std::cout << "FINAL CLEANSING" << std::endl;
		// data.setClean(plug);
		std::cout << "J'ai des problèmes de libération de mémoire, apparemment..." << std::endl;
		delete[] eta;
		delete[] s;
		delete[] up;
		delete[] vp;
		std::cout << "Nevermind." << std::endl;
	}
	stat = MS::kSuccess;
	std::cout << "Ayé, j'ai fini ! " << std::endl;
	return stat;
}

