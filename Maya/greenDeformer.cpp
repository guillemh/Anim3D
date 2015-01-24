#include <greenDeformer.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MFloatArray.h>
#include <maya/MFloatPointArray.h>

#define SQRT_8 2.82842712475  

#define MCHECKERROR(STAT, MSG)      \
	if (MS::kSuccess != STAT)       \
		{                               \
		cout << MSG << endl;       \
		cout << STAT << endl;      \
		return MS::kFailure;        \
		}

#define ADD_ATTRIBUTE(ATTR)                         \
	MStatus ATTR##_stat;                            \
	ATTR##_stat = addAttribute(ATTR);               \
	MCHECKERROR(ATTR##_stat, "addAttribute: ATTR")

double epsilon = 0.0001;

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
	cout << "Load greenDeformer" << endl;

	// Output attribute
	s_output_geom = t_attr.create("outMesh", "o", MFnData::kMesh);
	MCHECKERROR(stat, " failed to create s_output_geom attribute");
	t_attr.setStorable(false);
	ADD_ATTRIBUTE(s_output_geom);

	// Set the attribute dependencies

	attributeAffects(s_input_geom, s_output_geom);
	attributeAffects(s_input_cage, s_output_geom);

	return MS::kSuccess;
}

int sgn(double val) {
	return (0 < val) - (val < 0);
}

double GCTriInt(MVector p, MVector v1, MVector v2, MPoint eta) {
	double alpha = acos((v2 - v1).normal()*(p - v1).normal());
	double beta = acos(((v1 - p).normal()*(v2 - p).normal()));
	double lambda = pow(((p - v1).length()*sin(alpha)), 2);
	double c = pow((p - eta).length(), 2);
	double theta = M_PI - alpha;
	double I[2];
	for (int i = 0; i < 2; i++) {
		theta -= i*beta;
		double S = sin(theta);
		double C = cos(theta);

		I[i] = -sgn(S)*0.5*(2 * sqrt(c)*atan((sqrt(c)*C) / sqrt(lambda + pow(S, 2)*c))
			+ sqrt(lambda)*log(2 * sqrt(lambda)*pow(S, 2) / pow(1 - C, 2) * (1 - 2 * c*C / (c*(1 + C) + lambda + sqrt(pow(lambda, 2) + lambda*c*pow(S, 2))))));
		if (isnan(I[i])) {
			cout << "I["<<i<<"] NaN" << endl;
		}
	}
	return -1 / (4 * M_PI) * abs(I[0] - I[1] - sqrt(c)*beta);
}

//----------------------------------------
// MPxNode overrides

MStatus greenDeformer::compute(const MPlug& plug, MDataBlock& data)
{
	MStatus stat = MS::kUnknownParameter;
	cout << endl << "lancement de la computation" << endl;

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

		//MDataHandle outputData = data.outputValue(s_output_geom, &stat);
		//MCHECKERROR(stat,"ERROR getting outMesh"); 
		//// Copy the inMesh to the outMesh, so you can
		//// perform operations directly on outMesh
		////
		//outputData.set(inputGeomData.asMesh());
		//MFnMesh mesh = outputData.asMesh();
		//mesh.setName("coucou", false, &stat);
		//MCHECKERROR(stat,"ERROR set name"); 
		//outputData.setClean();


		MPointArray cagePoints;
		cageMesh.getPoints(cagePoints);

		const int numPoly = cageMesh.numPolygons();


		MPointArray geomPoints;
		geomMesh.getPoints(geomPoints);
		int numPoints = geomPoints.length();
		MIntArray polygons;
		MIntArray polyCounts;
		stat = geomMesh.getTriangles(polyCounts, polygons);
		MCHECKERROR(stat, " ERROR getting mesh triangles");


		MIntArray vertexList;

		if (!weight_initialized) {
			cout << "Initalize weights" << endl;
			u = new MVector[numPoly];
			v = new MVector[numPoly];
			area = new double[numPoly];

			phi = new double**[numPoly];
			psi = new double*[numPoly];
			for (int j = 0; j < numPoly; j++) {
				phi[j] = new double*[numPoints];
				psi[j] = new double[numPoints]();
			}
			for (int i = 0; i < numPoints; i++) {
				MPoint eta = geomPoints[i];
				for (int j = 0; j < numPoly; j++) {
					cageMesh.getPolygonVertices(j, vertexList);
					MVector v[3];
					for (int l = 0; l < 3; l++) {
						v[l] = cagePoints[vertexList[l]] - eta;
					}
					MVector n;
					cageMesh.getPolygonNormal(j, n);
					n.normalize();
					MVector p = (v[0] * n)*n;
					int sign[3];
					double I[3];
					double II[3];
					MVector N[3];

					for (int l = 0; l < 3; l++) {
						sign[l] = sgn(((v[l] - p) ^ (v[(l + 1) % 3] - p))*n);

						I[l] = GCTriInt(p, v[l], v[(l + 1) % 3], MPoint());
						II[l] = GCTriInt(MPoint(), v[(l + 1) % 3], v[l], MPoint());
						N[l] = v[(l + 1) % 3] ^ v[l];
						N[l].normalize();
					}
					double Isum = 0;
					for (int k = 0; k < 3; k++) {
						Isum += sign[k] * I[k];
					}
					Isum = -abs(Isum);
					psi[j][i] = -Isum;

					MVector w = Isum * n;
					for (int k = 0; k < 3; k++)
						w += II[k] * N[k];
					phi[j][i] = new double[3]();
					if (w.length() > epsilon)
						for (int l = 0; l < 3; l++) {
							phi[j][i][l] = phi[j][i][l] + (N[(l + 1) % 3] * w) / (N[(l + 1) % 3] * v[l]);
						}
				}
			}
			// Formule (14)
			for (int i = 0; i < numPoly; i++){
				cageMesh.getPolygonVertices(i, vertexList);
				u[i] = cagePoints[vertexList[1]] - cagePoints[vertexList[0]];
				v[i] = cagePoints[vertexList[2]] - cagePoints[vertexList[0]];
				area[i] = 0.5*(u[i] ^ v[i]).length();
			}
			weight_initialized = true;
		}
		// Formule (14)
		MFloatArray s;
		MVectorArray up;
		MVectorArray vp;
		vertex_array.clear();
		for (int i = 0; i < numPoly; i++) {
			cageMesh.getPolygonVertices(i, vertexList);
			up.append(cagePoints[vertexList[1]] - cagePoints[vertexList[0]]);
			vp.append(cagePoints[vertexList[2]] - cagePoints[vertexList[0]]);
			s.append(sqrt(pow(up[i].length()*v[i].length(), 2) + pow(u[i].length()*vp[i].length(), 2) - 2 * (up[i] * vp[i]) * (u[i] * v[i])) / (SQRT_8 * area[i]));
		}

		// Formule (4)
		for (int i = 0; i < numPoints; i++) {
			MPoint eta;
			for (int j = 0; j < numPoly; j++) {
				MVector n;
				cageMesh.getPolygonNormal(j, n);
				n.normalize();
				eta += MPoint(n) * psi[j][i] * s[j];
				cageMesh.getPolygonVertices(j, vertexList);
				for (int l = 0; l < 3; l++) {
					MVector vert = cagePoints[vertexList[l]];
					eta += MPoint(vert) * (phi[j][i][l]);
				}
			}
			vertex_array.append(eta);
		}
		MDataHandle outputGeomData = data.outputValue(s_output_geom, &stat);
		MCHECKERROR(stat, " ERROR getting Output Geometry data");

		///////////////////////////////////
		MFnMeshData dataCreator;
		MObject newOutputData = dataCreator.create(&stat);
		MCHECKERROR(stat, " ERROR dataCreator");
		MFnMesh outputGeomMesh;
		//createMesh(newOutputData, stat);
		//MObject newMesh = outputGeomMesh.create(geomPoints.length(), polyCounts.length(), geomPoints, polyCounts, polygons, newOutputData, &stat);
		const int nbVertices = vertex_array.length();
		const int nbFaces = polyCounts.length();
		const MPointArray vertices(vertex_array);
		//cout << "Vertices " << endl;
		//for (int i = 0; i < nbVertices; i++) {
		//	MPoint p = vertex_array[i];
		//	cout << " " << p(0) << " " << p(1) << " " << p(2) << endl;
		//}
		MIntArray polygonCounts(polyCounts);
		//cout << "polyCounts " << endl;
		//for (int i = 0; i < nbFaces; i++) {
		//	cout << polygonCounts[i] << " ";
		//}
		//cout << endl;
		MIntArray polys(polygons);
		//cout << "polygons " << endl;
		//for (int i = 0; i < polys.length(); i++) {
		//	cout << polys[i] << " ";
		//}
		//cout << endl;
		cout << "Lengths: " << endl;
		cout << "nbVertices " << nbVertices << endl;
		cout << "nbFacesCounts " << nbFaces << endl;
		cout << "nbFaces " << polys.length() << endl;
		//cout << "triangleCounts " << endl;
		//for (unsigned int i = 0; i < triangleCounts.length(); i++) {
		//	cout << triangleCounts[i] << " ";
		//}
		//cout << endl;	

		polygonCounts = MIntArray(polyCounts.length(), 3);
		//cout << "polyCounts " << endl;
		//for (unsigned int i = 0; i < polygonCounts.length(); i++) {
		//	cout << polygonCounts[i] << " ";
		//}
		//cout << endl;
		MObject newMesh = outputGeomMesh.create(nbVertices, nbFaces, vertices, polygonCounts, polys, newOutputData, &stat);
		MCHECKERROR(stat, " ERROR create");
		outputGeomData.set(newOutputData);
		///////////////////////////////////
		data.setClean(plug);
		//delete[] eta;
		//delete[] s;
		//delete[] up;
		//delete[] vp;

	} else {
		cout << "Rien à computer!" << endl;
	}
	stat = MS::kSuccess;
	cout << "Ayé, j'ai fini ! " << endl << endl;
	return stat;
}

MObject greenDeformer::createCube(MObject& outData, MStatus& stat)

{
	int numVertices;
	float cubeSize;
	MFloatPointArray points;
	MFnMesh meshFS;

	cubeSize = 2;

	const int numFaces = 6;
	numVertices = 8;
	const int numFaceConnects = 24;

	MFloatPoint vtx_1( -cubeSize, -cubeSize, -cubeSize );
	MFloatPoint vtx_2(  cubeSize, -cubeSize, -cubeSize );
	MFloatPoint vtx_3(  cubeSize, -cubeSize,  cubeSize );
	MFloatPoint vtx_4( -cubeSize, -cubeSize,  cubeSize );
	MFloatPoint vtx_5( -cubeSize,  cubeSize, -cubeSize );
	MFloatPoint vtx_6( -cubeSize,  cubeSize,  cubeSize );
	MFloatPoint vtx_7(  cubeSize,  cubeSize,  cubeSize );
	MFloatPoint vtx_8(  cubeSize,  cubeSize, -cubeSize );
	points.append( vtx_1 );
	points.append( vtx_2 );
	points.append( vtx_3 );
	points.append( vtx_4 );
	points.append( vtx_5 );
	points.append( vtx_6 );
	points.append( vtx_7 );
	points.append( vtx_8 );

	// Set up an array containing the number of vertices
	// for each of the 6 cube faces (4 verticies per face)
	//
	int face_counts[numFaces] = { 4, 4, 4, 4, 4, 4 };
	MIntArray faceCounts( face_counts, numFaces );

	// Set up and array to assign vertices from points to each face 
	//
	int face_connects[ numFaceConnects ] = {        0, 1, 2, 3,
		4, 5, 6, 7,
		3, 2, 6, 5,
		0, 3, 5, 4,
		0, 4, 7, 1,
		1, 7, 6, 2      };
	MIntArray faceConnects( face_connects, numFaceConnects );

	MObject newMesh = meshFS.create(numVertices, numFaces, points, faceCounts, faceConnects,
		outData, &stat);

	return newMesh;
}