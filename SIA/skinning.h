#ifndef _SKINNING_H_
#define _SKINNING_H_

#include "mesh.h"
#include "skeleton.h"
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>

class Skinning
{
public :
	// inputs :
	Mesh *_skin;			// mesh to animate
	Skeleton *_skel;		// animation skeleton

	int _nbJoints;			// number of joints in _skel
	int _nbVtx;				// number of vertices of _skin

	// required for skinning :
	std::vector<std::vector<double>> _weights;		// weights[i][j] = influence of j-th joint on i-th vertex
	std::vector<Skeleton*> _joints;					// animation skeleton flattened
	std::vector<glm::mat4> _transfoInit;			// initial global transformation of bones
	std::vector<glm::mat4> _transfoInitInv;			// inverse of initial global transformation of bones
	std::vector<glm::vec4> _pointsInit;				// initial position of vertices
	std::vector<glm::vec4> _posBonesInit;			// initial global position of bone
	std::vector<glm::mat4> _transfoCurr;			// current global transformation of bones

	int _meth;	//method to compute weights 1 : computeWeights(), 0 : load from Maya
	
public :
	Skinning() {
		_skin = NULL;
		_skel = NULL;
		_nbJoints = 0;
		_meth = 1;
	}
	~Skinning() {
	}

	// initialize attributes :
	void init();

	// build _joints :
	void getJoints(Skeleton *skel);
	// build _posBonesInit :
	void getBonesPos(Skeleton *skel, int *idx);
	// build _transfoCurr :
	void computeTransfo(Skeleton *skel, int *idx);

	// build _weights :
	void computeWeights();					// compute from data
	void loadWeights(std::string filename);	// load from file extracted from Maya
	// re-initialize weights :
	void recomputeWeights();

	// color the vertices of _skel according to the influence of jointName :
	void paintWeights(std::string jointName);
	
	// animation :
	void animate();
	// apply skinning to _skel :
	void applySkinning();

};

#endif