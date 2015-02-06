#ifndef _VIEWER_H_
#define _VIEWER_H_
#include <GL/glew.h>

#include <qglviewer.h>
#include <QKeyEvent>
#include "skeleton.h"
#include "mesh.h"
#include "skinning.h"


#define _SKINNING_ON 0
#define _SKINNING_GPU 0

enum _interpolationTypes {
	linear,
	smoothstep,
	smootherstep
};

class Viewer : public QGLViewer
{
private :

	int _nframes;			// number of frames in sequence
	int _iframe;			// current frame

	Skeleton* _root;		// animation skeleton
	Mesh* _human;			// mesh to animate

	Skinning *_skinning;	// skinning data

	// PROJECT ADD
	double interpolationFunction(double initialPoint, double finalPoint, unsigned int currentFrame, unsigned int maximumFrame, _interpolationTypes interpolationType);
	void applyInterpolation(Skeleton* initialSkeletonInitialState, unsigned int initialPeriodFrames, unsigned int initialBestStartFrame,
		Skeleton* initialSkeletonFinalState, unsigned int finalPeriodFrames, unsigned int finalBestStartFrame,
		Skeleton* finalSkeleton, unsigned int beginningFrame);
	void duplicateSkeleton(Skeleton* skeletonToBeDuplicated, unsigned int beginningFrame, unsigned int endFrame);

protected :

  virtual void draw();
  virtual void init();
  virtual void animate();
  virtual QString helpString() const;
  virtual void keyPressEvent(QKeyEvent *e);

};

#endif