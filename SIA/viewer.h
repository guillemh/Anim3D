#ifndef _VIEWER_H_
#define _VIEWER_H_
#include <GL/glew.h>

#include <qglviewer.h>
#include <QKeyEvent>
#include "skeleton.h"
#include "mesh.h"
#include "skinning.h"

#define _SKINNING_ON 1
#define _SKINNING_GPU 0



class Viewer : public QGLViewer
{
private :

	int _nframes;			// number of frames in sequence
	int _iframe;			// current frame

	Skeleton* _root;		// animation skeleton
	Mesh* _human;			// mesh to animate

	Skinning *_skinning;	// skinning data


protected :

  virtual void draw();
  virtual void init();
  virtual void animate();
  virtual QString helpString() const;
  virtual void keyPressEvent(QKeyEvent *e);

};

#endif