#ifndef _SKELETON_IO_H_
#define _SKELETON_IO_H

#include "skeleton.h"

Skeleton* readHierarchy(std::ifstream &inputfile);
void readJoint(std::ifstream &inputfile, Skeleton* parent);
void readMotion(std::ifstream &inputfile, Skeleton* root);
void readKeyFrame(std::ifstream &inputfile, Skeleton* skel);
void defineRotateOrder(Skeleton *skel);

#endif