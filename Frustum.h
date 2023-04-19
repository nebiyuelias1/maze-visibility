#ifndef _FRUSTUM_H_
#define _FRUSTUM_H_

#include "src/LineSeg.h"

class Frustum
{
public:
	Frustum(LineSeg*, LineSeg*);

public:
	LineSeg* left;
	LineSeg* right;
public:
	bool Clip_Edge(LineSeg*);
};

#endif