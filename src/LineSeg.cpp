/************************************************************************
     File:        LineSeg.cpp

     Author:     
                  Stephen Chenney, schenney@cs.wisc.edu
     Modifier
                  Yu-Chi Lai, yu-chi@cs.wisc.edu

     Comment:    
						(c) 2001-2002 Stephen Chenney, University of Wisconsin at Madison
						Class header file for LineSeg class.
		

     Platform:    Visio Studio.Net 2003 (converted to 2005)

*************************************************************************/

#include "LineSeg.h"

const char  LineSeg::LEFT = 0;
const char  LineSeg::RIGHT = 1;
const char  LineSeg::ON = 2;


//**********************************************************************
//
// * Constructor from an edge
//======================================================================
LineSeg::
LineSeg(Edge *e)
//======================================================================
{
	start[0] = e->endpoints[Edge::START]->posn[Vertex::X];
	start[1] = e->endpoints[Edge::START]->posn[Vertex::Y];

	end[0] = e->endpoints[Edge::END]->posn[Vertex::X];
	end[1] = e->endpoints[Edge::END]->posn[Vertex::Y];
}

//**********************************************************************
//
// * Constructor for specifyng start and end point
//======================================================================
LineSeg::
LineSeg(float xs, float ys, float xe, float ye)
//======================================================================
{
	start[0] = xs;
	start[1] = ys;
	end[0] = xe;
	end[1] = ye;
}


//**********************************************************************
//
// * Return the parameter value at which this segment crosses the given
//   segment. This will return parameter values outside the range 0,1
//   THIS FUNCTION IS EXTREMELY USEFUL FOR CLIPPING, but it 
//   DOES NOT tell you whether the edge is "entering" or "leaving".
//   But you can use tests like Edge::Point_Side() to figure that out.
//======================================================================
float LineSeg::
Cross_Param(LineSeg e)
//======================================================================
{
	float   dx1, dy1, dx2, dy2;
	float   denom, s;

	// This computation comes from writing each segment in parametric form,
	// and solving a simulataneous equation to determine the parameter
	// value of the intersection point on this LineSeg.
	dx1 = e.end[0] - e.start[0];
	dy1 = e.end[1] - e.start[1];
	dx2 = end[0] - start[0];
	dy2 = end[1] - start[1];

	if ( ( denom = dx2 * dy1 - dy2 * dx1 ) == 0.0 )
		// Parallel segments.
		return 1.0e20f;

	s = ( e.start[0] - start[0] ) * dy1 - ( e.start[1] - start[1] ) * dx1;

	return s / denom;
}

char LineSeg::
Point_Side(float x, float y)
//=======================================================================
{
	float   det;

	det = start[0] * (end[1] - y) -
		start[1] * (end[0] - x) +
		end[0] * y - end[1] * x;

	if (det == 0.0)
		return ON;
	else if (det > 0.0)
		return LEFT;
	else
		return RIGHT;
}
