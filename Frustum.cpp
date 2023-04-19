#include "Frustum.h"
#include <math.h>

Frustum::Frustum(LineSeg* l, LineSeg* r)
{
	left = l;
	right = r;
}

bool Frustum::Clip_Edge(LineSeg* edge)
{
	float distance_x = (float)edge->end[0] - (float)edge->start[0];
	float distance_y = (float)edge->end[1] - (float)edge->start[1];
	float left_x, left_y, right_x, right_y, s;

	s = edge->Cross_Param(*left);
	left_x = edge->start[0] + distance_x * s;
	left_y = edge->start[1] + distance_y * s;

	s = edge->Cross_Param(*right);
	right_x = edge->start[0] + distance_x * s;
	right_y = edge->start[1] + distance_y * s;

	char left_edge = left->Point_Side(right_x, right_y);
	char right_edge = right->Point_Side(left_x, left_y);

	if (left_edge == LineSeg::LEFT)
	{
		if (right_edge == LineSeg::RIGHT)
		{
			if (fminf(left_x, right_x) > edge->end[0] || fmaxf(left_x, right_x) < edge->start[0])
				return false;

			else if (fminf(left_y, right_y) > edge->end[1] || fmaxf(left_y, right_y) < edge->start[1])
				return false;

			if (fminf(left_x, right_x) > edge->start[0])
				edge->start[0] = fminf(left_x, right_x);

			if (fmaxf(left_x, right_x) < edge->end[0])
				edge->end[0] = fmaxf(left_x, right_x);

			if (fminf(left_y, right_y) > edge->start[1])
				edge->start[1] = fminf(left_y, right_y);

			if (fmaxf(left_y, right_y) < edge->end[1])
				edge->end[1] = fmaxf(left_y, right_y);

			return true;
		}

		if (right_edge == Edge::LEFT)
		{
			if (right_x < edge->start[0] || right_x > edge->end[0])
				return false;

			if (right_y < edge->start[1] || right_y > edge->end[1])
				return false;

			if (left->Point_Side(edge->start[0], edge->start[1]) == Edge::LEFT && right->Point_Side(edge->start[0], edge->start[1]) == Edge::RIGHT)
			{
				edge->end[0] = right_x;
				edge->end[1] = right_y;

				return true;
			}
			else if (left->Point_Side(edge->end[0], edge->end[1]) == Edge::LEFT && right->Point_Side(edge->end[0], edge->end[1]) == Edge::RIGHT)
			{
				edge->start[0] = right_x;
				edge->start[1] = right_y;
				return true;
			}
		}
	}
	else if (left_edge == Edge::RIGHT)
	{
		if (right_edge == Edge::RIGHT)
		{
			if (left_x < edge->start[0] || left_x > edge->end[0])
				return false;

			if (left_y < edge->start[1] || left_y > edge->end[1])
				return false;

			if (left->Point_Side(edge->start[0], edge->start[1]) == Edge::LEFT && right->Point_Side(edge->start[0], edge->start[1]) == Edge::RIGHT)
			{
				edge->end[0] = left_x;
				edge->end[1] = left_y;

				return true;
			}
			else if (left->Point_Side(edge->end[0], edge->end[1]) == Edge::LEFT && right->Point_Side(edge->end[0], edge->end[1]) == Edge::RIGHT)
			{
				edge->start[0] = left_x;
				edge->start[1] = left_y;

				return true;
			}
		}
	}

	return false;
}
