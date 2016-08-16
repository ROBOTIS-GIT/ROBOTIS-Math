/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*
 * bezier_curve.cpp
 *
 *  Created on: 2016. 8. 12.
 *      Author: Jay Song
 */

#include "robotis_math/bezier_curve.h"

using namespace robotis_framework;

BezierCurve::BezierCurve()
{ }

BezierCurve::~BezierCurve()
{  }

void BezierCurve::setBezierControlPoints(const std::vector<Point2D>& points)
{
  control_points_.clear();
  control_points_ = points;
}

Point2D BezierCurve::getPoint(double t)
{
  if(t > 1)
    t = 1;
  else if(t < 0)
    t = 0;

  int points_num = control_points_.size();
  Point2D point_at_t;
  point_at_t.x = 0;
  point_at_t.y = 0;

  if(points_num < 2)
    return point_at_t;

  point_at_t.x = control_points_[0].x * powDI(1-t, points_num - 1);
  point_at_t.y = control_points_[0].y * powDI(1-t, points_num - 1);

  for(unsigned int i = 1; i < (points_num - 1); i++)
  {
    point_at_t.x += control_points_[i].x * combination(points_num - 1, i)* powDI(1-t, points_num - 1 - i)*powDI(t, i);
    point_at_t.y += control_points_[i].y * combination(points_num - 1, i)* powDI(1-t, points_num - 1 - i)*powDI(t, i);
  }

  point_at_t.x += control_points_[points_num - 1].x * powDI(t, points_num - 1);
  point_at_t.y += control_points_[points_num - 1].y * powDI(t, points_num - 1);

  return point_at_t;
}

