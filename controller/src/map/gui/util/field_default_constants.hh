//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
  \file    field_default_constants.h
  \brief   Definition of field dimensions
  \author  Stefan Zickler / Tim Laue, (C) 2009
*/
//========================================================================

#ifndef FIELD_DEFAULT_CONSTANTS_H
#define FIELD_DEFAULT_CONSTANTS_H

#include <vector>
#include "field.hh"
#include "gvector.hh"

namespace FieldConstantsRoboCup2015 {
    const double kFieldLength = 9000.0;
    const double kFieldWidth = 6000.0;
    const double kGoalWidth = 1000.0;
    const double kGoalDepth = 200.0;
    const double kBoundaryWidth = 250.0;

    // WR
    const double kCenterRadius = 500.0;
    const double kDefenseRadius = 1000.0;
    const double kDefenseStretch = 500.0;

    const double kLineThickness = 10.0;
    const double kXMax = (kFieldLength-2*kLineThickness)/2;
    const double kXMin = -kXMax;
    const double kYMax = (kFieldWidth-kLineThickness)/2;
    const double kYMin = -kYMax;

    const std::size_t kNumFieldLines = 8;
    const FieldLine kFieldLines[kNumFieldLines] = {
      FieldLine("TopTouchLine", kXMin-kLineThickness/2, kYMax, kXMax+kLineThickness/2, kYMax, kLineThickness),
      FieldLine("BottomTouchLine", kXMin-kLineThickness/2, kYMin, kXMax+kLineThickness/2, kYMin, kLineThickness),
      FieldLine("LeftGoalLine", kXMin, kYMin, kXMin, kYMax, kLineThickness),
      FieldLine("RightGoalLine", kXMax, kYMin, kXMax, kYMax, kLineThickness),
      FieldLine("HalfwayLine", 0, kYMin, 0, kYMax, kLineThickness),
      FieldLine("CenterLine", kXMin, 0, kXMax, 0, kLineThickness),
      FieldLine("LeftPenaltyStretch", kXMin+kDefenseRadius, -kDefenseStretch/2, kXMin+kDefenseRadius, kDefenseStretch/2, kLineThickness),
      FieldLine("RightPenaltyStretch", kXMax-kDefenseRadius, -kDefenseStretch/2, kXMax-kDefenseRadius, kDefenseStretch/2, kLineThickness),
    };

    const std::size_t kNumFieldArcs = 5;
    const FieldCircularArc kFieldArcs[kNumFieldArcs] = {
      FieldCircularArc("LeftFieldLeftPenaltyArc",   kXMin, kDefenseStretch/2,  kDefenseRadius-kLineThickness/2, 0,        M_PI/2,   kLineThickness),
      FieldCircularArc("LeftFieldRightPenaltyArc",  kXMin, -kDefenseStretch/2, kDefenseRadius-kLineThickness/2, 1.5*M_PI, 2*M_PI,   kLineThickness),
      FieldCircularArc("RightFieldLeftPenaltyArc",  kXMax, -kDefenseStretch/2, kDefenseRadius-kLineThickness/2, M_PI,     1.5*M_PI, kLineThickness),
      FieldCircularArc("RightFieldRightPenaltyArc", kXMax, kDefenseStretch/2,  kDefenseRadius-kLineThickness/2, M_PI/2,   M_PI,     kLineThickness),
      FieldCircularArc("CenterCircle",              0,     0,                  kCenterRadius-kLineThickness/2,  0,        2*M_PI,   kLineThickness)
    };
}

namespace FieldConstantsRoboCup2012 {
    const double kFieldLength = 6050.0;
    const double kFieldWidth = 4050.0;
    const double kGoalWidth = 700.0;
    const double kGoalDepth = 200.0;
    const double kBoundaryWidth = 250.0;

    // WR
    const double kCenterRadius = 500.0;
    const double kDefenseRadius = 800.0;
    const double kDefenseStretch = 350.0;

    const double kLineThickness = 10.0;
    const double kXMax = (kFieldLength-2*kLineThickness)/2;
    const double kXMin = -kXMax;
    const double kYMax = (kFieldWidth-kLineThickness)/2;
    const double kYMin = -kYMax;

    const std::size_t kNumFieldLines = 7;
    const FieldLine kFieldLines[kNumFieldLines] = {
      FieldLine("TopTouchLine", kXMin-kLineThickness/2, kYMax, kXMax+kLineThickness/2, kYMax, kLineThickness),
      FieldLine("BottomTouchLine", kXMin-kLineThickness/2, kYMin, kXMax+kLineThickness/2, kYMin, kLineThickness),
      FieldLine("LeftGoalLine", kXMin, kYMin, kXMin, kYMax, kLineThickness),
      FieldLine("RightGoalLine", kXMax, kYMin, kXMax, kYMax, kLineThickness),
      FieldLine("HalfwayLine", 0, kYMin, 0, kYMax, kLineThickness),
      FieldLine("LeftPenaltyStretch", kXMin+kDefenseRadius, -kDefenseStretch/2, kXMin+kDefenseRadius, kDefenseStretch/2, kLineThickness),
      FieldLine("RightPenaltyStretch", kXMax-kDefenseRadius, -kDefenseStretch/2, kXMax-kDefenseRadius, kDefenseStretch/2, kLineThickness),
    };

    const std::size_t kNumFieldArcs = 7;
    const FieldCircularArc kFieldArcs[kNumFieldArcs] = {
      FieldCircularArc("LeftFieldLeftPenaltyArc",   kXMin, kDefenseStretch/2,  kDefenseRadius-kLineThickness/2, 0,        M_PI/2,   kLineThickness),
      FieldCircularArc("LeftFieldRightPenaltyArc",  kXMin, -kDefenseStretch/2, kDefenseRadius-kLineThickness/2, 1.5*M_PI, 2*M_PI,   kLineThickness),
      FieldCircularArc("RightFieldLeftPenaltyArc",  kXMax, -kDefenseStretch/2, kDefenseRadius-kLineThickness/2, M_PI,     1.5*M_PI, kLineThickness),
      FieldCircularArc("RightFieldRightPenaltyArc", kXMax, kDefenseStretch/2,  kDefenseRadius-kLineThickness/2, M_PI/2,   M_PI,     kLineThickness),
      FieldCircularArc("CenterCircle",              0,     0,                  kCenterRadius-kLineThickness/2,  0,        2*M_PI,   kLineThickness),

      FieldCircularArc("RightPenaltyMark", kXMax-kDefenseRadius+kDefenseRadius/16, 0, kLineThickness/3, 0, 2*M_PI, kLineThickness),
      FieldCircularArc("LeftPenaltyMark", kXMin+kDefenseRadius-kDefenseRadius/16, 0, kLineThickness/3, 0, 2*M_PI, kLineThickness),
    };
}

namespace FieldConstantsVSS2008 {
    const double kFieldLength = 1500.0;
    const double kFieldWidth = 1300.0;
    const double kGoalWidth = 400.0;
    const double kGoalDepth = 100.0;
    const double kBoundaryWidth = 300.0;

    // WR
    const double kCenterRadius = 200.0;
    const double kDefenseRadius = 150.0;
    const double kDefenseStretch = 700.0;

    const double kLineThickness = 3.0;
    const double kXMax = kFieldLength/2;
    const double kXMin = -kXMax;
    const double kYMax = kFieldWidth/2;
    const double kYMin = -kYMax;

    const double kFieldCorner = kFieldLength*5/150;
    const double kMarkDistanceX = kFieldLength/4;
    const double kMarkDistanceY = 4*kFieldLength/15;
    const double kMarkLength = kFieldLength*5/150;
    const double kMarkCircleDistance = 2*kFieldLength/15;
    const double kMarkCircleRadius = kMarkLength/10;

    const std::size_t kNumFieldLines = 29;
    const FieldLine kFieldLines[kNumFieldLines] = {
      FieldLine("TopTouchLine", kXMin-kLineThickness/2, kYMax, kXMax+kLineThickness/2, kYMax, kLineThickness),
      FieldLine("BottomTouchLine", kXMin-kLineThickness/2, kYMin, kXMax+kLineThickness/2, kYMin, kLineThickness),
      FieldLine("LeftGoalLine", kXMin, kYMin, kXMin, kYMax, kLineThickness),
      FieldLine("RightGoalLine", kXMax, kYMin, kXMax, kYMax, kLineThickness),
      FieldLine("HalfwayLine", 0, kYMin, 0, kYMax, kLineThickness),

      FieldLine("LeftPenaltyStretch", kXMin+kDefenseRadius, -kDefenseStretch/2, kXMin+kDefenseRadius, kDefenseStretch/2, kLineThickness),
      FieldLine("RightPenaltyStretch", kXMax-kDefenseRadius, -kDefenseStretch/2, kXMax-kDefenseRadius, kDefenseStretch/2, kLineThickness),
      FieldLine("LeftFieldLeftDefenseLine", kXMin, kDefenseStretch/2, kXMin+kDefenseRadius+kLineThickness/2, kDefenseStretch/2, kLineThickness),
      FieldLine("LeftFieldRightDefenseLine", kXMin, -kDefenseStretch/2, kXMin+kDefenseRadius+kLineThickness/2, -kDefenseStretch/2, kLineThickness),
      FieldLine("RightFieldLeftDefenseLine", kXMax, kDefenseStretch/2, kXMax-kDefenseRadius-kLineThickness/2, kDefenseStretch/2, kLineThickness),
      FieldLine("RightFieldRightDefenseLine", kXMax, -kDefenseStretch/2, kXMax-kDefenseRadius-kLineThickness/2, -kDefenseStretch/2, kLineThickness),

      FieldLine("LeftGoalStretch", kXMin-kGoalDepth, -kGoalWidth/2, kXMin-kGoalDepth, kGoalWidth/2, kLineThickness),
      FieldLine("RightGoalStretch", kXMax+kGoalDepth, -kGoalWidth/2, kXMax+kGoalDepth, kGoalWidth/2, kLineThickness),
      FieldLine("LeftGoalLeftLine", kXMin, kGoalWidth/2, kXMin-kGoalDepth-kLineThickness/2, kGoalWidth/2, kLineThickness),
      FieldLine("LeftGoalRightLine", kXMin, -kGoalWidth/2, kXMin-kGoalDepth-kLineThickness/2, -kGoalWidth/2, kLineThickness),
      FieldLine("RightGoalLeftLine", kXMax, kGoalWidth/2, kXMax+kGoalDepth+kLineThickness/2, kGoalWidth/2, kLineThickness),
      FieldLine("RightGoalRightLine", kXMax, -kGoalWidth/2, kXMax+kGoalDepth+kLineThickness/2, -kGoalWidth/2, kLineThickness),

      FieldLine("FBRightTopH", kMarkDistanceX-kMarkLength/2, kMarkDistanceY, kMarkDistanceX+kMarkLength/2, kMarkDistanceY, kLineThickness),
      FieldLine("FBRightTopV", kMarkDistanceX, kMarkDistanceY-kMarkLength/2, kMarkDistanceX, kMarkDistanceY+kMarkLength/2, kLineThickness),

      FieldLine("FBRightBottomH", kMarkDistanceX-kMarkLength/2, -kMarkDistanceY, kMarkDistanceX+kMarkLength/2, -kMarkDistanceY, kLineThickness),
      FieldLine("FBRightBottomV", kMarkDistanceX, -kMarkDistanceY-kMarkLength/2, kMarkDistanceX, -kMarkDistanceY+kMarkLength/2, kLineThickness),

      FieldLine("FBRightTopH", -kMarkDistanceX-kMarkLength/2, kMarkDistanceY, -kMarkDistanceX+kMarkLength/2, kMarkDistanceY, kLineThickness),
      FieldLine("FBRightTopV", -kMarkDistanceX, kMarkDistanceY-kMarkLength/2, -kMarkDistanceX, kMarkDistanceY+kMarkLength/2, kLineThickness),

      FieldLine("FBLeftBottomH", -kMarkDistanceX-kMarkLength/2, -kMarkDistanceY, -kMarkDistanceX+kMarkLength/2, -kMarkDistanceY, kLineThickness),
      FieldLine("FBLeftBottomV", -kMarkDistanceX, -kMarkDistanceY-kMarkLength/2, -kMarkDistanceX, -kMarkDistanceY+kMarkLength/2, kLineThickness),

      FieldLine("PKRightH", kMarkDistanceX-kMarkLength/2, 0, kMarkDistanceX+kMarkLength/2, 0, kLineThickness),
      FieldLine("PKRightV", kMarkDistanceX, -kMarkLength/2, kMarkDistanceX, kMarkLength/2, kLineThickness),

      FieldLine("PKLeftH", -kMarkDistanceX-kMarkLength/2, 0, -kMarkDistanceX+kMarkLength/2, 0, kLineThickness),
      FieldLine("PKLeftV", -kMarkDistanceX, -kMarkLength/2, -kMarkDistanceX, kMarkLength/2, kLineThickness),
    };

    const std::size_t kNumFieldArcs = 11;
    const FieldCircularArc kFieldArcs[kNumFieldArcs] = {
      FieldCircularArc("CenterCircle", 0, 0, kCenterRadius-kLineThickness/2, 0, 2*M_PI, kLineThickness),

      FieldCircularArc("FBRightTopLeft", kMarkDistanceX-kMarkCircleDistance, kMarkDistanceY, kMarkCircleRadius, 0, 2*M_PI, 2*kMarkCircleRadius),
      FieldCircularArc("FBRightTopRight", kMarkDistanceX+kMarkCircleDistance, kMarkDistanceY, kMarkCircleRadius, 0, 2*M_PI, 2*kMarkCircleRadius),

      FieldCircularArc("FBRightBottomLeft", kMarkDistanceX-kMarkCircleDistance, -kMarkDistanceY, kMarkCircleRadius, 0, 2*M_PI, 2*kMarkCircleRadius),
      FieldCircularArc("FBRightBottomRight", kMarkDistanceX+kMarkCircleDistance, -kMarkDistanceY, kMarkCircleRadius, 0, 2*M_PI, 2*kMarkCircleRadius),

      FieldCircularArc("FBLeftTopLeft", -kMarkDistanceX-kMarkCircleDistance, kMarkDistanceY, kMarkCircleRadius, 0, 2*M_PI, 2*kMarkCircleRadius),
      FieldCircularArc("FBLeftTopRight", -kMarkDistanceX+kMarkCircleDistance, kMarkDistanceY, kMarkCircleRadius, 0, 2*M_PI, 2*kMarkCircleRadius),

      FieldCircularArc("FBLeftBottomLeft", -kMarkDistanceX-kMarkCircleDistance, -kMarkDistanceY, kMarkCircleRadius, 0, 2*M_PI, 2*kMarkCircleRadius),
      FieldCircularArc("FBLeftBottomRight", -kMarkDistanceX+kMarkCircleDistance, -kMarkDistanceY, kMarkCircleRadius, 0, 2*M_PI, 2*kMarkCircleRadius),

      FieldCircularArc("GoalRightArc", kXMax-kDefenseRadius/2, 0, kDefenseRadius/2+50, M_PI-0.9272, M_PI+0.9272, kLineThickness),
      FieldCircularArc("GoalLeftArc", kXMin+kDefenseRadius/2, 0, kDefenseRadius/2+50, -0.9272, +0.9272, kLineThickness),
    };

    const std::size_t kNumFieldTriangles = 4;
    const FieldTriangle kFieldTriangles[kNumFieldTriangles] = {
      FieldTriangle("RightTop", kXMax-kFieldCorner, kYMax, kXMax, kYMax-kFieldCorner, kXMax, kYMax, kLineThickness),
      FieldTriangle("RightBottom", kXMax-kFieldCorner, -kYMax, kXMax, -kYMax+kFieldCorner, kXMax, -kYMax, kLineThickness),
      FieldTriangle("LeftTop", -kXMax+kFieldCorner, kYMax, -kXMax, kYMax-kFieldCorner, -kXMax, kYMax, kLineThickness),
      FieldTriangle("LeftBottom", -kXMax+kFieldCorner, -kYMax, -kXMax, -kYMax+kFieldCorner, -kXMax, -kYMax, kLineThickness),
    };

}

#endif // FIELD_DEFAULT_CONSTANTS_H
