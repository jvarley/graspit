//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s):  Jake Varley
//
// $Id: displayBody.h
//
//######################################################################

#ifndef DISPLAY_BODY_HXX

#include <QTextStream>
#include <QString>
#include <list>
#include <vector>

class SoCoordinate3;
class SoGroup;
class SoIndexedFaceSet;
class SoMaterial;
class SoScale;
class SoSeparator;
class SoSwitch;
class SoTranslation;
class SoTransform;

class Body;

class DisplayBody {

  public:
    //! Parameter to control the height of friction cones
    static const float CONE_HEIGHT;

  protected:

    //! Body whose geometry we are displaying
    Body *mBody;

    //! A pointer to the root node of the geometry of this model
    SoSeparator *IVGeomRoot;

    //! A pointer to a node that scales the geometry of this model
    SoTransform *IVScaleTran;

    //! A pointer to a node that offsets the geometry of this model
    SoTransform *IVOffsetTran;

    //! A pointer to a node that can hold the geometry of the bounding volume struture
    SoSeparator *IVBVRoot;

    //! A pointer to the Inventor transform for the body
    SoTransform *IVTran;

    //! A pointer to the material node that controls this body's transparency
    SoMaterial *IVMat;

    //! A pointer to the root of the friction cones on this body
    SoSeparator *IVContactIndicators;

    //! Inventor root of the axes in the body subtree
    SoSwitch *IVAxes;

    //! Inventor root of the worst case disturbance wrench indicator
    SoSeparator *IVWorstCase;

    //! Inventor transform from body frame to center of gravity
    SoTranslation *axesTranToCOG;

    //! Inventor scale for axes so that they extend outside the body
    SoScale *axesScale;

    /////////////////////////////// PUBLIC /////////////////////////////////
  public:
    //! constructor for display body. Takes in body that it will be representing.
    DisplayBody(Body *b);

    //!
    virtual ~DisplayBody();

};



#define DISPLAY_BODY_HXX
#endif
