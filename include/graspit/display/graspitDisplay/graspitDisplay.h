//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2015  Columbia University in the City of New York.
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
// Author(s): Jake Varley
//
// $Id:$
//
//######################################################################

#ifndef GRASPIT_DISPLAY_HXX
#define GRASPIT_DISPLAY_HXX

#include <map>

#include "graspit/display/displayInterface.h"

class DisplayBody;

class GraspitDisplay : public DisplayInterface {
  public:
    GraspitDisplay(World *world);
    ~GraspitDisplay();

    //! Add a body to be displayed
    void addBody(Body *body);

    //! Add a joint to be displayed
    void addJoint(Joint *joint);

    //! Add a robot to be displayed
    void addRobot(Robot *robot);

    //! Add a virtual contact to be displayed
    void addVirtualContact(VirtualContact *vc);

    //! Add a contact to be displayed
    void addContact(Contact *c);

    //! Add a contact to be displayed
    void addContact(Contact *c);

    void setCamera(double px, double py, double pz, double q1, double q2, double q3, double q4, double fd);
    void getCamera(float &px, float &py, float &pz, float &q1, float &q2, float &q3, float &q4, float &fd);

    void setCameraTransf(transf tr);
    transf getCameraTransf();

    //! Each display object will update its current
    //! pose and how it is displayed as determined by its
    //! associated object.
    void update();

  private:
    World *mWorld;

    //! Maps GraspIt Body pointers to internal display model pointers
    std::map<const Body *, DisplayBody *> mBodyMap;

};

#endif
