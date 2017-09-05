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

/*! \file
  \brief Defines the interface for display engines used by GraspIt!
 */
#ifndef DISPLAY_INTERFACE_HXX
#define DISPLAY_INTERFACE_HXX

class World;
class Body;
class Joint;
class Robot;
class VirtualContact;
class Contact;
class transf;

class DisplayInterface {
  protected:

  public:

    //! Initialized with a pointer to the world that will
    //! be displayed
    DisplayInterface(World *w)=0;

    virtual ~DisplayInterface() =0;

    //! Add a body to be displayed
    virtual void addBody(Body *body) = 0;
    virtual void removeBody(Body *body) = 0;

    //! Add a joint to be displayed
    virtual void addJoint(Joint *joint) = 0;
    virtual void removeJoint(Joint *joint) = 0;

    //! Add a robot to be displayed
    virtual void addRobot(Robot *robot) = 0;
    virtual void removeRobot(Robot *robot) = 0;

    //! Add a virtual contact to be displayed
    virtual void addVirtualContact(VirtualContact *vc) = 0;
    virtual void removeVirtualContact(VirtualContact *vc) = 0;

    //! Add a contact to be displayed
    virtual void addContact(Contact *c) = 0;
    virtual void removeContact(Contact *c) = 0;

    virtual void setCamera(double px, double py, double pz, double q1, double q2, double q3, double q4, double fd) = 0;
    virtual void getCamera(float &px, float &py, float &pz, float &q1, float &q2, float &q3, float &q4, float &fd) = 0;

    virtual void setCameraTransf(transf tr) = 0;
    virtual transf getCameraTransf() = 0;

    //! Each display object will update its current
    //! pose and how it is displayed as determined by its
    //! associated object.
    virtual void update() = 0;
};

#endif
