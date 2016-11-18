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
// Author(s): Jake Varley, Iretiayo Akinola
//
//
//######################################################################

#ifndef _bulletcollision_h_
#define _bulletcollision_h_

/*! \file
  Implementation of the collision detection interface. This collision
  engine was written from scratch as part of GraspIt.
*/

#include <map>
#include <set>
#include <list>
#include <vector>

#include "collisionInterface.h"

class BulletEngine;
class btRigidBody;
class World;

class BulletCollision : public CollisionInterface
{
  private:

    BulletEngine *mBulletEngine;
    World *mWorld;

  public:
    BulletCollision(World *w, BulletEngine *be) {mWorld=w; mBulletEngine=be;}
    virtual ~BulletCollision() {}

    //adding and moving bodies
    virtual bool addBody(Body *body, bool ExpectEmpty = false);
    virtual bool updateBodyGeometry(Body *body, bool ExpectEmpty = false);
    virtual void removeBody(Body *body);
    virtual void cloneBody(Body *, const Body *);
    virtual void setBodyTransform(Body *body, const transf &t);

    //enable / disable collision
    virtual void activateBody(const Body *, bool);
    virtual void activatePair(const Body *, const Body *, bool);
    virtual bool isActive(const Body *, const Body *);

    //collision detection
    virtual int allCollisions(DetectionType type, CollisionReport *report,
                              const std::vector<Body *> *interestList);

    //contact
    virtual int allContacts(CollisionReport *report, double threshold,
                            const std::vector<Body *> *interestList);
    virtual int contact(ContactReport *report, double threshold,
                        const Body *body1, const Body *body2);

    //point-to-body and body-to-body distances
    virtual double pointToBodyDistance(const Body *body1, position point,
                                       position &closestPoint, vec3 &closestNormal);
    virtual double bodyToBodyDistance(const Body *body1, const Body *body2,
                                      position &p1, position &p2);

    //region on a body around a point
    virtual void bodyRegion(const Body *body, position point, vec3 normal,
                            double radius, Neighborhood *neighborhood);

    //show bounding box hierarchy
    virtual void getBoundingVolumes(const Body *, int, std::vector<BoundingBox> *);

    //threading
    virtual void newThread() {}
    virtual int getThread() {return 0;}
};

#endif
