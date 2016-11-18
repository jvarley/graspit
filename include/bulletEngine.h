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

#ifndef BULLETENGINE_HXX
#define BULLETENGINE_HXX

#include <LinearMath/btAlignedObjectArray.h>
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"

#include "dynamicsEngine.h"
#include "joint.h"

class btDiscreteDynamicsWorld;
class btRigidBody;
class btHingeConstraint;
class BulletEngine;
class Body;

typedef std::pair<Body *, btRigidBody *> bodybtRigidBodyPair;
typedef std::pair<btRigidBody*, Body *> btRigidBodyBodyPair;
typedef std::pair<Body *, btConvexHullShape *> btBody2ConvexPair;
typedef std::pair<Body *, btHingeConstraint *> btJointPair;

class BulletEngine {
  public:
    explicit BulletEngine();
    ~BulletEngine();

    void addBody(Body *newBody);

    btDiscreteDynamicsWorld *mBtDynamicsWorld;
    btAlignedObjectArray<btRigidBody *> mBtLinks;

    std::map<Body *, btRigidBody *> btBodyMap;
    std::map<btRigidBody *, Body *> bodyMap;
    std::map<Body *, btConvexHullShape *> btConvexHullMap;

    std::map<Joint *, btHingeConstraint *> btJointMap;
};

#endif
