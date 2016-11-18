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


#include "bulletEngine.h"

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <BulletDynamics/ConstraintSolver/btHingeConstraint.h>
#include <BulletDynamics/ConstraintSolver/btGearConstraint.h>
#include "LinearMath/btConvexHullComputer.h"

#include "body.h"
#include "triangle.h"

BulletEngine::BulletEngine()
{

  // collision configuration contains default setup for memory, collision setup.
  btDefaultCollisionConfiguration *collisionConfiguration;
  collisionConfiguration = new btDefaultCollisionConfiguration();

  // use the default collision dispatcher.
  btCollisionDispatcher *dispatcher = new btCollisionDispatcher(collisionConfiguration);
  btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);

  // btDbvtBroadphase is a good general purpose broadphase. .
  btBroadphaseInterface *overlappingPairCache = new btDbvtBroadphase();

  //the default constraint solver.
  btSequentialImpulseConstraintSolver *solver = new btSequentialImpulseConstraintSolver;
  mBtDynamicsWorld =
    new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);

  mBtDynamicsWorld->setGravity(btVector3(0, 0, 0));
}

void BulletEngine::addBody(Body *newBody)
{
  std::map<Body *, btRigidBody *>::iterator it  = btBodyMap.find(newBody);
  std::cout << btBodyMap.count(newBody) << std::endl;
  if (btBodyMap.count(newBody) == 1){
    std::cout << "Body already exists, not adding it: " << newBody->name() << std::endl;
    return;
  }
  std::cout << "added new body: " << newBody->name() << std::endl;

  // Creation of CollisionShape
  btTriangleMesh *triMesh = new btTriangleMesh(true, true); //true,true);

  // Get the geometry data form the Graspit object
  std::vector<Triangle> triangles;

  newBody->getGeometryTriangles(&triangles);
  int numTriangles = triangles.size();
  Triangle tritemp = triangles.at(0);

  std::vector<btVector3> points;

  for (int i = 0; i < numTriangles - 1; i = i + 1)
  {
    tritemp = triangles.at(i);
    btScalar v01(tritemp.v1[0]);
    btScalar v02(tritemp.v1[1]);
    btScalar v03(tritemp.v1[2]);
    btScalar v11(tritemp.v2[0]);
    btScalar v12(tritemp.v2[1]);
    btScalar v13(tritemp.v2[2]);
    btScalar v21(tritemp.v3[0]);
    btScalar v22(tritemp.v3[1]);
    btScalar v23(tritemp.v3[2]);

    btVector3 v0(v01, v02, v03);
    btVector3 v1(v11, v12, v13);
    btVector3 v2(v21, v22, v23);

    triMesh->btTriangleMesh::addTriangle(v0, v1, v2, true);
    points.push_back(v0);
    points.push_back(v1);
    points.push_back(v2);
  }

  btConvexHullComputer convexUtil;
  convexUtil.compute(&points[0].getX(), sizeof(btVector3), points.size(), 0,0);
  btConvexHullShape *hull = new btConvexHullShape(&(convexUtil.vertices[0].getX()), convexUtil.vertices.size(), sizeof(btVector3));

  btCollisionShape *triMeshShape;
  btScalar mass(0.);
  btVector3 localInertia(0, 0, 0);

  if (newBody->isDynamic())
  {
    mass = ((DynamicBody *)newBody)->getMass(); //mass g
    mass = mass / 1000; //kg
    triMeshShape = new btGImpactMeshShape(triMesh);
    ((btGImpactMeshShape *)triMeshShape)->updateBound();
    triMeshShape->calculateLocalInertia(mass, localInertia);
  }
  else
  {
    triMeshShape = new btBvhTriangleMeshShape(triMesh, true, true);
  }

  //using motionstate is recommended, it provides interpolation capabilities,
  //and only synchronizes 'active' objects
  btDefaultMotionState *myMotionState =
    new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0 , 0)));
  btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, triMeshShape, localInertia);
  btRigidBody *body = new btRigidBody(rbInfo);

  body->setFriction(1.0);
  body->setRollingFriction(1.0);

  // avoid deactive
  body->setSleepingThresholds(0, 0);

  //add the body to the dynamics world
  mBtDynamicsWorld->addRigidBody(body);

  mBtLinks.push_back(body);
  //add the newlink and body to the map
  btBodyMap.insert(bodybtRigidBodyPair(newBody, body));
  bodyMap.insert(btRigidBodyBodyPair(body, newBody));
  btConvexHullMap.insert(btBody2ConvexPair(newBody, hull));

}

BulletEngine::~BulletEngine()
{
  delete mBtDynamicsWorld;
}
