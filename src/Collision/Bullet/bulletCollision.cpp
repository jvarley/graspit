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
// Author(s): Matei T. Ciocarlie
//
// $Id: graspitCollision.cpp,v 1.10 2009/09/11 19:06:32 jweisz Exp $
//
//######################################################################

#include <algorithm>

#include "Collision/Bullet/bulletCollision.h"

//#define GRASPITDBG
#include "debug.h"

#include "body.h"
#include "collisionModel.h"
#include "collisionAlgorithms.h"
#include "bulletEngine.h"


//#include <Bullet3Collision/NarrowPhaseCollision/shared/b3ContactSphereSphere.h>

#include <btBulletDynamicsCommon.h>
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btPointCollector.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"

#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"

//#define PROF_ENABLED
#include "profiling.h"

static btVoronoiSimplexSolver sGjkSimplexSolver;

void
BulletCollision::activateBody(const Body *body, bool active)
{

}

void
BulletCollision::activatePair(const Body *body1, const Body *body2, bool active)
{
}

bool
BulletCollision::isActive(const Body *body1, const Body *body2)
{
}

bool
BulletCollision::addBody(Body *body,  bool)
{
  mBulletEngine->addBody(body);
}

bool BulletCollision::updateBodyGeometry(Body *body, bool)
{

}

void
BulletCollision::removeBody(Body *body)
{
  return;
}

void
BulletCollision::cloneBody(Body *clone, const Body *original)
{

}

void
BulletCollision::setBodyTransform(Body *body, const transf &t)
{
  btRigidBody *btbody =  mBulletEngine->btBodyMap.find(const_cast<Body*>(body))->second;

    btTransform feedbacktransform = btbody->getCenterOfMassTransform();
    btQuaternion btrotation = feedbacktransform.getRotation();
    btVector3 bttranslation = feedbacktransform.getOrigin();

    Quaternion *rot = new Quaternion(btrotation.getAngle() ,
                                     vec3(btrotation.getAxis()[0] ,
                                          btrotation.getAxis()[1] ,
                                          btrotation.getAxis()[2]));
    vec3 *transl = new vec3(bttranslation.getX() , bttranslation.getY() , bttranslation.getZ());
    Quaternion rotfix = *rot;
    vec3 translfix = *transl;
    transf *temptrans = new transf(rotfix , translfix) ;

    std::cout << "Old bullet pose" << *temptrans << std::endl;

  const btQuaternion q(t.rotation().x,t.rotation().y,t.rotation().z,t.rotation().w) ;
  const btVector3& c = btVector3(t.translation().x(),t.translation().y(),t.translation().z());
  btTransform btTrans(q,c);
  btbody->setWorldTransform(btTrans);

    btTransform feedbacktransform2 = btbody->getCenterOfMassTransform();
    btQuaternion btrotation2 = feedbacktransform2.getRotation();
    btVector3 bttranslation2 = feedbacktransform2.getOrigin();

    Quaternion *rot2 = new Quaternion(btrotation2.getAngle() ,
                                     vec3(btrotation2.getAxis()[0] ,
                                          btrotation2.getAxis()[1] ,
                                          btrotation2.getAxis()[2]));
    vec3 *transl2 = new vec3(bttranslation2.getX() , bttranslation2.getY() , bttranslation2.getZ());
    Quaternion rotfix2= *rot2;
    vec3 translfix2 = *transl2;
    transf *temptrans2 = new transf(rotfix2 , translfix2) ;

   std::cout << "desired Graspit pose" << t << std::endl;
    std::cout << "New bullet pose" << *temptrans2 << std::endl;
}


int
BulletCollision::allCollisions(DetectionType type, CollisionReport *report,
                               const std::vector<Body *> *interestList)
{
//  double timeStep = 1.0f / 60.f;
//  if(!mWorld->dynamicsAreOn()){
//      mBulletEngine->mBtDynamicsWorld->stepSimulation(timeStep, 10);
//    }

  int manifold_count = 0;
  //Todo we currently ignore interest list
  int numManifolds = mBulletEngine->mBtDynamicsWorld->getDispatcher()->getNumManifolds();
  std::cout << "numManifolds: " << numManifolds << std::endl;
  for (int i=0;i<numManifolds;i++)
  {
      btPersistentManifold* contactManifold =  mBulletEngine->mBtDynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);

      btCollisionObject* obA = const_cast<btCollisionObject*>(contactManifold->getBody0());
      btCollisionObject* obB = const_cast<btCollisionObject*>(contactManifold->getBody1());
      Body *b1 = mBulletEngine->bodyMap.find(static_cast<btRigidBody*>(obA))->second;
      Body *b2 = mBulletEngine->bodyMap.find(static_cast<btRigidBody*>(obB))->second;

      if(interestList)
        {
          if((std::find(interestList->begin(), interestList->end(), b1) != interestList->end() &&
             std::find(interestList->begin(), interestList->end(), b2) == interestList->end()) ||
             (std::find(interestList->begin(), interestList->end(), b1) == interestList->end() &&
                          std::find(interestList->begin(), interestList->end(), b2) != interestList->end())) {
              report->push_back(CollisionData(b1, b2));
              manifold_count ++;

              std::cout << "We have found a contact between: " << b1->name() << " " << b2->name() << std::endl;
              int numContacts = contactManifold->getNumContacts();
              std::cout << "numContacts: " << numContacts << std::endl;
              for (int j=0;j<numContacts;j++)
                {
                  btManifoldPoint& pt = contactManifold->getContactPoint(j);
                  std::cout << "Contact Dist: " << pt.getDistance() << std::endl;
                }

          } else {
             std::cout << "Collision Not in interest list" << std::endl;
          }
        }
      else{
          manifold_count = numManifolds;
        }

   }

  return manifold_count;
}

int
BulletCollision::allContacts(CollisionReport *report, double threshold,
                             const std::vector<Body *> *interestList)
{
  if(!mWorld->dynamicsAreOn()){
      double timeStep = 1.0f / 60.f;
      mBulletEngine->mBtDynamicsWorld->stepSimulation(timeStep, 10);
    }

  int numManifolds = mBulletEngine->mBtDynamicsWorld->getDispatcher()->getNumManifolds();
  for (int i=0;i<numManifolds;i++)
    {
      btPersistentManifold* contactManifold =  mBulletEngine->mBtDynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
      btCollisionObject* obA = const_cast<btCollisionObject*>(contactManifold->getBody0());
      btCollisionObject* obB = const_cast<btCollisionObject*>(contactManifold->getBody1());
      Body *b1 = mBulletEngine->bodyMap.find(static_cast<btRigidBody*>(obA))->second;
      Body *b2 = mBulletEngine->bodyMap.find(static_cast<btRigidBody*>(obB))->second;
      report->push_back(CollisionData(b1, b2));

      int numContacts = contactManifold->getNumContacts();
      for (int j=0;j<numContacts;j++)
        {
          btManifoldPoint& pt = contactManifold->getContactPoint(j);
          if (pt.getDistance()<0.f)
            {
              ContactData cd;
              const btVector3& ptA = pt.getPositionWorldOnA();
              const btVector3& ptB = pt.getPositionWorldOnB();
              const btVector3& normalOnB = pt.m_normalWorldOnB;

              cd.b1_pos = position(ptA.x(),ptA.y(), ptA.z());
              cd.b2_pos = position(ptB.x(),ptB.y(), ptB.z());
              cd.b1_normal = vec3(-pt.m_normalWorldOnB.x(), -pt.m_normalWorldOnB.y(), -pt.m_normalWorldOnB.z());
              cd.b2_normal = vec3(normalOnB.x(), normalOnB.y(), normalOnB.z());
              cd.distSq = pt.getDistance()*pt.getDistance();

              report->back().contacts.push_back(cd);
            }
        }
    }

  return numManifolds;
}

int
BulletCollision::contact(ContactReport *report, double threshold,
                         const Body *body1, const Body *body2)
{
  int contacts = 0;
  return contacts;
}

double
BulletCollision::pointToBodyDistance(const Body *body1, position point,
                                     position &closestPoint, vec3 &closestNormal)
{
  btRigidBody *btbodyB =  mBulletEngine->btBodyMap.find(const_cast<Body*>(body1))->second;
  btConvexHullShape *bthullB =  mBulletEngine->btConvexHullMap.find(const_cast<Body*>(body1))->second;

  btVector3 v0(point.x(), point.y(), point.z());
  btTransform trans;
  trans.setIdentity();
  btConvexHullShape *bthullA = new btConvexHullShape(&(v0.getX()), 1, sizeof(btVector3));

  btGjkEpaPenetrationDepthSolver epa;
  btGjkPairDetector convexConvex(bthullA,bthullB, &sGjkSimplexSolver, &epa);

  btPointCollector gjkOutput;
  btGjkPairDetector::ClosestPointInput input;

  //Todo: Not sure this is the correct Transform
  //input.m_transformB = btbodyB->getCenterOfMassTransform();
  input.m_transformA = trans;
  input.m_transformB = btbodyB->getWorldTransform();

  convexConvex.getClosestPoints(input ,gjkOutput,0);

  btVector3 endPt = gjkOutput.m_pointInWorld +
      gjkOutput.m_normalOnBInWorld*gjkOutput.m_distance;

  closestPoint.set(endPt.x(), endPt.y(), endPt.z());
  closestNormal.set(gjkOutput.m_normalOnBInWorld.x(),
                    gjkOutput.m_normalOnBInWorld.y(),
                    gjkOutput.m_normalOnBInWorld.z());

  return gjkOutput.m_distance;
}

double
BulletCollision::bodyToBodyDistance(const Body *body1, const Body *body2,
                                    position &p1, position &p2)
{

  //  //Todo Better way than const_cast?
  btRigidBody *btbody1 =  mBulletEngine->btBodyMap.find(const_cast<Body*>(body1))->second;
  btRigidBody *btbody2 =  mBulletEngine->btBodyMap.find(const_cast<Body*>(body2))->second;

  btConvexHullShape *bthull1 =  mBulletEngine->btConvexHullMap.find(const_cast<Body*>(body1))->second;
  btConvexHullShape *bthull2 =  mBulletEngine->btConvexHullMap.find(const_cast<Body*>(body2))->second;

  btGjkEpaPenetrationDepthSolver epa;
  btGjkPairDetector convexConvex(bthull1, bthull2, &sGjkSimplexSolver, &epa);

  btPointCollector gjkOutput;
  btGjkPairDetector::ClosestPointInput input;

  //Todo: Not sure this is the correct Transform
  input.m_transformA = btbody1->getCenterOfMassTransform();
  input.m_transformB = btbody2->getCenterOfMassTransform();

  convexConvex.getClosestPoints(input ,gjkOutput,0);

  btVector3 endPt = gjkOutput.m_pointInWorld +
      gjkOutput.m_normalOnBInWorld*gjkOutput.m_distance;

  p1.set(gjkOutput.m_pointInWorld.x(),gjkOutput.m_pointInWorld.y(),gjkOutput.m_pointInWorld.z());
  p2.set(endPt.x(),endPt.y(),endPt.z());

  return gjkOutput.m_distance;
}

void
BulletCollision::bodyRegion(const Body *body, position point, vec3 normal,
                            double radius, Neighborhood *neighborhood)
{

}


void
BulletCollision::getBoundingVolumes(const Body *body, int depth, std::vector<BoundingBox> *bvs)
{

}
