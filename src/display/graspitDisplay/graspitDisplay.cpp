#include <iostream>

#include "graspit/display/graspitDisplay/graspitDisplay.h"
#include "graspit/display/graspitDisplay/displayBody.h"

#include "graspit/body.h"
#include "graspit/robot.h"
#include "graspit/contact/contact.h"
#include "graspit/contact/virtualContact.h"
#include "graspit/transform.h"


GraspitDisplay::GraspitDisplay(World *world):mWorld(world)
{

}


GraspitDisplay::~GraspitDisplay()
{

}

void GraspitDisplay::addBody(Body *body)
{
    DisplayBody *db = new DisplayBody(body);
    mBodyMap[body] = db;
}

void GraspitDisplay::removeBody(Body *body)
{
    DisplayBody *db  = getModel(body);

    // If the body was not registered with the display interface,
    // there is nothing left to do.
    if (!db) {
      DBGA("GraspitDisplay::removeBody Display Body not found");
      return;
    }

    // remove the entry from the body map
    mBodyMap.erase(mBodyMap.find(db));

    // delete the associated display body.
    delete db;
}

void GraspitDisplay::addJoint(Joint *joint)
{
    std::cout << "GraspitDisplay::addJoint not Implemented Yet!" << std::endl;
}
void GraspitDisplay::removeJoint(Joint *joint)
{
    std::cout << "GraspitDisplay::removeJoint not Implemented Yet!" << std::endl;
}

void GraspitDisplay::addRobot(Robot *robot)
{
    std::cout << "GraspitDisplay::addRobot not Implemented Yet!" << std::endl;
}
void GraspitDisplay::removeRobot(Robot *robot)
{
    std::cout << "GraspitDisplay::removeRobot not Implemented Yet!" << std::endl;
}

void GraspitDisplay::addVirtualContact(VirtualContact *vc)
{
    std::cout << "GraspitDisplay::addVirtualContact not Implemented Yet!" << std::endl;
}
void GraspitDisplay::removeVirtualContact(VirtualContact *vc)
{
    std::cout << "GraspitDisplay::removeVirtualContact not Implemented Yet!" << std::endl;
}

void GraspitDisplay::addContact(Contact *c)
{
    std::cout << "GraspitDisplay::addContact not Implemented Yet!" << std::endl;
}
void GraspitDisplay::removeContact(Contact *c)
{
    std::cout << "GraspitDisplay::removeContact not Implemented Yet!" << std::endl;
}

void GraspitDisplay::setCamera(double px, double py, double pz, double q1, double q2, double q3, double q4, double fd)
{
    std::cout << "GraspitDisplay::setCamera not Implemented Yet!" << std::endl;
}

void GraspitDisplay::getCamera(float &px, float &py, float &pz, float &q1, float &q2, float &q3, float &q4, float &fd)
{
    std::cout << "GraspitDisplay::getCamera not Implemented Yet!" << std::endl;
}

void GraspitDisplay::setCameraTransf(transf tr){
    std::cout << "GraspitDisplay::setCameraTransf not Implemented Yet!" << std::endl;
}

transf GraspitDisplay::getCameraTransf()
{
    std::cout << "GraspitDisplay::getCameraTransf not Implemented Yet!" << std::endl;
    return transf::IDENTITY();
}
