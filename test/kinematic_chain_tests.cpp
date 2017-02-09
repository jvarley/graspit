#include <gtest/gtest.h>
#include <iostream>
#include <QString>
#include <pthread.h>
#include <QMutex>
#include <Inventor/Qt/SoQt.h>

#include "graspitCore.h"
#include <graspitApp.h>
#include "mainWindow.h"
#include "ivmgr.h"
#include "world.h"
#include "plannerdlg.h"
#include "quality.h"
#include "robot.h"
#include "qmDlg.h"
#include "grasp.h"
#include "collisionInterface.h"
#include "collisionStructures.h"
#include "collisionModel.h"
#include "Graspit/collisionAlgorithms.h"
#include "bBox.h"
#include "matrix.h"

#define SLOP 0.001

TEST(TEST_KINEMATIC_CHAIN, TEST_LINK_JACOBIAN) {
    int argc = 0;
    GraspItApp app(argc, NULL);
    GraspitCore core(argc, NULL);

    app.setMainWidget(core.getMainWindow()->mWindow);
    QString fileName = QString(QString(getenv("GRASPIT"))+QString("/worlds/") + QString("plannerMug.xml"));

    graspitCore->getMainWindow()->mUI->worldBox->setTitle(fileName);
    graspitCore->getWorld()->load(fileName);
    graspitCore->getMainWindow()->setMainWorld(graspitCore->getWorld());

    vec3 x = vec3(1,0,0);
    transf rot = transf::AXIS_ANGLE_ROTATION(10, x);
    transf translate = transf::TRANSLATION(vec3(1,2,3));

    transf newRobotTran = rot%translate;
    graspitCore->getWorld()->getRobot(0)->setTran(newRobotTran);

    KinematicChain *c = graspitCore->getWorld()->getRobot(0)->getChain(0);
    Matrix J = c->linkJacobian(false);

    int numLinks = c->getNumLinks();
    //jacobian size is: numLinks * 6, numLinks * 6

//    for (int i =0; i < numLinks*6; i++)
//    {
//        for(int j=0; j< numLinks*6; j++)
//        {
//            std::cout << " ASSERT_NEAR(J.elem(" <<i<< ","<< j<<"), " << J.elem(i,j) << ", SLOP);";
//        }
//        std::cout << std::endl;
//    }

    ASSERT_NEAR(J.elem(0,0), 1.11022e-16, SLOP); ASSERT_NEAR(J.elem(0,1), 1, SLOP); ASSERT_NEAR(J.elem(0,2), 1.66533e-16, SLOP); ASSERT_NEAR(J.elem(0,3), -2.33147e-15, SLOP); ASSERT_NEAR(J.elem(0,4), -2.36658e-30, SLOP); ASSERT_NEAR(J.elem(0,5), 1.57652e-14, SLOP); ASSERT_NEAR(J.elem(0,6), 0, SLOP); ASSERT_NEAR(J.elem(0,7), 0, SLOP); ASSERT_NEAR(J.elem(0,8), 0, SLOP); ASSERT_NEAR(J.elem(0,9), 0, SLOP); ASSERT_NEAR(J.elem(0,10), 0, SLOP); ASSERT_NEAR(J.elem(0,11), 0, SLOP); ASSERT_NEAR(J.elem(0,12), 0, SLOP); ASSERT_NEAR(J.elem(0,13), 0, SLOP); ASSERT_NEAR(J.elem(0,14), 0, SLOP); ASSERT_NEAR(J.elem(0,15), 0, SLOP); ASSERT_NEAR(J.elem(0,16), 0, SLOP); ASSERT_NEAR(J.elem(0,17), 0, SLOP);
    ASSERT_NEAR(J.elem(1,0), 0, SLOP); ASSERT_NEAR(J.elem(1,1), 1.11022e-16, SLOP); ASSERT_NEAR(J.elem(1,2), -1, SLOP); ASSERT_NEAR(J.elem(1,3), -50, SLOP); ASSERT_NEAR(J.elem(1,4), 2.13163e-14, SLOP); ASSERT_NEAR(J.elem(1,5), 2.36658e-30, SLOP); ASSERT_NEAR(J.elem(1,6), 0, SLOP); ASSERT_NEAR(J.elem(1,7), 0, SLOP); ASSERT_NEAR(J.elem(1,8), 0, SLOP); ASSERT_NEAR(J.elem(1,9), 0, SLOP); ASSERT_NEAR(J.elem(1,10), 0, SLOP); ASSERT_NEAR(J.elem(1,11), 0, SLOP); ASSERT_NEAR(J.elem(1,12), 0, SLOP); ASSERT_NEAR(J.elem(1,13), 0, SLOP); ASSERT_NEAR(J.elem(1,14), 0, SLOP); ASSERT_NEAR(J.elem(1,15), 0, SLOP); ASSERT_NEAR(J.elem(1,16), 0, SLOP); ASSERT_NEAR(J.elem(1,17), 0, SLOP);
    ASSERT_NEAR(J.elem(2,0), -1, SLOP); ASSERT_NEAR(J.elem(2,1), 1.66533e-16, SLOP); ASSERT_NEAR(J.elem(2,2), 0, SLOP); ASSERT_NEAR(J.elem(2,3), -1.77494e-30, SLOP); ASSERT_NEAR(J.elem(2,4), -1.06581e-14, SLOP); ASSERT_NEAR(J.elem(2,5), 50, SLOP); ASSERT_NEAR(J.elem(2,6), 0, SLOP); ASSERT_NEAR(J.elem(2,7), 0, SLOP); ASSERT_NEAR(J.elem(2,8), 0, SLOP); ASSERT_NEAR(J.elem(2,9), 0, SLOP); ASSERT_NEAR(J.elem(2,10), 0, SLOP); ASSERT_NEAR(J.elem(2,11), 0, SLOP); ASSERT_NEAR(J.elem(2,12), 0, SLOP); ASSERT_NEAR(J.elem(2,13), 0, SLOP); ASSERT_NEAR(J.elem(2,14), 0, SLOP); ASSERT_NEAR(J.elem(2,15), 0, SLOP); ASSERT_NEAR(J.elem(2,16), 0, SLOP); ASSERT_NEAR(J.elem(2,17), 0, SLOP);
    ASSERT_NEAR(J.elem(3,0), 0, SLOP); ASSERT_NEAR(J.elem(3,1), 0, SLOP); ASSERT_NEAR(J.elem(3,2), 0, SLOP); ASSERT_NEAR(J.elem(3,3), 1.11022e-16, SLOP); ASSERT_NEAR(J.elem(3,4), 1, SLOP); ASSERT_NEAR(J.elem(3,5), 1.66533e-16, SLOP); ASSERT_NEAR(J.elem(3,6), 0, SLOP); ASSERT_NEAR(J.elem(3,7), 0, SLOP); ASSERT_NEAR(J.elem(3,8), 0, SLOP); ASSERT_NEAR(J.elem(3,9), 0, SLOP); ASSERT_NEAR(J.elem(3,10), 0, SLOP); ASSERT_NEAR(J.elem(3,11), 0, SLOP); ASSERT_NEAR(J.elem(3,12), 0, SLOP); ASSERT_NEAR(J.elem(3,13), 0, SLOP); ASSERT_NEAR(J.elem(3,14), 0, SLOP); ASSERT_NEAR(J.elem(3,15), 0, SLOP); ASSERT_NEAR(J.elem(3,16), 0, SLOP); ASSERT_NEAR(J.elem(3,17), 0, SLOP);
    ASSERT_NEAR(J.elem(4,0), 0, SLOP); ASSERT_NEAR(J.elem(4,1), 0, SLOP); ASSERT_NEAR(J.elem(4,2), 0, SLOP); ASSERT_NEAR(J.elem(4,3), 0, SLOP); ASSERT_NEAR(J.elem(4,4), 1.11022e-16, SLOP); ASSERT_NEAR(J.elem(4,5), -1, SLOP); ASSERT_NEAR(J.elem(4,6), 0, SLOP); ASSERT_NEAR(J.elem(4,7), 0, SLOP); ASSERT_NEAR(J.elem(4,8), 0, SLOP); ASSERT_NEAR(J.elem(4,9), 0, SLOP); ASSERT_NEAR(J.elem(4,10), 0, SLOP); ASSERT_NEAR(J.elem(4,11), 0, SLOP); ASSERT_NEAR(J.elem(4,12), 0, SLOP); ASSERT_NEAR(J.elem(4,13), 0, SLOP); ASSERT_NEAR(J.elem(4,14), 0, SLOP); ASSERT_NEAR(J.elem(4,15), 0, SLOP); ASSERT_NEAR(J.elem(4,16), 0, SLOP); ASSERT_NEAR(J.elem(4,17), 0, SLOP);
    ASSERT_NEAR(J.elem(5,0), 0, SLOP); ASSERT_NEAR(J.elem(5,1), 0, SLOP); ASSERT_NEAR(J.elem(5,2), 0, SLOP); ASSERT_NEAR(J.elem(5,3), -1, SLOP); ASSERT_NEAR(J.elem(5,4), 1.66533e-16, SLOP); ASSERT_NEAR(J.elem(5,5), 0, SLOP); ASSERT_NEAR(J.elem(5,6), 0, SLOP); ASSERT_NEAR(J.elem(5,7), 0, SLOP); ASSERT_NEAR(J.elem(5,8), 0, SLOP); ASSERT_NEAR(J.elem(5,9), 0, SLOP); ASSERT_NEAR(J.elem(5,10), 0, SLOP); ASSERT_NEAR(J.elem(5,11), 0, SLOP); ASSERT_NEAR(J.elem(5,12), 0, SLOP); ASSERT_NEAR(J.elem(5,13), 0, SLOP); ASSERT_NEAR(J.elem(5,14), 0, SLOP); ASSERT_NEAR(J.elem(5,15), 0, SLOP); ASSERT_NEAR(J.elem(5,16), 0, SLOP); ASSERT_NEAR(J.elem(5,17), 0, SLOP);
    ASSERT_NEAR(J.elem(6,0), 2.22045e-16, SLOP); ASSERT_NEAR(J.elem(6,1), 0.996195, SLOP); ASSERT_NEAR(J.elem(6,2), -0.0871557, SLOP); ASSERT_NEAR(J.elem(6,3), -4.35779, SLOP); ASSERT_NEAR(J.elem(6,4), 1.43208e-15, SLOP); ASSERT_NEAR(J.elem(6,5), 5.26654e-15, SLOP); ASSERT_NEAR(J.elem(6,6), 0.996195, SLOP); ASSERT_NEAR(J.elem(6,7), 0.0871557, SLOP); ASSERT_NEAR(J.elem(6,8), 0, SLOP); ASSERT_NEAR(J.elem(6,9), -1.5482e-15, SLOP); ASSERT_NEAR(J.elem(6,10), 1.7696e-14, SLOP); ASSERT_NEAR(J.elem(6,11), -2.39808e-14, SLOP); ASSERT_NEAR(J.elem(6,12), 0, SLOP); ASSERT_NEAR(J.elem(6,13), 0, SLOP); ASSERT_NEAR(J.elem(6,14), 0, SLOP); ASSERT_NEAR(J.elem(6,15), 0, SLOP); ASSERT_NEAR(J.elem(6,16), 0, SLOP); ASSERT_NEAR(J.elem(6,17), 0, SLOP);
    ASSERT_NEAR(J.elem(7,0), 0, SLOP); ASSERT_NEAR(J.elem(7,1), -0.0871557, SLOP); ASSERT_NEAR(J.elem(7,2), -0.996195, SLOP); ASSERT_NEAR(J.elem(7,3), -119.81, SLOP); ASSERT_NEAR(J.elem(7,4), 3.18528e-14, SLOP); ASSERT_NEAR(J.elem(7,5), -2.78675e-15, SLOP); ASSERT_NEAR(J.elem(7,6), -0.0871557, SLOP); ASSERT_NEAR(J.elem(7,7), 0.996195, SLOP); ASSERT_NEAR(J.elem(7,8), 0, SLOP); ASSERT_NEAR(J.elem(7,9), -1.7696e-14, SLOP); ASSERT_NEAR(J.elem(7,10), -1.5482e-15, SLOP); ASSERT_NEAR(J.elem(7,11), 70, SLOP); ASSERT_NEAR(J.elem(7,12), 0, SLOP); ASSERT_NEAR(J.elem(7,13), 0, SLOP); ASSERT_NEAR(J.elem(7,14), 0, SLOP); ASSERT_NEAR(J.elem(7,15), 0, SLOP); ASSERT_NEAR(J.elem(7,16), 0, SLOP); ASSERT_NEAR(J.elem(7,17), 0, SLOP);
    ASSERT_NEAR(J.elem(8,0), -1, SLOP); ASSERT_NEAR(J.elem(8,1), 2.22045e-16, SLOP); ASSERT_NEAR(J.elem(8,2), 0, SLOP); ASSERT_NEAR(J.elem(8,3), 1.35467e-15, SLOP); ASSERT_NEAR(J.elem(8,4), 6.1009, SLOP); ASSERT_NEAR(J.elem(8,5), 119.734, SLOP); ASSERT_NEAR(J.elem(8,6), 0, SLOP); ASSERT_NEAR(J.elem(8,7), 0, SLOP); ASSERT_NEAR(J.elem(8,8), 1, SLOP); ASSERT_NEAR(J.elem(8,9), 6.1009, SLOP); ASSERT_NEAR(J.elem(8,10), -69.7336, SLOP); ASSERT_NEAR(J.elem(8,11), 0, SLOP); ASSERT_NEAR(J.elem(8,12), 0, SLOP); ASSERT_NEAR(J.elem(8,13), 0, SLOP); ASSERT_NEAR(J.elem(8,14), 0, SLOP); ASSERT_NEAR(J.elem(8,15), 0, SLOP); ASSERT_NEAR(J.elem(8,16), 0, SLOP); ASSERT_NEAR(J.elem(8,17), 0, SLOP);
    ASSERT_NEAR(J.elem(9,0), 0, SLOP); ASSERT_NEAR(J.elem(9,1), 0, SLOP); ASSERT_NEAR(J.elem(9,2), 0, SLOP); ASSERT_NEAR(J.elem(9,3), 2.22045e-16, SLOP); ASSERT_NEAR(J.elem(9,4), 0.996195, SLOP); ASSERT_NEAR(J.elem(9,5), -0.0871557, SLOP); ASSERT_NEAR(J.elem(9,6), 0, SLOP); ASSERT_NEAR(J.elem(9,7), 0, SLOP); ASSERT_NEAR(J.elem(9,8), 0, SLOP); ASSERT_NEAR(J.elem(9,9), 0.996195, SLOP); ASSERT_NEAR(J.elem(9,10), 0.0871557, SLOP); ASSERT_NEAR(J.elem(9,11), 0, SLOP); ASSERT_NEAR(J.elem(9,12), 0, SLOP); ASSERT_NEAR(J.elem(9,13), 0, SLOP); ASSERT_NEAR(J.elem(9,14), 0, SLOP); ASSERT_NEAR(J.elem(9,15), 0, SLOP); ASSERT_NEAR(J.elem(9,16), 0, SLOP); ASSERT_NEAR(J.elem(9,17), 0, SLOP);
    ASSERT_NEAR(J.elem(10,0), 0, SLOP); ASSERT_NEAR(J.elem(10,1), 0, SLOP); ASSERT_NEAR(J.elem(10,2), 0, SLOP); ASSERT_NEAR(J.elem(10,3), 0, SLOP); ASSERT_NEAR(J.elem(10,4), -0.0871557, SLOP); ASSERT_NEAR(J.elem(10,5), -0.996195, SLOP); ASSERT_NEAR(J.elem(10,6), 0, SLOP); ASSERT_NEAR(J.elem(10,7), 0, SLOP); ASSERT_NEAR(J.elem(10,8), 0, SLOP); ASSERT_NEAR(J.elem(10,9), -0.0871557, SLOP); ASSERT_NEAR(J.elem(10,10), 0.996195, SLOP); ASSERT_NEAR(J.elem(10,11), 0, SLOP); ASSERT_NEAR(J.elem(10,12), 0, SLOP); ASSERT_NEAR(J.elem(10,13), 0, SLOP); ASSERT_NEAR(J.elem(10,14), 0, SLOP); ASSERT_NEAR(J.elem(10,15), 0, SLOP); ASSERT_NEAR(J.elem(10,16), 0, SLOP); ASSERT_NEAR(J.elem(10,17), 0, SLOP);
    ASSERT_NEAR(J.elem(11,0), 0, SLOP); ASSERT_NEAR(J.elem(11,1), 0, SLOP); ASSERT_NEAR(J.elem(11,2), 0, SLOP); ASSERT_NEAR(J.elem(11,3), -1, SLOP); ASSERT_NEAR(J.elem(11,4), 2.22045e-16, SLOP); ASSERT_NEAR(J.elem(11,5), 0, SLOP); ASSERT_NEAR(J.elem(11,6), 0, SLOP); ASSERT_NEAR(J.elem(11,7), 0, SLOP); ASSERT_NEAR(J.elem(11,8), 0, SLOP); ASSERT_NEAR(J.elem(11,9), 0, SLOP); ASSERT_NEAR(J.elem(11,10), 0, SLOP); ASSERT_NEAR(J.elem(11,11), 1, SLOP); ASSERT_NEAR(J.elem(11,12), 0, SLOP); ASSERT_NEAR(J.elem(11,13), 0, SLOP); ASSERT_NEAR(J.elem(11,14), 0, SLOP); ASSERT_NEAR(J.elem(11,15), 0, SLOP); ASSERT_NEAR(J.elem(11,16), 0, SLOP); ASSERT_NEAR(J.elem(11,17), 0, SLOP);
    ASSERT_NEAR(J.elem(12,0), 0, SLOP); ASSERT_NEAR(J.elem(12,1), 0.707107, SLOP); ASSERT_NEAR(J.elem(12,2), -0.707107, SLOP); ASSERT_NEAR(J.elem(12,3), -80.3505, SLOP); ASSERT_NEAR(J.elem(12,4), 2.26093e-14, SLOP); ASSERT_NEAR(J.elem(12,5), 2.26093e-14, SLOP); ASSERT_NEAR(J.elem(12,6), 0.707107, SLOP); ASSERT_NEAR(J.elem(12,7), 0.707107, SLOP); ASSERT_NEAR(J.elem(12,8), -8.97498e-17, SLOP); ASSERT_NEAR(J.elem(12,9), -3.66959e-14, SLOP); ASSERT_NEAR(J.elem(12,10), 4.2407e-14, SLOP); ASSERT_NEAR(J.elem(12,11), 44.9951, SLOP); ASSERT_NEAR(J.elem(12,12), 0.766044, SLOP); ASSERT_NEAR(J.elem(12,13), 0.642788, SLOP); ASSERT_NEAR(J.elem(12,14), -9.9005e-17, SLOP); ASSERT_NEAR(J.elem(12,15), 1.4769e-14, SLOP); ASSERT_NEAR(J.elem(12,16), -1.7601e-14, SLOP); ASSERT_NEAR(J.elem(12,17), 0, SLOP);
    ASSERT_NEAR(J.elem(13,0), -2.22045e-16, SLOP); ASSERT_NEAR(J.elem(13,1), -0.707107, SLOP); ASSERT_NEAR(J.elem(13,2), -0.707107, SLOP); ASSERT_NEAR(J.elem(13,3), -143.978, SLOP); ASSERT_NEAR(J.elem(13,4), 3.25995e-14, SLOP); ASSERT_NEAR(J.elem(13,5), 1.26124e-14, SLOP); ASSERT_NEAR(J.elem(13,6), -0.707107, SLOP); ASSERT_NEAR(J.elem(13,7), 0.707107, SLOP); ASSERT_NEAR(J.elem(13,8), 3.71756e-17, SLOP); ASSERT_NEAR(J.elem(13,9), -3.09853e-14, SLOP); ASSERT_NEAR(J.elem(13,10), -3.66961e-14, SLOP); ASSERT_NEAR(J.elem(13,11), 108.623, SLOP); ASSERT_NEAR(J.elem(13,12), -0.642788, SLOP); ASSERT_NEAR(J.elem(13,13), 0.766044, SLOP); ASSERT_NEAR(J.elem(13,14), -8.27043e-18, SLOP); ASSERT_NEAR(J.elem(13,15), 2.14799e-14, SLOP); ASSERT_NEAR(J.elem(13,16), 1.86176e-14, SLOP); ASSERT_NEAR(J.elem(13,17), 55, SLOP);
    ASSERT_NEAR(J.elem(14,0), -1, SLOP); ASSERT_NEAR(J.elem(14,1), 2.22045e-16, SLOP); ASSERT_NEAR(J.elem(14,2), 0, SLOP); ASSERT_NEAR(J.elem(14,3), 9.99018e-15, SLOP); ASSERT_NEAR(J.elem(14,4), 44.9918, SLOP); ASSERT_NEAR(J.elem(14,5), 158.625, SLOP); ASSERT_NEAR(J.elem(14,6), 8.97498e-17, SLOP); ASSERT_NEAR(J.elem(14,7), 3.71756e-17, SLOP); ASSERT_NEAR(J.elem(14,8), 1, SLOP); ASSERT_NEAR(J.elem(14,9), 44.9918, SLOP); ASSERT_NEAR(J.elem(14,10), -108.625, SLOP); ASSERT_NEAR(J.elem(14,11), 1.7653e-19, SLOP); ASSERT_NEAR(J.elem(14,12), 7.05261e-17, SLOP); ASSERT_NEAR(J.elem(14,13), 6.99747e-17, SLOP); ASSERT_NEAR(J.elem(14,14), 1, SLOP); ASSERT_NEAR(J.elem(14,15), 35.3533, SLOP); ASSERT_NEAR(J.elem(14,16), -42.1324, SLOP); ASSERT_NEAR(J.elem(14,17), 4.54873e-16, SLOP);
    ASSERT_NEAR(J.elem(15,0), 0, SLOP); ASSERT_NEAR(J.elem(15,1), 0, SLOP); ASSERT_NEAR(J.elem(15,2), 0, SLOP); ASSERT_NEAR(J.elem(15,3), 0, SLOP); ASSERT_NEAR(J.elem(15,4), 0.707107, SLOP); ASSERT_NEAR(J.elem(15,5), -0.707107, SLOP); ASSERT_NEAR(J.elem(15,6), 0, SLOP); ASSERT_NEAR(J.elem(15,7), 0, SLOP); ASSERT_NEAR(J.elem(15,8), 0, SLOP); ASSERT_NEAR(J.elem(15,9), 0.707107, SLOP); ASSERT_NEAR(J.elem(15,10), 0.707107, SLOP); ASSERT_NEAR(J.elem(15,11), -8.97498e-17, SLOP); ASSERT_NEAR(J.elem(15,12), 0, SLOP); ASSERT_NEAR(J.elem(15,13), 0, SLOP); ASSERT_NEAR(J.elem(15,14), 0, SLOP); ASSERT_NEAR(J.elem(15,15), 0.766044, SLOP); ASSERT_NEAR(J.elem(15,16), 0.642788, SLOP); ASSERT_NEAR(J.elem(15,17), -9.9005e-17, SLOP);
    ASSERT_NEAR(J.elem(16,0), 0, SLOP); ASSERT_NEAR(J.elem(16,1), 0, SLOP); ASSERT_NEAR(J.elem(16,2), 0, SLOP); ASSERT_NEAR(J.elem(16,3), -2.22045e-16, SLOP); ASSERT_NEAR(J.elem(16,4), -0.707107, SLOP); ASSERT_NEAR(J.elem(16,5), -0.707107, SLOP); ASSERT_NEAR(J.elem(16,6), 0, SLOP); ASSERT_NEAR(J.elem(16,7), 0, SLOP); ASSERT_NEAR(J.elem(16,8), 0, SLOP); ASSERT_NEAR(J.elem(16,9), -0.707107, SLOP); ASSERT_NEAR(J.elem(16,10), 0.707107, SLOP); ASSERT_NEAR(J.elem(16,11), 3.71756e-17, SLOP); ASSERT_NEAR(J.elem(16,12), 0, SLOP); ASSERT_NEAR(J.elem(16,13), 0, SLOP); ASSERT_NEAR(J.elem(16,14), 0, SLOP); ASSERT_NEAR(J.elem(16,15), -0.642788, SLOP); ASSERT_NEAR(J.elem(16,16), 0.766044, SLOP); ASSERT_NEAR(J.elem(16,17), -8.27043e-18, SLOP);
    ASSERT_NEAR(J.elem(17,0), 0, SLOP); ASSERT_NEAR(J.elem(17,1), 0, SLOP); ASSERT_NEAR(J.elem(17,2), 0, SLOP); ASSERT_NEAR(J.elem(17,3), -1, SLOP); ASSERT_NEAR(J.elem(17,4), 2.22045e-16, SLOP); ASSERT_NEAR(J.elem(17,5), 0, SLOP); ASSERT_NEAR(J.elem(17,6), 0, SLOP); ASSERT_NEAR(J.elem(17,7), 0, SLOP); ASSERT_NEAR(J.elem(17,8), 0, SLOP); ASSERT_NEAR(J.elem(17,9), 8.97498e-17, SLOP); ASSERT_NEAR(J.elem(17,10), 3.71756e-17, SLOP); ASSERT_NEAR(J.elem(17,11), 1, SLOP); ASSERT_NEAR(J.elem(17,12), 0, SLOP); ASSERT_NEAR(J.elem(17,13), 0, SLOP); ASSERT_NEAR(J.elem(17,14), 0, SLOP); ASSERT_NEAR(J.elem(17,15), 7.05261e-17, SLOP); ASSERT_NEAR(J.elem(17,16), 6.99747e-17, SLOP); ASSERT_NEAR(J.elem(17,17), 1, SLOP);


    //Matrix linkJacobianInWorldFrame = c->linkJacobian(true);


}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

