#include "graspit/display/graspitDisplay/displayBody.h"

DisplayBody::DisplayBody(Body *b):
    mBody(b)
{
    IVRoot = new SoSeparator;
    IVTran = new SoTransform;
    IVRoot->insertChild(IVTran, 0);

    IVContactIndicators = new SoSeparator;
    IVRoot->addChild(IVContactIndicators);

    IVBVRoot = new SoSeparator;
    IVRoot->addChild(IVBVRoot);

    IVGeomRoot = new SoSeparator;
    IVRoot->addChild(IVGeomRoot);
}

