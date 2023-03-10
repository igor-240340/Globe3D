#pragma once

#include "Ogre.h"
#include "OgreApplicationContext.h"
#include "OgreInput.h"
#include "OgreCameraMan.h"

using namespace Ogre;
using namespace OgreBites;

class Globe3D : public ApplicationContext, public InputListener
{
public:
    Globe3D();
    virtual ~Globe3D() {}

    void setup();

    bool keyPressed(const KeyboardEvent& evt);
    bool keyReleased(const KeyboardEvent& evt);

    bool mousePressed(const MouseButtonEvent& evt);
    bool mouseReleased(const MouseButtonEvent& evt);
    bool mouseMoved(const MouseMotionEvent& evt);
    bool mouseWheelRolled(const MouseWheelEvent& evt);

private:
    std::unique_ptr<CameraMan> camMan;

    void GenerateSphereMesh(std::string meshName, float radius);
    Vector3f SphericalToCartesian(float rho, float theta, float phi);
    int GetRightSiblingIndex(int index, int PARALLELS, int vertexCount);
    int GetUpSiblingIndex(int index);

    MeshPtr sphereMesh;
};
