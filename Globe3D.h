#pragma once

#include "Ogre.h"
#include "OgreApplicationContext.h"
#include "OgreCameraMan.h"
#include "OgreTrays.h"
#include "OgreImGuiOverlay.h"
#include "OgreImGuiInputListener.h"

using namespace Ogre;
using namespace OgreBites;

// Сан-Паулу.
static const float LAT_A = -23.555771f;
static const float LON_A = -46.639557f;

// Нью-Дели.
static const float LAT_B = 28.613830f;
static const float LON_B = 77.208491f;

class Globe3D : public ApplicationContext, public InputListener, public TrayListener, public RenderTargetListener
{
public:
    Globe3D();
    virtual ~Globe3D();

    void setup();

    bool keyPressed(const KeyboardEvent& evt) override { return listenerChain.keyPressed(evt); }
    bool keyReleased(const KeyboardEvent& evt) override { return listenerChain.keyReleased(evt); }
    bool mouseMoved(const MouseMotionEvent& evt) override { return listenerChain.mouseMoved(evt); }
    bool mouseWheelRolled(const MouseWheelEvent& evt) override { return listenerChain.mouseWheelRolled(evt); }
    bool mousePressed(const MouseButtonEvent& evt) override { return listenerChain.mousePressed(evt); }
    bool mouseReleased(const MouseButtonEvent& evt) override { return listenerChain.mouseReleased(evt); }
    bool textInput(const TextInputEvent& evt) override { return listenerChain.textInput(evt); }

    void preViewportUpdate(const RenderTargetViewportEvent& evt);

protected:
    NativeWindowPair createWindow(const Ogre::String& name, uint32_t w, uint32_t h, Ogre::NameValuePairList miscParams) override {
        return ApplicationContext::createWindow(name, 1024, 768, miscParams);
    }

private:
    SceneManager* scnMgr;
    std::unique_ptr<CameraMan> camMan;
    std::unique_ptr<TrayManager> trayMgr;
    std::unique_ptr<ImGuiInputListener> imGuiListener;
    InputListenerChain listenerChain;

    MeshPtr sphereMesh;
    ManualObject* lineMesh;

    // Базисные векторы для плоскости, образованной векторами локаций.
    Vector3f xPlaneAB;
    Vector3f yPlaneAB;

    float placeALatLon[2] = { LAT_A, LON_A };
    float placeBLatLon[2] = { LAT_B, LON_B };

    SceneNode* placeANode;
    SceneNode* placeBNode;

    float surfaceDistanceKm = 0;
    float directDistanceKm = 0;

    Vector3f LatLonToCartesian(float lat, float lon, float rho);
    Vector3f SphericalToCartesian(float rho, float theta, float phi);

    void BuildSphereMesh(std::string meshName, float radius);
    int GetRightSiblingIndex(int index, int vertexCount);
    int GetUpSiblingIndex(int index);

    float GetAngleBetweenVectors(Vector3f a, Vector3f b);

    void RecalcPlaneAB(Vector3f a, Vector3f b);
    void RebuildSurfacePath(Vector3f a, Vector3f b);

    void PreparePathLine();
    void PrepareLight(SceneNode* node);
    SceneNode* PrepareCam();
    void PrepareImGui();
    void CreateImGuiOverlay();

    void CreateGlobe();
    void CreatePlacemarks();

    void CreateLonMarks(SceneNode* node);
    void CreateLatMarks(SceneNode* node);

    void UpdatePlaceFeatures();
    float CalcSurfaceDistance(Vector3f a, Vector3f b);
    void RecalcDistanceKm(Vector3f a, Vector3f b);
};
