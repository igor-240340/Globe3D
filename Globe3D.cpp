#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>

#include "Globe3D.h"

static const float SPHERE_RADIUS = 30.0f;
static const unsigned short int MERIDIANS = 36 + 1; // Дополнительный меридиан, совпадающий со 180-м, чтобы замкнуть текстуру.
static const unsigned short int PARALLELS = 17 + 2; // Две дополнительные параллели для текстурирования полюсов (все их точки будут сведены в полюс).

//
//
//
Globe3D::Globe3D() : ApplicationContext("Globe3D")
{
}

//
//
//
bool Globe3D::keyPressed(const KeyboardEvent& evt)
{
    camMan->keyPressed(evt);
    return true;
}

//
//
//
bool Globe3D::keyReleased(const KeyboardEvent& evt)
{
    camMan->keyReleased(evt);
    return true;
}

//
//
//
bool Globe3D::mousePressed(const MouseButtonEvent& evt)
{
    camMan->mousePressed(evt);
    return true;
}

bool Globe3D::mouseReleased(const MouseButtonEvent& evt)
{
    camMan->mouseReleased(evt);
    return true;
}

//
//
//
bool Globe3D::mouseMoved(const MouseMotionEvent& evt) {
    camMan->mouseMoved(evt);
    return true;
}

//
//
//
bool Globe3D::mouseWheelRolled(const MouseWheelEvent& evt) {
    camMan->mouseWheelRolled(evt);
    return true;
}

//
//
//
void Globe3D::setup()
{
    ResourceGroupManager::getSingleton().addResourceLocation("./Data", "FileSystem");

    ApplicationContext::setup();
    addInputListener(this);

    Root* root = getRoot();
    SceneManager* scnMgr = root->createSceneManager();

    RTShader::ShaderGenerator* shadergen = RTShader::ShaderGenerator::getSingletonPtr();
    shadergen->addSceneManager(scnMgr);

    scnMgr->setAmbientLight(ColourValue(1, 1, 1));

    //Light* light = scnMgr->createLight("light");
    //SceneNode* lightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    //lightNode->attachObject(light);
    //lightNode->setPosition(20, 80, 100);

    Camera* cam = scnMgr->createCamera("cam");
    cam->setNearClipDistance(10);
    cam->setAutoAspectRatio(true);
    //cam->setProjectionType(PT_ORTHOGRAPHIC);

    SceneNode* camNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    camNode->attachObject(cam);
    camNode->setPosition(0, 0, 0);
    //camNode->lookAt(Ogre::Vector3(0, -30, 0), Ogre::Node::TS_PARENT);

    camMan.reset(new CameraMan(camNode));
    camMan->setStyle(OgreBites::CS_ORBIT);

    getRenderWindow()->addViewport(cam);

    //
    /*auto tex = TextureManager::getSingleton().loadImage(CIRCLES_MATERIAL, RGN_DEFAULT, bmap);
    MaterialPtr material = MaterialManager::getSingleton().create(CIRCLES_MATERIAL, RGN_DEFAULT);
    auto texLayer = material->getTechnique(0)->getPass(0)->createTextureUnitState();
    texLayer->setTexture(tex);
    texLayer->setTextureAddressingMode(TextureUnitState::TAM_CLAMP);
    material->setSceneBlending(SBT_ADD);
    material->setLightingEnabled(false);
    material->setDepthWriteEnabled(false);*/
    //

    GenerateSphereMesh("sphere", SPHERE_RADIUS);
    Entity* sphereEntity = scnMgr->createEntity("Globe", "sphere");
    sphereEntity->setMaterialName("Earth", "General");
    SceneNode* sphereNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    sphereNode->attachObject(sphereEntity);
    //sphereNode->yaw(Degree(-50));
    //sphereNode->pitch(Degree(-10));

    Pass* pass = sphereEntity->getSubEntity(0)->getMaterial()->getTechnique(0)->getPass(0);
    pass->setPolygonMode(PM_SOLID);
}

//
// Создает меш для сферы.
//
void Globe3D::GenerateSphereMesh(std::string meshName, float radius) {
    sphereMesh = MeshManager::getSingleton().createManual(meshName, RGN_DEFAULT);
    sphereMesh->_setBounds(AxisAlignedBox({ -100, -100, 0 }, { 100, 100, 0 }));

    // Меридиан с индексом 0 мы берем за 180-й, а не за нулевой, потому что
    // нулевая текстурная координата для файла с текстурой земли соответствует 180-му меридиану,
    // а нулевому меридиану соответствует середина текстуры.
    // Поэтому мы должны начать с 180-го меридиана, чтобы на момент,
    // когда мы доберемся в построении до 0-го меридиана, у нас были вычислены правильные текстурные координаты, соответствующие половине текстуры Земли.
    float angleStep = M_PI / (PARALLELS - 1);                   // Шаг общий для меридианов и параллелей.
    const int vertexCount = MERIDIANS * PARALLELS;
    float vertices[vertexCount * (3 + 3 + 2)];                  // (vertex, normal, texcoord).
    int verticesIndex = 0;                                      // Сквозной индекс по массиву vertices.

    float texcoordXStep = 1.0f / (MERIDIANS - 1);   // Шаг при котором между меридианами и их текстурными координатами есть отображение [0..1]->[0..MERIDIANS-1].
    float texcoordYStep = 1.0f / (PARALLELS - 1);

    const int triangleCount = (PARALLELS - 1) * (MERIDIANS - 1) * 2;
    uint16 triangles[triangleCount * 3];                    // Индексы, образующие треугольные полигоны.
    int vertexIndex = 0;                                    // Абстрактный индекс, определяющий порядковый номер вершины.
    int trianglesIndex = 0;                                 // Сквозной индекс по массиву triangles.
    for (int i = 0; i < MERIDIANS; i++) {
        for (int j = 0; j < PARALLELS; j++) {   // Для каждой параллели j вычисляет вершину, лежащую на мередиане i.
            // Вектор +Z, через который проходит нулевой меридиан, смотрит в камеру,
            // а начать построение мы должны со 180-го меридиана.
            // Поэтому начинаем c вектора -Z, который лежит в отрицательной полуплоскости, то есть - с угла 3*(Pi/2).
            float theta = angleStep * j;
            float phi = 3 * (M_PI / 2) - (angleStep * i);

            // Вершина.
            Vector3f v = SphericalToCartesian(radius, theta, phi);
            vertices[verticesIndex++] = v.x;
            vertices[verticesIndex++] = v.y;
            vertices[verticesIndex++] = v.z;

            // Нормаль.
            Vector3f n = v.normalisedCopy();
            vertices[verticesIndex++] = n.x;
            vertices[verticesIndex++] = n.y;
            vertices[verticesIndex++] = n.z;

            vertices[verticesIndex++] = texcoordXStep * i;
            vertices[verticesIndex++] = texcoordYStep * j;

            // Задаем индексы треугольников. Обход вершин для лицевых сторон - CCW.
            // Нулевую параллель и последний меридиан пропускаем.
            if (i != MERIDIANS - 1 && j != 0) {
                triangles[trianglesIndex++] = vertexIndex;
                triangles[trianglesIndex++] = GetRightSiblingIndex(vertexIndex, PARALLELS, vertexCount);
                triangles[trianglesIndex++] = GetUpSiblingIndex(GetRightSiblingIndex(vertexIndex, PARALLELS, vertexCount));

                triangles[trianglesIndex++] = vertexIndex;
                triangles[trianglesIndex++] = GetUpSiblingIndex(GetRightSiblingIndex(vertexIndex, PARALLELS, vertexCount));
                triangles[trianglesIndex++] = GetUpSiblingIndex(vertexIndex);
            }

            vertexIndex++;
        }
    }

    sphereMesh->sharedVertexData = new VertexData();
    sphereMesh->sharedVertexData->vertexCount = vertexCount;
    VertexDeclaration* decl = sphereMesh->sharedVertexData->vertexDeclaration;
    VertexBufferBinding* bind = sphereMesh->sharedVertexData->vertexBufferBinding;

    size_t offset = 0;
    offset += decl->addElement(0, offset, VET_FLOAT3, VES_POSITION).getSize();
    offset += decl->addElement(0, offset, VET_FLOAT3, VES_NORMAL).getSize();
    offset += decl->addElement(0, offset, VET_FLOAT2, VES_TEXTURE_COORDINATES, 0).getSize();

    HardwareVertexBufferPtr vbuf = HardwareBufferManager::getSingleton().createVertexBuffer(offset, vertexCount, HBU_GPU_ONLY);
    vbuf->writeData(0, vbuf->getSizeInBytes(), vertices, true);
    bind->setBinding(0, vbuf);

    HardwareIndexBufferPtr ibuf = HardwareBufferManager::getSingleton().createIndexBuffer(HardwareIndexBuffer::IT_16BIT, triangleCount * 3, HBU_GPU_ONLY);
    ibuf->writeData(0, ibuf->getSizeInBytes(), triangles, true);

    SubMesh* sub = sphereMesh->createSubMesh();
    sub->useSharedVertices = true;
    sub->indexData->indexBuffer = ibuf;
    sub->indexData->indexCount = triangleCount * 3;
    sub->indexData->indexStart = 0;

    sphereMesh->load();
}

//
// Возвращает радиус-вектор точки, заданной в сферической СК.
//
Vector3f Globe3D::SphericalToCartesian(float rho, float theta, float phi) {
    float y = cos(theta) * rho;

    float projXZ = sin(theta) * rho; // Величина проекции вектора на плоскость XZ.
    float x = cos(phi) * projXZ;
    float z = sin(phi) * projXZ;

    return Vector3f(x, y, z);
}

//
// Возвращает индекс соседней вершины с правого меридиана.
//
int Globe3D::GetRightSiblingIndex(int index, int PARALLELS, int vertexCount) {
    return index + PARALLELS;
}

//
// Возвращает индекс соседней вершины с верхней параллели.
//
int Globe3D::GetUpSiblingIndex(int index) {
    return index - 1;
}
