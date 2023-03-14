#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>

#include "Globe3D.h"

static const float EARTH_RADIUS_IN_UNITS = 30.0f;
static const float EARTH_RADIUS_IN_KM = 6371.0f;
static const float UNIT_TO_KM = EARTH_RADIUS_IN_KM / EARTH_RADIUS_IN_UNITS;

static const unsigned short int MERIDIANS = 36 + 1; // Дополнительный меридиан, совпадающий со 180-м, чтобы визуально замкнуть текстуру.
static const unsigned short int PARALLELS = 17 + 2; // Две дополнительные параллели для текстурирования полюсов (все их точки будут сведены в полюсы).

static const float DEG_TO_RAD = M_PI / 180.0f;
static const float RAD_TO_DEG = 180.0f / M_PI;

//
// Общая инициализация приложения и настройка сцены.
//
void Globe3D::setup() {
    ResourceGroupManager::getSingleton().addResourceLocation("./Data", "FileSystem");
    // Необходим для трея, который мы используем в связке с ImGui.
    ResourceGroupManager::getSingleton().addResourceLocation("./Data/SdkTrays.zip", "Zip");
    ResourceGroupManager::getSingleton().addResourceLocation("./Data/SpaceSkyBox.zip", "Zip");

    ApplicationContext::setup();

    addInputListener(this);
    getRenderWindow()->addListener(this);   // Слушаем preViewportUpdate.

    Root* root = getRoot();
    scnMgr = root->createSceneManager();

    RTShader::ShaderGenerator* shaderGen = RTShader::ShaderGenerator::getSingletonPtr();
    shaderGen->addSceneManager(scnMgr);

    SceneNode* cam = PrepareCam();
    PrepareLight(cam);

    CreateGlobe();

    CreatePlacemarks();
    PreparePathLine();

    RecalcDistanceKm(placeANode->getPosition(), placeBNode->getPosition());
    RebuildSurfacePath(placeANode->getPosition(), placeBNode->getPosition());

    PrepareImGui();

    scnMgr->setSkyBox(true, "SpaceSkyBox");
}

//
// Создает поля ввода/вывода для параметров локаций.
//
void Globe3D::CreateImGuiOverlay() {
    static int location = 0;
    ImGuiIO& io = ImGui::GetIO();
    ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_NoDecoration
        | ImGuiWindowFlags_AlwaysAutoResize
        | ImGuiWindowFlags_NoSavedSettings
        | ImGuiWindowFlags_NoFocusOnAppearing
        | ImGuiWindowFlags_NoNav;

    if (location >= 0) {
        const float PAD = 10.0f;
        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImVec2 work_pos = viewport->WorkPos;
        ImVec2 work_size = viewport->WorkSize;
        ImVec2 window_pos, window_pos_pivot;
        window_pos.x = (location & 1) ? (work_pos.x + work_size.x - PAD) : (work_pos.x + PAD);
        window_pos.y = (location & 2) ? (work_pos.y + work_size.y - PAD) : (work_pos.y + PAD);
        window_pos_pivot.x = (location & 1) ? 1.0f : 0.0f;
        window_pos_pivot.y = (location & 2) ? 1.0f : 0.0f;
        ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
        window_flags |= ImGuiWindowFlags_NoMove;
    }
    else if (location == -2) {
        ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
        window_flags |= ImGuiWindowFlags_NoMove;
    }

    ImGui::SetNextWindowBgAlpha(0.35f);
    if (ImGui::Begin("Place overlay.", nullptr, window_flags)) {
        ImGui::Text("Zoom: use mouse wheel.");
        ImGui::Text("Rotation: press left mouse and move.");

        bool locIsChanged = false;

        ImGui::Separator();
        ImGui::Text("Place A (Lat, Lon)");
        if (ImGui::InputFloat2("##1", placeALatLon)) {
            locIsChanged = true;
        }

        ImGui::Separator();
        ImGui::Text("Place B (Lat, Lon)");
        if (ImGui::InputFloat2("##2", placeBLatLon)) {
            locIsChanged = true;
        }

        if (locIsChanged) {
            UpdatePlaceFeatures();
        }

        ImGui::Separator();
        ImGui::Text("Surface distance: %.2f km.", surfaceDistanceKm);
        ImGui::Text("Direct distance: %.2f km.", directDistanceKm);
    }

    ImGui::End();
}

//
//
//
void Globe3D::UpdatePlaceFeatures() {
    Vector3f placeAPos = LatLonToCartesian(placeALatLon[0], placeALatLon[1], EARTH_RADIUS_IN_UNITS);
    Vector3f placeBPos = LatLonToCartesian(placeBLatLon[0], placeBLatLon[1], EARTH_RADIUS_IN_UNITS);

    placeANode->setPosition(placeAPos);
    placeBNode->setPosition(placeBPos);

    RecalcDistanceKm(placeAPos, placeBPos);
    RebuildSurfacePath(placeAPos, placeBPos);
}

void Globe3D::RecalcDistanceKm(Vector3f a, Vector3f b) {
    surfaceDistanceKm = CalcSurfaceDistance(a, b) * UNIT_TO_KM;
    directDistanceKm = (a - b).length() * UNIT_TO_KM;
}

//
// https://ogrecave.github.io/ogre/api/13/class_ogre_1_1_render_target_listener.html#ae0f7f5f1cdf13b5870d0da1cc8f1bd64
//
void Globe3D::preViewportUpdate(const RenderTargetViewportEvent& evt) {
    if (!evt.source->getOverlaysEnabled()) return;
    if (!trayMgr->getTraysLayer()->isVisible()) return;

    ImGuiOverlay::NewFrame();
    CreateImGuiOverlay();
}

//
// Инициализация ImGui.
// https://github.com/OGRECave/ogre/blob/9dfd6c9dcdd304615d4e469e24cf130fa992af30/Samples/Simple/include/ImGuiDemo.h#L42
//
void Globe3D::PrepareImGui() {
    trayMgr.reset(new TrayManager("TrayManager", getRenderWindow(), this));

    auto imguiOverlay = new ImGuiOverlay();

    // Handle DPI scaling
    float vpScale = OverlayManager::getSingleton().getPixelRatio();
    ImGui::GetIO().FontGlobalScale = std::round(vpScale); // Default font does not work with fractional scaling.
    ImGui::GetStyle().ScaleAllSizes(vpScale);

    imguiOverlay->setZOrder(300);
    imguiOverlay->show();
    OverlayManager::getSingleton().addOverlay(imguiOverlay);

    scnMgr->addRenderQueueListener(mOverlaySystem);

    imGuiListener.reset(new ImGuiInputListener());
    listenerChain = InputListenerChain({ trayMgr.get(), imGuiListener.get(), camMan.get() });

    trayMgr->hideCursor();
}

//
//
//
void Globe3D::CreatePlacemarks() {
    BuildSphereMesh("PlacemarkMesh", 0.5f);

    Entity* placeAEntity = scnMgr->createEntity("PlaceA", "PlacemarkMesh");
    placeAEntity->setMaterialName("Red", "General");
    placeANode = scnMgr->getRootSceneNode()->createChildSceneNode();
    placeANode->attachObject(placeAEntity);
    placeAEntity->getSubEntity(0)->getMaterial()->getTechnique(0)->getPass(0)->setPolygonMode(PM_SOLID);
    placeANode->setPosition(LatLonToCartesian(placeALatLon[0], placeALatLon[1], EARTH_RADIUS_IN_UNITS));

    Entity* placeBEntity = scnMgr->createEntity("PlaceB", "PlacemarkMesh");
    placeBEntity->setMaterialName("Red", "General");
    placeBNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    placeBNode->attachObject(placeBEntity);
    placeBEntity->getSubEntity(0)->getMaterial()->getTechnique(0)->getPass(0)->setPolygonMode(PM_SOLID);
    placeBNode->setPosition(LatLonToCartesian(placeBLatLon[0], placeBLatLon[1], EARTH_RADIUS_IN_UNITS));
}

//
//
//
void Globe3D::CreateGlobe() {
    BuildSphereMesh("GlobeMesh", EARTH_RADIUS_IN_UNITS);

    Entity* globeEntity = scnMgr->createEntity("Globe", "GlobeMesh");
    globeEntity->setMaterialName("Earth", "General");
    SceneNode* globeNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    globeNode->attachObject(globeEntity);
    globeEntity->getSubEntity(0)->getMaterial()->getTechnique(0)->getPass(0)->setPolygonMode(PM_SOLID);
}

//
// Создает и связывает меш, ноду и материал для линии, которая будет представлять путь между локациями.
//
void Globe3D::PreparePathLine() {
    lineMesh = scnMgr->createManualObject("LineMesh");
    SceneNode* lineNode = scnMgr->getRootSceneNode()->createChildSceneNode("PathAB");

    MaterialPtr lineMaterial = MaterialManager::getSingleton().create("LineMaterial", "General");
    lineMaterial->setReceiveShadows(false);
    lineMaterial->getTechnique(0)->setLightingEnabled(true);
    lineMaterial->getTechnique(0)->getPass(0)->setDiffuse(1, 0, 0, 0);
    lineMaterial->getTechnique(0)->getPass(0)->setAmbient(1, 0, 0);
    lineMaterial->getTechnique(0)->getPass(0)->setSelfIllumination(1, 0, 0);

    lineNode->attachObject(lineMesh);
}

//
//
//
SceneNode* Globe3D::PrepareCam() {
    Camera* cam = scnMgr->createCamera("MainCamera");
    cam->setNearClipDistance(5);
    cam->setAutoAspectRatio(true);

    SceneNode* camNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    camNode->attachObject(cam);
    camNode->setPosition(0, 0, 0);

    camMan.reset(new CameraMan(camNode));
    camMan->setStyle(OgreBites::CS_ORBIT);

    getRenderWindow()->addViewport(cam);

    return camNode;
}

//
//
//
void Globe3D::PrepareLight(SceneNode* node) {
    //scnMgr->setAmbientLight(ColourValue(0.1f, 0.1f, 0.1f));

    Light* light = scnMgr->createLight("Sun");
    node->attachObject(light);
}

//
// Перестраивает путь из пункта A в пункт B по поверхности сферы при изменении координат.
// Поскольку за геометрическую модель поверхности Земли мы взяли сферу, то этот путь эквивалентен дуге окружности в плоскости векторов A и B.
//
void Globe3D::RebuildSurfacePath(Vector3f a, Vector3f b) {
    lineMesh->clear();
    lineMesh->begin("LineMaterial", RenderOperation::OT_LINE_STRIP);

    RecalcPlaneAB(a, b);
    float angle = GetAngleBetweenVectors(a, b);
    const int edges = 15;
    const int vertices = edges + 1;
    const float angleStep = angle / edges;
    for (int i = 0; i < vertices; i++) {
        float phi = i * angleStep;

        float x = cos(phi) * (EARTH_RADIUS_IN_UNITS + 0.05f);
        float y = sin(phi) * (EARTH_RADIUS_IN_UNITS + 0.05f);

        // Не забываем, что дугу мы рисуем в плоскости AB.
        Vector3f pos = xPlaneAB * x + yPlaneAB * y;
        lineMesh->position(pos);
    }

    lineMesh->end();
}

//
// Вычисляет кратчайшее расстояние между точками по поверхности сферы.
//
float Globe3D::CalcSurfaceDistance(Vector3f a, Vector3f b) {
    return GetAngleBetweenVectors(a, b) * EARTH_RADIUS_IN_UNITS;
}

//
// Вычисляет базисные векторы для плоскости, образованной векторами локаций.
//
void Globe3D::RecalcPlaneAB(Vector3f a, Vector3f b) {
    // Единичный вектора A определяет ось X.
    xPlaneAB = a.normalisedCopy();

    // Перпендикулярный к X вектор определяет ось Y.
    Vector3f vectorBProjX = b.dotProduct(xPlaneAB) * xPlaneAB;
    yPlaneAB = (b - vectorBProjX).normalisedCopy();
}

//
// Вычисляет угол между векторами.
// dot(A,B) = |A|*|B|*cos(angle).
//
float Globe3D::GetAngleBetweenVectors(Vector3f a, Vector3f b) {
    return acos(a.dotProduct(b) / (a.length() * b.length()));
}

//
// Строит меш для текстурируемой сферы.
//
void Globe3D::BuildSphereMesh(std::string meshName, float radius) {
    // Меридиан с индексом 0 мы берем за 180-й, а не за нулевой, потому что
    // нулевая текстурная координата для файла с текстурой земли соответствует 180-му меридиану,
    // а нулевому меридиану соответствует середина текстуры.
    // Поэтому мы должны начать с 180-го меридиана, чтобы на момент,
    // когда мы доберемся в построении до 0-го меридиана, у нас были вычислены правильные текстурные координаты, соответствующие половине текстуры Земли.
    float angleStep = M_PI / (PARALLELS - 1);                   // Шаг общий для меридианов и параллелей.
    const int vertexCount = MERIDIANS * PARALLELS;
    float* vertices = new float[vertexCount * (3 + 3 + 2)];     // (vertex, normal, texcoord).
    int verticesIndex = 0;                                      // Сквозной индекс по массиву vertices.

    // Шаг, при котором между меридианами/параллелями и их текстурными координатами
    // существует отображение вида [0..1]->[0..MERIDIANS/PARALLELS-1],
    // где MERIDIANS/PARALLELS-1 - индекс последнего меридиана/параллели (здесь "MERIDIANS/PARALLELS" - это не арифметическое деление).
    float texcoordXStep = 1.0f / (MERIDIANS - 1);
    float texcoordYStep = 1.0f / (PARALLELS - 1);

    const int triangleCount = (PARALLELS - 1) * (MERIDIANS - 1) * 2;
    uint16* triangles = new uint16[triangleCount * 3];      // Индексы, образующие треугольные полигоны.
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
                triangles[trianglesIndex++] = GetRightSiblingIndex(vertexIndex, vertexCount);
                triangles[trianglesIndex++] = GetUpSiblingIndex(GetRightSiblingIndex(vertexIndex, vertexCount));

                triangles[trianglesIndex++] = vertexIndex;
                triangles[trianglesIndex++] = GetUpSiblingIndex(GetRightSiblingIndex(vertexIndex, vertexCount));
                triangles[trianglesIndex++] = GetUpSiblingIndex(vertexIndex);
            }

            vertexIndex++;
        }
    }

    // Работа с аппаратными буферами специфичная для OGRE.
    sphereMesh = MeshManager::getSingleton().createManual(meshName, RGN_DEFAULT);
    sphereMesh->_setBounds(AxisAlignedBox({ -100, -100, 0 }, { 100, 100, 0 }));

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

    delete[] vertices;
    delete[] triangles;
}

//
// Преобразует широту/долготу в вектор.
// Входные углы ожидаются в градусах.
//
Vector3f Globe3D::LatLonToCartesian(float lat, float lon, float rho) {
    float theta = 90 - lat;
    float phi = (lon > 90) ? (450 - lon) : (90 - lon); // Долготу в первой четверти обрабатываем отдельно: 360 - (lon - 90) = 450 - lon.

    return SphericalToCartesian(rho, theta * DEG_TO_RAD, phi * DEG_TO_RAD);
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
int Globe3D::GetRightSiblingIndex(int index, int vertexCount) {
    return index + PARALLELS;
}

//
// Возвращает индекс соседней вершины с верхней параллели.
//
int Globe3D::GetUpSiblingIndex(int index) {
    return index - 1;
}

//
// Создает метки долготы на каждый меридиан и привязывает к ноде.
// NOTE: Требуется дальнейшая реализация взаимодействия с камерой, по аналогии с Google Earth.
//
void Globe3D::CreateLonMarks(SceneNode* node) {
    BillboardSet* lonMarks = scnMgr->createBillboardSet("LongitudeMarks", MERIDIANS - 1); // Пропускаем доп. меридиан, введенный для коррекции текстурирования.
    lonMarks->setDefaultWidth(2.0f);
    lonMarks->setDefaultHeight(2.0f);

    float angleStep = 2 * M_PI / (MERIDIANS - 1);
    float theta = M_PI_2;
    for (int i = 0; i < MERIDIANS - 1; i++) {
        float phi = angleStep * i;

        Vector3f v = SphericalToCartesian(EARTH_RADIUS_IN_UNITS + 1, theta, phi);
        lonMarks->createBillboard(v);
    }
    node->attachObject(lonMarks);
}

//
// Создает метки широты на каждую параллель (включая точки полюсов) и привязывает к ноде.
// NOTE: Требуется дальнейшая реализация взаимодействия с камерой, по аналогии с Google Earth.
//
void Globe3D::CreateLatMarks(SceneNode* node) {
    BillboardSet* latMarks = scnMgr->createBillboardSet("LatitudeMarks", PARALLELS);
    latMarks->setDefaultWidth(2.0f);
    latMarks->setDefaultHeight(2.0f);

    float angleStep = M_PI / (PARALLELS - 1);
    float phi = M_PI_2;
    for (int i = 0; i < PARALLELS; i++) {
        float theta = angleStep * i;

        Vector3f v = SphericalToCartesian(EARTH_RADIUS_IN_UNITS + 1, theta, phi);
        latMarks->createBillboard(v);
    }
    node->attachObject(latMarks);
}

//
//
//
Globe3D::Globe3D() : ApplicationContext("Globe3D") {
}

//
//
//
Globe3D::~Globe3D() {
}
