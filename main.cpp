#include <iostream>

#include "Globe3D.h"

int main(int argc, char* argv[])
{
    try {
        Globe3D app;
        app.initApp();
        app.getRoot()->startRendering();
        app.closeApp();
    }
    catch (const std::exception& e) {
        std::cerr << "Error occurred during execution: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
