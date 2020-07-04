#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include "image.hpp"
#include "scene.hpp"
#include <ctime>
#include <string>

using namespace std;

int main(int argc, char* argv[])
{
    for (int argNum = 1; argNum < argc; ++argNum) {
        std::cout << "Argument " << argNum << " is: " << argv[argNum] << std::endl;
    }

    if (argc != 3) {
        cout << "Usage: <input scene file> <output bmp file>" << endl;
        return 1;
    }
    string inputFile = argv[1];
    string outputFile = argv[2]; // only bmp is allowed.
    clock_t start_time = clock();

    Scene* scene = new Scene(inputFile.c_str());
    scene->Run();
    scene->getImage()->SaveBMP(outputFile.c_str());
    delete scene;
    clock_t end_time = clock();
    printf("Escaped time: %.2lf s\n", (double)(end_time - start_time) / CLOCKS_PER_SEC);
    return 0;
}
