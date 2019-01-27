#include <QGuiApplication>

#include <QtQuick/QQuickView>

#include <cassert>
#include <random>
#include <chrono>
#include <iostream>
#include <cstdlib>

#include <sonar/HomographyInitializator.h>
#include <sonar/Initializator.h>

#include "test_synthetic_initialization.h"
#include "test_demo.h"

void runTests()
{
    using namespace std;
    using namespace std::chrono;
    using namespace sonar;

    srand(static_cast<unsigned int>(duration_cast<microseconds>(system_clock::now().time_since_epoch()).count()));

    system_clock::time_point start_time = system_clock::now();

    unique_ptr<AbstractInitializator> initializator(new HomographyInitializator());
    //unique_ptr<AbstractInitializator> initializator(new Initializator());

    int nTests = 100;
    int nSuccess = 0;
    for (int i = 0; i < nTests; ++i)
    {
        if (test_synthetic_initialization(initializator.get(),
                                          (dynamic_cast<HomographyInitializator*>(initializator.get()) != nullptr)))
        {
            ++nSuccess;
        }
    }
    auto end_time = system_clock::now();
    int delta_time = static_cast<int>(duration_cast<milliseconds>(end_time - start_time).count() / nTests);

    cout << "test result: " << nSuccess << " / " << nTests << " time = " << delta_time << "ms" << endl;

    assert(nSuccess >= static_cast<int>(nTests * 0.95));
}

void runDemo()
{
    assert(test_demo());
}

int main(int argc, char ** argv)
{
    QGuiApplication app(argc, argv);

    runDemo();
    //runTests();

    //return app.exec();
    return 0;
}
