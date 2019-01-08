#include <QGuiApplication>

#include <QtQuick/QQuickView>

#include <cassert>
#include <random>
#include <chrono>
#include <iostream>

#include "test_synthetic_initialization.h"

void runTests()
{
    using namespace std;
    using namespace std::chrono;

    srand(static_cast<unsigned int>(duration_cast<microseconds>(system_clock::now().time_since_epoch()).count()));

    system_clock::time_point start_time = system_clock::now();

    int nTests = 100;
    int nSuccess = 0;
    for (int i = 0; i < nTests; ++i)
    {
        if (test_synthetic_initialization())
            ++nSuccess;
    }
    auto end_time = system_clock::now();
    int delta_time = static_cast<int>(duration_cast<milliseconds>(end_time - start_time).count() / nTests);

    cout << "test result: " << nSuccess << " / " << nTests << " time = " << delta_time << "ms" << endl;
    assert(nSuccess == nTests);
}

int main(int argc, char ** argv)
{
    QGuiApplication app(argc, argv);

    runTests();

    //return app.exec();
    return 0;
}
