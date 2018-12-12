#include <QGuiApplication>

#include <QtQuick/QQuickView>

#include <cassert>

#include "test_synthetic_initialization.h"

int main(int argc, char **argv)
{
    QGuiApplication app(argc, argv);

    assert(test_synthetic_initialization());

    return app.exec();
}
