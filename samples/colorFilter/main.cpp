//Qt
#include <QtGui>
#include <QApplication>

#include "gui/threadgui.h"
#include "gui/gui.h"
#include "sensors/sensors.h"
#include "sensors/threadsensors.h"

int main(int argc, char* argv[])
{
    QApplication a(argc, argv);

    Sensors* sensors = new Sensors("/Users/ahcorde/Desktop/Golpe.MP4");
    threadGUI* gui = new threadGUI(sensors);
    ThreadSensors* threadSensors = new ThreadSensors(sensors);

    gui->start();
    threadSensors->start();

    return a.exec();

}
