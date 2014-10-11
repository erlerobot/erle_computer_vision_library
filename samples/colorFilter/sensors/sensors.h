#ifndef SENSORS_H
#define SENSORS_H
//Qt
#include <QtGui>

//Opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class Sensors
{
public:
    Sensors(std::string name);

    void update();
    cv::Mat getImage();

private:
    QMutex mutex;

    cv::Mat image;
    cv::VideoCapture cap;
};

#endif // SENSORS_H
