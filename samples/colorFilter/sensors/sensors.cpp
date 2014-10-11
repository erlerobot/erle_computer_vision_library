#include "sensors.h"

Sensors::Sensors(std::string name ):cap(name)
{
}

void Sensors::update()
{
    mutex.lock();
    cap>> image;
    if(image.data==0){
        cap.set(CV_CAP_PROP_POS_FRAMES, 1);
        cap>> image;
    }
    mutex.unlock();

}

cv::Mat Sensors::getImage()
{
    mutex.lock();

    cv::Mat result = image.clone();
    cv::resize(result, result, cv::Size(640, 480));
    cv::cvtColor(result, result, CV_RGB2BGR);
//    cv::transpose(result, result);
//    cv::flip(result, result, 1);
//    cv::transpose(result, result);
//    cv::flip(result, result, 1);

    mutex.unlock();

    return result;
}
