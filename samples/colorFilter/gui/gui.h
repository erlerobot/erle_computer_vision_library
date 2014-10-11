#ifndef GUI_H
#define GUI_H

#include <QtGui>
#include "../sensors/sensors.h"
#include "../ComputerVision/colorfilter.h"
#include <QSlider>

#include <QtWidgets>

#include <iostream>

class GUI:public QWidget
{
    Q_OBJECT

public:
    GUI(Sensors* sensors);
    void updateThreadGUI();

private:
    QLabel* labelImage;
    QLabel* labelImage_filtered_hsv;
    QLabel* labelImage_filtered_rgb;
    QLabel* labelImage_hsv;
    QLabel* labelImage_rgb;
    Sensors* sensors;

    QSlider* sliderR_min;
    QSlider* sliderB_min;
    QSlider* sliderG_min;
    QSlider* sliderR_max;
    QSlider* sliderB_max;
    QSlider* sliderG_max;
    QLabel* textR_min;
    QLabel* textG_min;
    QLabel* textB_min;
    QLabel* textR_max;
    QLabel* textG_max;
    QLabel* textB_max;
    QLabel* textR_value_min;
    QLabel* textG_value_min;
    QLabel* textB_value_min;
    QLabel* textR_value_max;
    QLabel* textG_value_max;
    QLabel* textB_value_max;

    QDoubleSpinBox* sliderH_min;
    QDoubleSpinBox* sliderS_min;
    QDoubleSpinBox* sliderV_min;
    QDoubleSpinBox* sliderH_max;
    QDoubleSpinBox* sliderS_max;
    QDoubleSpinBox* sliderV_max;
    QLabel* textH_min;
    QLabel* textS_min;
    QLabel* textV_min;
    QLabel* textH_max;
    QLabel* textS_max;
    QLabel* textV_max;
    QLabel* textH_value_max;
    QLabel* textS_value_max;
    QLabel* textV_value_max;
    QLabel* textH_value_min;
    QLabel* textS_value_min;
    QLabel* textV_value_min;

    QCheckBox* checkRGB;
    QCheckBox* checkHSV;

    cv::Mat frame;
    cv::Mat frame_filtered_hsv;
    cv::Mat frame_filtered_rgb;
    cv::Mat hsv_image;
    QMutex mutex;

signals:
    void signal_updateGUI();

public slots:
    void on_updateGUI_recieved();


    void mousePressEvent(QMouseEvent *event);
};

#endif // GUI_H
