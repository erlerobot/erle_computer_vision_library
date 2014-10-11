#include "gui.h"

GUI::GUI(Sensors* sensors)
{

    this->sensors = sensors;

    QGridLayout* mainLayout = new QGridLayout();

    labelImage = new QLabel();
    labelImage_filtered_hsv = new QLabel();
    labelImage_filtered_rgb = new QLabel();
    labelImage_hsv = new QLabel();
    labelImage_rgb = new QLabel();

    QGroupBox* boxRGB = new QGroupBox("RGB");
    QGroupBox* boxHSV = new QGroupBox("HSV");
    QGridLayout* layRGB = new QGridLayout();
    QGridLayout* layHSV = new QGridLayout();

    boxRGB->setLayout(layRGB);
    boxHSV->setLayout(layHSV);

    sliderR_min = new QSlider();
    sliderB_min = new QSlider();
    sliderG_min = new QSlider();
    sliderR_max = new QSlider();
    sliderB_max = new QSlider();
    sliderG_max = new QSlider();
    textR_min = new QLabel("R min");
    textG_min = new QLabel("G min");
    textB_min = new QLabel("B min");
    textR_max = new QLabel("R max");
    textG_max = new QLabel("G max");
    textB_max = new QLabel("B max");
    textR_value_min = new QLabel();
    textG_value_min = new QLabel();
    textB_value_min = new QLabel();
    textR_value_max = new QLabel();
    textG_value_max = new QLabel();
    textB_value_max = new QLabel();
    sliderR_min->setMaximum(255);
    sliderG_min->setMaximum(255);
    sliderB_min->setMaximum(255);
    sliderR_max->setMaximum(255);
    sliderG_max->setMaximum(255);
    sliderB_max->setMaximum(255);

    layRGB->addWidget(textR_max, 0, 0);
    layRGB->addWidget(textR_min, 0, 1);
    layRGB->addWidget(textG_max, 0, 2);
    layRGB->addWidget(textG_min, 0, 3);
    layRGB->addWidget(textB_max, 0, 4);
    layRGB->addWidget(textB_min, 0, 5);
    layRGB->addWidget(sliderR_max, 1, 0);
    layRGB->addWidget(sliderR_min, 1, 1);
    layRGB->addWidget(sliderG_max, 1, 2);
    layRGB->addWidget(sliderG_min, 1, 3);
    layRGB->addWidget(sliderB_max, 1, 4);
    layRGB->addWidget(sliderB_min, 1, 5);
    layRGB->addWidget(textR_value_max, 2, 0);
    layRGB->addWidget(textR_value_min, 2, 1);
    layRGB->addWidget(textG_value_max, 2, 2);
    layRGB->addWidget(textG_value_min, 2, 3);
    layRGB->addWidget(textB_value_max, 2, 4);
    layRGB->addWidget(textB_value_min, 2, 5);

    sliderH_min = new QDoubleSpinBox();
    sliderS_min= new QDoubleSpinBox();
    sliderV_min = new QDoubleSpinBox();
    textH_min = new QLabel("H min");
    textS_min = new QLabel("S min");
    textV_min = new QLabel("V min");
    sliderH_max = new QDoubleSpinBox();
    sliderS_max= new QDoubleSpinBox();
    sliderV_max = new QDoubleSpinBox();
    textH_max = new QLabel("H max");
    textS_max = new QLabel("S max");
    textV_max = new QLabel("V max");
    textH_value_max = new QLabel();
    textS_value_max = new QLabel();
    textV_value_max = new QLabel();
    textH_value_min = new QLabel();
    textS_value_min = new QLabel();
    textV_value_min = new QLabel();

    sliderH_min->setMaximum(6.29);
    sliderH_max->setMaximum(6.29);
    sliderS_min->setMaximum(1.1);
    sliderS_max->setMaximum(1);
    sliderV_max->setMaximum(255.1);
    sliderV_min->setMaximum(255);

    layHSV->addWidget(textH_max, 0, 0);
    layHSV->addWidget(textH_min, 0, 1);
    layHSV->addWidget(textS_max, 0, 2);
    layHSV->addWidget(textS_min, 0, 3);
    layHSV->addWidget(textV_max, 0, 4);
    layHSV->addWidget(textV_min, 0, 5);
    layHSV->addWidget(sliderH_max, 1, 0);
    layHSV->addWidget(sliderH_min, 1, 1);
    layHSV->addWidget(sliderS_max, 1, 2);
    layHSV->addWidget(sliderS_min, 1, 3);
    layHSV->addWidget(sliderV_max, 1, 4);
    layHSV->addWidget(sliderV_min, 1, 5);
    layHSV->addWidget(textH_value_max, 2, 0);
    layHSV->addWidget(textH_value_min, 2, 1);
    layHSV->addWidget(textS_value_max, 2, 2);
    layHSV->addWidget(textS_value_min, 2, 3);
    layHSV->addWidget(textV_value_max, 2, 4);
    layHSV->addWidget(textV_value_min, 2, 5);


    mainLayout->addWidget(labelImage, 0, 0);
    mainLayout->addWidget(labelImage_filtered_rgb, 0, 1);
    mainLayout->addWidget(labelImage_filtered_hsv, 0, 2);
    mainLayout->addWidget(labelImage_hsv, 2, 2);
    mainLayout->addWidget(labelImage_rgb, 2, 1);
    mainLayout->addWidget(boxRGB, 1, 1);
    mainLayout->addWidget(boxHSV, 1, 2);

    setLayout(mainLayout);

    setVisible(true);

    hsv_image = ColorFilter::draw_hsvmap(320);

    connect(this, SIGNAL(signal_updateGUI()), this, SLOT(on_updateGUI_recieved()));

    show();
}

void GUI::updateThreadGUI()
{
    emit signal_updateGUI();
}


void GUI::on_updateGUI_recieved()
{
    mutex.lock();
    frame = this->sensors->getImage();

    frame.copyTo(frame_filtered_rgb);
    frame.copyTo(frame_filtered_hsv);

    cv::Mat hsv_image_temp;
    cv::Mat rgb_image_temp;
    hsv_image.copyTo(hsv_image_temp);
    hsv_image.copyTo(rgb_image_temp);

//    frame_filtered_rgb = ColorFilter::filterRGB(frame, rgb_image_temp,
//                                                sliderR_max->value(),
//                                                sliderR_min->value(),
//                                                sliderG_max->value(),
//                                                sliderG_min->value(),
//                                                sliderB_max->value(),
//                                                sliderB_min->value());
//    frame.copyTo(frame_filtered_hsv);
//    frame_filtered_hsv = ColorFilter::filterHSV(frame, hsv_image_temp,
//                                            sliderH_max->value(),
//                                            sliderH_min->value(),
//                                            sliderS_max->value(),
//                                            sliderS_min->value(),
//                                            sliderV_max->value(),
//                                            sliderV_min->value());

    ColorFilter::filterHSV_RGB(frame,frame_filtered_rgb, frame_filtered_hsv, rgb_image_temp, hsv_image_temp,
                               sliderH_max->value(),
                               sliderH_min->value(),
                               sliderS_max->value(),
                               sliderS_min->value(),
                               sliderV_max->value(),
                               sliderV_min->value(),
                               sliderR_max->value(),
                               sliderR_min->value(),
                               sliderG_max->value(),
                               sliderG_min->value(),
                               sliderB_max->value(),
                               sliderB_min->value());


    hsv_image_temp = ColorFilter::drawcheese(hsv_image_temp,
                                             sliderH_max->value(),
                                             sliderH_min->value(),
                                             sliderS_max->value(),
                                             sliderS_min->value());

    QImage imageQt = QImage((const unsigned char*)(frame.data),
                            frame.cols,
                            frame.rows,
                            frame.step,
                            QImage::Format_RGB888);
    mutex.unlock();

    QImage imageQt_filtered_rgb = QImage((const unsigned char*)(frame_filtered_rgb.data),
                                        frame.cols,
                                        frame.rows,
                                        frame.step,
                                        QImage::Format_RGB888);

    QImage imageQt_filtered_hsv = QImage((const unsigned char*)(frame_filtered_hsv.data),
                                        frame.cols,
                                        frame.rows,
                                        frame.step,
                                        QImage::Format_RGB888);
    labelImage->setPixmap(QPixmap::fromImage(imageQt));
    labelImage_filtered_rgb->setPixmap(QPixmap::fromImage(imageQt_filtered_rgb));
    labelImage_filtered_hsv->setPixmap(QPixmap::fromImage(imageQt_filtered_hsv));

    textR_value_max->setText(QString("%1").arg(sliderR_max->value()));
    textG_value_max->setText(QString("%1").arg(sliderG_max->value()));
    textB_value_max->setText(QString("%1").arg(sliderB_max->value()));
    textR_value_min->setText(QString("%1").arg(sliderR_min->value()));
    textG_value_min->setText(QString("%1").arg(sliderG_min->value()));
    textB_value_min->setText(QString("%1").arg(sliderB_min->value()));

    QImage imageQt_hsv = QImage((const unsigned char*)(hsv_image_temp.data),
                            hsv_image_temp.cols,
                            hsv_image_temp.rows,
                            hsv_image_temp.step,
                            QImage::Format_RGB888);
    labelImage_hsv->setPixmap(QPixmap::fromImage(imageQt_hsv));

    QImage imageQt_rgb = QImage((const unsigned char*)(rgb_image_temp.data),
                            rgb_image_temp.cols,
                            rgb_image_temp.rows,
                            rgb_image_temp.step,
                            QImage::Format_RGB888);
    labelImage_rgb->setPixmap(QPixmap::fromImage(imageQt_rgb));
}

void GUI::mousePressEvent(QMouseEvent *event)
{
    std::cout << "Click!" <<std::endl;
    QPoint p = labelImage->mapFromParent(QPoint(event->x(), event->y() ));

    if(p.x()>0 && p.x()<frame.cols && p.y()>0 && p.y()<frame.rows ){
        std::cout << "Click ok!" <<std::endl;

        mutex.lock();
        if(frame.data!=0){
            std::cout << "Click mutex!" <<std::endl;

            int indice = frame.step*p.y()+frame.channels()*p.x();
            int r =frame.data[indice];
            int g =frame.data[indice+1];
            int b =frame.data[indice+2];

            sliderR_max->setValue(r+20);
            sliderG_max->setValue(g+20);
            sliderB_max->setValue(b+20);

            sliderR_min->setValue(r-20);
            sliderG_min->setValue(g-20);
            sliderB_min->setValue(b-20);

            double h = ColorFilter::getH(r, g, b);
            double s = ColorFilter::getS(r, g, b);
            double v = ColorFilter::getV(r, g, b);

            sliderH_max->setValue( h*DEGTORAD + 0.2);
            sliderS_max->setValue(s + 0.1);
            sliderV_max->setValue(v+50);

            sliderH_min->setValue( h*DEGTORAD - 0.2);
            sliderS_min->setValue(s - 0.1);
            sliderV_min->setValue(v-50);
        }
        mutex.unlock();
    }
}

