/*
*  Copyright (C) 2014-2014 ERLE Developers Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 *  Authors : Alejandro Hernández Cordero <ahcorde at erlerobot dot com>
 *
 */

#include "PatternDetector.h"
namespace erle_computer_vision{

    PatternDetector::PatternDetector(int IMAGE_WIDTH, int IMAGE_HEIGHT) {

        //image size
        this->IMAGE_HEIGHT = IMAGE_HEIGHT;
        this->IMAGE_WIDTH = IMAGE_WIDTH;

        //thresholds
        this->threshold1 = 13;
        this->threshold2 = 3;

        //homography sie
        this->hsize = 56;
        this->msize = 7;

        //filtered image size
        this->filtered = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);

        //Distortions
        this->camera_matrix = cv::Mat(3, 3, CV_32FC1);
        this->dist_coeffs = cv::Mat(5, 1, CV_32FC1);
        this->mapx = cv::Mat(IMAGE_WIDTH, IMAGE_HEIGHT, CV_32FC1);
        this->mapy = cv::Mat(IMAGE_WIDTH, IMAGE_HEIGHT, CV_32FC1);

        //K matrix
        this->camera_matrix.at<float>(0,0) = 524.11;
        this->camera_matrix.at<float>(0,1) = 0;
        this->camera_matrix.at<float>(0,2) = 328.383;
        this->camera_matrix.at<float>(1,0) = 0;
        this->camera_matrix.at<float>(1,1) = 541.38;
        this->camera_matrix.at<float>(1,2) = 240.418;
        this->camera_matrix.at<float>(2,0) = 0;
        this->camera_matrix.at<float>(2,1) = 0;
        this->camera_matrix.at<float>(2,2) = 1;

        std::cout << "PatternDetector:PatternDetector Camera_matrix:\n" <<  camera_matrix << std::endl;

        //Distorision Eigen::Matrix
        //    this->dist_coeffs.at<float>(0,0) = -0.420164;
        //    this->dist_coeffs.at<float>(1,0) = 0.236811;
        //    this->dist_coeffs.at<float>(2,0) = -0.00213207;
        //    this->dist_coeffs.at<float>(3,0) = 0.00205521;
        //    this->dist_coeffs.at<float>(4,0) = 0.177425;
        this->dist_coeffs.at<float>(0,0) = 0.;
        this->dist_coeffs.at<float>(1,0) = 0.;
        this->dist_coeffs.at<float>(2,0) = 0.;
        this->dist_coeffs.at<float>(3,0) = 0.;
        this->dist_coeffs.at<float>(4,0) = 0.;

        //Inicialize patterns
        this->init_patterns_bw();
    }

    PatternDetector::~PatternDetector() {}

    std::vector<TPatternPoint> * PatternDetector::getDetected() {
          return &(this->detected);
    }

    bool PatternDetector::detect(cv::Mat &src, cv::Mat &src2) {
        int index, orientation;

        //Image gray
        assert(src.type() == CV_8UC1);

        //clear candidates and detected (new image)
        this->detected.clear();
        this->candidates.clear();

        //Copy color image
        color = src2.clone();

        //filter image
        //TODO parametros para color HSV
        this->filter_bw(src2);

        //Search rectangles
        this->search_rectangles(this->filtered);

        //Find patterns inside rectangles
        this->check_rectangles(this->filtered);

        //Save detected patterns
        for(int i=0;i<this->candidates.size();i++) {
            index = (int) this->candidates[i](8,0);
            orientation = (int) this->candidates[i](9,0);
            for(int j=0;j<4;j++) {
                TPatternPoint pp;
                pp.p2D<< (double) this->candidates[i](2*j,0), (double) this->candidates[i](2*j+1,0);
                pp.p3D << this->positions3D[index]((j+orientation)%4, 0), this->positions3D[index]((j+orientation)%4, 1), this->positions3D[index]((j+orientation)%4, 2);
                this->detected.push_back(pp);
            }
        }

        if (DEBUG_CV){
            cv::Mat drawing = src2.clone();
            /*Dibujamos temporal*/
            for(int k=0;k<this->candidates.size();k++) {
                cv::Scalar color( rand()&255, rand()&255, rand()&255 );
                cv::line(drawing,cv::Point(this->candidates[k](0,0), this->candidates[k](1,0)),cv::Point(this->candidates[k](2,0), this->candidates[k](3,0)),color, 4);
                cv::line(drawing,cv::Point(this->candidates[k](2,0), this->candidates[k](3,0)),cv::Point(this->candidates[k](4,0), this->candidates[k](5,0)),color, 4);
                cv::line(drawing,cv::Point(this->candidates[k](4,0), this->candidates[k](5,0)),cv::Point(this->candidates[k](6,0), this->candidates[k](7,0)),color, 4);
                cv::line(drawing,cv::Point(this->candidates[k](6,0), this->candidates[k](7,0)),cv::Point(this->candidates[k](0,0), this->candidates[k](1,0)),color, 4);
            }
            cv::imshow("drawing", drawing);
        }
        return true;
    }

    void PatternDetector::search_rectangles(cv::Mat &src) {
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        std::vector<cv::Point> approximation;
        int image_area = IMAGE_WIDTH*2 + IMAGE_HEIGHT*2;
        bool valid;
        double dist, min_dist=image_area/200;
       Eigen::MatrixXd mcandidate;
        double lx1, ly1, lx2, ly2;

        mcandidate.resize(10,1);

        /*Find contours*/
        cv::findContours(src, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE );

        cv::Mat candidates;
        cv::cvtColor(this->filtered, candidates, CV_GRAY2BGR);

        /*Check detected contours*/
        for (int i=0;i<contours.size();i++ ) {

            /*Check contour size*/
            if(contours[i].size() < image_area/20 || contours[i].size() > image_area/2)
                continue;

            /*Approximate contour and check if it is a parallelogram*/
            cv::approxPolyDP(contours[i], approximation, double(contours[i].size())*0.1, true); /*TODO: Probar otros valores distintos de 0.1*/
            if(!(approximation.size()==4))
                continue;

            /*Check if it is convex*/
            if(!cv::isContourConvex(cv::Mat(approximation)))
                continue;

            /*Check distance between points*/
            valid = true;
            for(int j=0;j<4 && valid;j++) {
                dist = sqrt( pow(approximation[j].x -  approximation[(j+1)%4].x, 2) + pow(approximation[j].y - approximation[(j+1)%4].y, 2) );
                if(dist < min_dist)
                valid = false;
            }
            if(!valid)
                continue;

            std::vector<cv::Point2f> cornerBuf;
            for(int i=0; i< approximation.size(); i++){
                cornerBuf.push_back(cv::Point2f(approximation[i].x,approximation[i].y ));
            }

            cv::cornerSubPix( this->filtered, cornerBuf, cv::Size(5,5),
                              cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

            /*Check points orientation and save it in clockwise order*/
            lx1 = cornerBuf[1].x - cornerBuf[0].x;
            ly1 = cornerBuf[1].y - cornerBuf[0].y;
            lx2 = cornerBuf[2].x - cornerBuf[0].x;
            ly2 = cornerBuf[2].y - cornerBuf[0].y;
            if((lx1*ly2)-(ly1*lx2) >= 0.0)  {
                mcandidate << cornerBuf[0].x, cornerBuf[0].y, cornerBuf[1].x, cornerBuf[1].y,
                              cornerBuf[2].x, cornerBuf[2].y, cornerBuf[3].x, cornerBuf[3].y, -1, -1;
            } else {
                mcandidate << cornerBuf[0].x, cornerBuf[0].y, cornerBuf[3].x, cornerBuf[3].y,
                              cornerBuf[2].x, cornerBuf[2].y, cornerBuf[1].x, cornerBuf[1].y, -1, -1;
            }

            cv::Scalar color(255, 255, 0);
            cv::drawContours(candidates, contours, i, color, 2, 8, hierarchy);

            /*Save approximation*/
            this->candidates.push_back(mcandidate);
        }
        cv::imshow("candidates", candidates);
    }

    void PatternDetector::check_rectangles(cv::Mat &src) {
        Eigen::MatrixXd cmatrix;
        Eigen::MatrixXd hmatrix;
        Eigen::MatrixXd bitmatrix;
        cv::Mat hom, havg;
        cv::Point2f points1[4], points2[4];
        int msize = 7;
        int step=hsize/msize;
        int mcolor, color, index=0;
        bool valid;
        int val, orientation;

        havg = cv::Mat::zeros(cv::Size(5,5), CV_8UC1);
        hmatrix.resize(8,1);
        bitmatrix.resize(5,5);

        hmatrix << 0, 0, hsize-1, 0, hsize-1, hsize-1, 0, hsize-1;
        points2[0] = cv::Point2f(hmatrix(0,0),hmatrix(1,0));
        points2[1] = cv::Point2f(hmatrix(2,0),hmatrix(3,0));
        points2[2] = cv::Point2f(hmatrix(4,0),hmatrix(5,0));
        points2[3] = cv::Point2f(hmatrix(6,0),hmatrix(7,0));

        std::vector<Eigen::MatrixXd>::iterator it = this->candidates.begin();
        while(it!=this->candidates.end()) {
            cmatrix = (*it);

            points1[0] = cv::Point2f(cmatrix(0,0),cmatrix(1,0));
            points1[1] = cv::Point2f(cmatrix(2,0),cmatrix(3,0));
            points1[2] = cv::Point2f(cmatrix(4,0),cmatrix(5,0));
            points1[3] = cv::Point2f(cmatrix(6,0),cmatrix(7,0));

            /*Create homography*/
            cv::Mat pers = cv::getPerspectiveTransform(points1, points2);
            cv::warpPerspective(src, hom, pers, cv::Size(hsize, hsize), cv::INTER_NEAREST);

            /*Borders with the same color*/
            valid = true;
            for(int row=0;row<this->msize && valid;row++) {
                for(int col=0;col<this->msize && valid;col++) {
                    /*Check only borders*/
                    if(row!=0 && row!=msize-1 && col!=0 && col!=msize-1)
                        continue;

                    /*Save first color*/
                    if(row==0 && col==0)
                        mcolor = this->get_color(hom, row, col, step);
                    else {
                        /*Compare with first color*/
                        color = this->get_color(hom, row, col, step);
                        valid = (color == mcolor);
                    }
                }
            }

            /*Delete not valid*/
            if(!valid) {
                it = this->candidates.erase(it);
                continue;
            }

            /*Create average from homography*/
            for(int row=1;row<this->msize-1 && valid;row++) {
                for(int col=1;col<this->msize-1 && valid;col++) {
                    color = this->get_color(hom, row, col, step);
                    if(color != mcolor) {
                        havg.at<uchar>(col-1,row-1) = 1*255;
                        bitmatrix(col-1,row-1) = 1;
                    } else {
                        havg.at<uchar>(col-1,row-1) = 0;
                        bitmatrix(col-1,row-1) = 0;
                    }
                }
            }

            /*Compare to saved patterns for each orientation*/
            val = -1;
            for(int rot=0;rot<4 && val<0;rot++) {
                val = this->search_pattern(bitmatrix);
                orientation = rot;
                this->turn_pattern_right(bitmatrix);
            }

            if(val < 0) {
                it = this->candidates.erase(it);
                continue;
            }

            /*
             *      stringstream ss;
            index++;
            ss << "homografia" << index << ".png";
            cv::imwrite(ss.str(), hom);
            ss << ".res.png";
            cv::imwrite(ss.str(), havg);

            cout << "para el index " << index << " la matrix es " << endl << bitmatrix << endl;
            cout << "res es " << val << endl;
              */
            /*Save detected pattern*/
            (*it)(8,0) = (double) val;
            (*it)(9,0) = (double) orientation;

            it++;
        }

    }

    int PatternDetector::get_color(cv::Mat &src, int row, int col, int size) {
        int rs, cs, tot, max;

        rs = row*size;
        cs = col*size;

        /*Count how many rectangles are zero*/
        cv::Mat box = src(cv::Rect(rs,cs,size,size));
        tot = cv::countNonZero(box);

        /*Calc average*/
        max = (size*size)/2;
        if (tot > max)
            return 1;
        else
            return 0;
    }

    void PatternDetector::filter(cv::Mat &src) {
        /*Filter img, check H and S*/
        for(int i=0;i<IMAGE_WIDTH;i++) {
            for(int j=0;j<IMAGE_HEIGHT;j++) {
                cv::Vec3b pixel = src.at<cv::Vec3b>(j,i);
                if((int)(unsigned int)pixel[0] < (255*3.0/6.28) && (int)(unsigned int)pixel[1] < (255*0.55))
                    this->filtered.at<uchar>(j, i) = 1*255;
                else
                    this->filtered.at<uchar>(j, i) = 0;
            }
        }
    }

    void PatternDetector::filter_bw(cv::Mat &src) {
        for(int i=0;i<IMAGE_WIDTH;i++) {
            for(int j=0;j<IMAGE_HEIGHT;j++) {
                cv::Vec3b pixel = src.at<cv::Vec3b>(j,i);
                if((int)(unsigned int)pixel[2] > (255*135.0/255.0))
                    this->filtered.at<uchar>(j, i) = 1*255;
                else
                    this->filtered.at<uchar>(j, i) = 0;
            }
        }
    }

    int PatternDetector::search_pattern(Eigen::MatrixXd p) {
        int size = this->patterns.size();
        bool cancel[size];
        bool found = true;

        memset(cancel, 0, size*sizeof(bool));
        for(int i=0;i<5 && found;i++) {
            for(int j=0;j<5 && found;j++) {
                found = false;
                /*Compare with patterns*/
                int k=0;
                for(std::vector<Eigen::MatrixXd>::iterator it = this->patterns.begin(); it!= this->patterns.end(); it++) {
                    if(cancel[k]==0) {
                        if(p(i,j) == (*it)(i,j))
                            found = true;
                        else
                            cancel[k] = true;
                    }
                    k++;
                }
            }
        }
        /*Get found pattern*/
        if(found) {
            for(int i=0;i<size;i++) {
                if(cancel[i]==0)
                    return i;
            }
        }
        return -1;
    }

    void  PatternDetector::turn_pattern_right(Eigen::MatrixXd &pin) {
        Eigen::MatrixXd mcol;
        int i2;

        mcol.resize(5,1);

        pin.transposeInPlace();

        /*Swap colums*/
        for(int i=0;i<2;i++) {
            if(i==0) i2=4;
            if(i==1) i2=3;

            mcol = pin.block(0,i,5,1);
            pin.block(0,i,5,1) = pin.block(0,i2,5,1);
            pin.block(0,i2,5,1) = mcol;
        }
    }

    cv::Mat PatternDetector::rectify_image(cv::Mat src) {
        cv::initUndistortRectifyMap(this->camera_matrix, this->dist_coeffs, cv::Mat(), this->camera_matrix, src.size(), CV_32FC1, this->mapx, this->mapy);
        cv::remap(src, src, this->mapx, this->mapy, cv::INTER_LINEAR);
        return src;
    }

    void PatternDetector::init_patterns_bw()
    {
        Eigen::MatrixXd pattern;
        Eigen::MatrixXd position;

        // 1a Fila
        // 1-1
        pattern = Eigen::MatrixXd(5, 5);
        pattern << 0,1,1,1,0, 0,1,1,1,0, 1,0,1,1,1, 0,1,0,0,1, 1,0,1,1,1;
        this->patterns.push_back(pattern);

        position = Eigen::MatrixXd(4, 3);
        position << 0.0897,    0,  0.1365  ,
                   0.0507 ,    0, 0.1365  ,
                   0.0507 ,    0, 0.0975  ,
                   0.0897  ,    0, 0.0975  ;

        this->positions3D.push_back(position);

        // 1-2
        pattern = Eigen::MatrixXd(5, 5);
        pattern << 0,1,1,1,0, 1,0,1,1,1, 1,0,0,0,0, 1,0,1,1,1, 0,1,0,0,1;
        this->patterns.push_back(pattern);

        position = Eigen::MatrixXd(4, 3);
        position << 0.0429 ,       0,  0.1365  ,
                   0.0039 ,       0,  0.1365  ,
                   0.0039 ,       0,  0.0975  ,
                   0.0429 ,       0,  0.0975;
        this->positions3D.push_back(position);

        //1-3
        pattern = Eigen::MatrixXd(5, 5);
        pattern << 0,1,1,1,0, 0,1,0,0,1, 1,0,0,0,0, 0,1,1,1,0, 1,0,0,0,0;
        this->patterns.push_back(pattern);

        position = Eigen::MatrixXd(4, 3);
        position << -0.0039 ,       0, 0.1365 ,
                    -0.0429  ,       0, 0.1365    ,
                    -0.0429 ,       0,  0.0975    ,
                    -0.0039 ,       0,  0.0975    ;
        this->positions3D.push_back(position);

        //1-4
        pattern = Eigen::MatrixXd(5, 5);
        pattern << 1,0,1,1,1, 1,0,0,0,0, 0,1,0,0,1, 0,1,0,0,1, 0,1,1,1,0;
        this->patterns.push_back(pattern);

        position = Eigen::MatrixXd(4, 3);
        position << -0.0507, 0,   0.1365,
                -0.0897,   0, 0.1365     ,
                -0.0897,   0, 0.0975     ,
                -0.0507,   0, 0.0975     ;
        this->positions3D.push_back(position);

        //2 Fila
        //2-1
        pattern = Eigen::MatrixXd(5, 5);
        pattern << 1,0,1,1,1, 0,1,0,0,1, 0,1,0,0,1, 0,1,1,1,0, 1,0,0,0,0;
        this->patterns.push_back(pattern);

        position = Eigen::MatrixXd(4, 3);
        position <<    0.0897 ,  0, 0.0897    ,
                       0.0507  , 0, 0.0897    ,
                       0.0507 ,  0, 0.0507    ,
                       0.0897 ,  0, 0.0507    ;
        this->positions3D.push_back(position);

        //2-2
        pattern = Eigen::MatrixXd(5, 5);
        pattern << 1,0,0,0,0, 0,1,0,0,1, 0,1,1,1,0, 1,0,0,0,0, 1,0,1,1,1;
        this->patterns.push_back(pattern);

        position = Eigen::MatrixXd(4, 3);
        position <<    0.0429 ,  0,  0.0897   ,
                   0.0039  ,   0,0.0897   ,
                   0.0039 ,   0, 0.0507   ,
                   0.0429  ,  0, 0.0507   ;
        this->positions3D.push_back(position);

        //2-3
        pattern = Eigen::MatrixXd(5, 5);
        pattern << 1,0,0,0,0, 1,0,1,1,1, 1,0,0,0,0, 1,0,0,0,0, 1,0,0,0,0;
        this->patterns.push_back(pattern);

        position = Eigen::MatrixXd(4, 3);
        position << -0.0039 ,   0, 0.0897  ,
                -0.0429  ,   0,0.0897     ,
                -0.0429  ,  0,0.0507    ,
                -0.0039  ,  0,0.0507    ;
        this->positions3D.push_back(position);

        //2-4
        pattern = Eigen::MatrixXd(5, 5);
        pattern << 1,0,1,1,1, 1,0,1,1,1, 1,0,1,1,1, 1,0,1,1,1, 1,0,1,1,1;
        this->patterns.push_back(pattern);

        position = Eigen::MatrixXd(4, 3);
        position << -0.0507 ,  0,  0.0897 ,
                    -0.0897 ,  0,  0.0897   ,
                    -0.0897 ,  0,  0.0507   ,
                    -0.0507 ,  0,  0.0507 ;
        this->positions3D.push_back(position);

        //3a fila
        //3-1
        pattern = Eigen::MatrixXd(5, 5);
        pattern << 0,1,0,0,1, 0,1,1,1,0, 0,1,1,1,0, 0,1,0,0,1, 1,0,0,0,0;
        this->patterns.push_back(pattern);

        position = Eigen::MatrixXd(4, 3);
        position <<  0.0897, 0,  0.0429  ,
                   0.0507 , 0, 0.0429  ,
                   0.0507 , 0, 0.0039  ,
                   0.0897 , 0, 0.0039  ;
        this->positions3D.push_back(position);

        //3-2
        pattern = Eigen::MatrixXd(5, 5);
        pattern <<0,1,1,1,0, 1,0,1,1,1, 0,1,1,1,0, 1,0,0,0,0, 0,1,0,0,1;
        this->patterns.push_back(pattern);

        position = Eigen::MatrixXd(4, 3);
        position << 0.0429  ,0, 0.0429    ,
                   0.0039  , 0,0.0429   ,
                   0.0039 , 0, 0.0039  ,
                   0.0429 , 0, 0.0039   ;
        this->positions3D.push_back(position);

        //3-3
        pattern = Eigen::MatrixXd(5, 5);
        pattern << 0,1,1,1,0, 0,1,1,1,0, 1,0,1,1,1, 1,0,1,1,1, 0,1,0,0,1;
        this->patterns.push_back(pattern);

        position = Eigen::MatrixXd(4, 3);
        position << -0.0039,  0, 0.0429,
                -0.0429,  0, 0.0429  ,
                -0.0429 , 0, 0.0039  ,
                -0.0039 , 0, 0.0039 ;
        this->positions3D.push_back(position);

        //3-4
        pattern = Eigen::MatrixXd(5, 5);
        pattern <<0,1,1,1,0, 0,1,1,1,0, 1,0,1,1,1, 1,0,0,0,0, 1,0,1,1,1;
        this->patterns.push_back(pattern);

        position = Eigen::MatrixXd(4, 3);
        position << -0.0507  ,0, 0.0429  ,
                -0.0897  , 0,0.0429    ,
                -0.0897  , 0,0.0039    ,
                -0.0507  , 0,0.0039    ;
        this->positions3D.push_back(position);

        //4a fila
        //4-1
        pattern = Eigen::MatrixXd(5, 5);
        pattern << 1,0,1,1,1, 0,1,1,1,0, 1,0,1,1,1, 0,1,1,1,0, 1,0,1,1,1;
        this->patterns.push_back(pattern);

        position = Eigen::MatrixXd(4, 3);
        position << 0.0897  ,  0.0039 ,        0,
                    0.0507  ,  0.0039    ,     0,
                    0.0507  ,  0.0429   ,      0,
                    0.0897  ,  0.0429   ,      0;
        this->positions3D.push_back(position);

        //4-2
        pattern = Eigen::MatrixXd(5, 5);
        pattern << 1,0,0,0,0, 1,0,1,1,1, 0,1,1,1,0, 0,1,1,1,0, 1,0,1,1,1;
        this->patterns.push_back(pattern);

        position = Eigen::MatrixXd(4, 3);
        position <<  0.0429  ,  0.0039   ,      0,
                   0.0039 ,   0.0039    ,     0,
                   0.0039 ,   0.0429    ,     0,
                   0.0429  ,  0.0429    ,     0;
        this->positions3D.push_back(position);

        //4-3
        pattern = Eigen::MatrixXd(5, 5);
        pattern << 0,1,0,0,1, 0,1,1,1,0, 1,0,0,0,0, 0,1,1,1,0, 1,0,1,1,1;
        this->patterns.push_back(pattern);

        position = Eigen::MatrixXd(4, 3);
        position << -0.0039 ,   0.0039  ,       0,
                -0.0429  ,  0.0039     ,    0,
                -0.0429  ,  0.0429    ,     0,
                -0.0039  ,  0.0429      ,   0;
        this->positions3D.push_back(position);

        //4-4
        pattern = Eigen::MatrixXd(5, 5);
        pattern << 0,1,1,1,0, 1,0,0,0,0, 1,0,1,1,1, 1,0,1,1,1, 0,1,1,1,0;
        this->patterns.push_back(pattern);

        position = Eigen::MatrixXd(4, 3);
        position <<-0.0507 ,   0.0039  ,       0,
                -0.0897 ,   0.0039   ,      0,
                -0.0897  ,  0.0429   ,      0,
                -0.0507  ,  0.0429   ,      0;
        this->positions3D.push_back(position);

        //5a fila
        //5-1
        pattern = Eigen::MatrixXd(5, 5);
        pattern << 0,1,0,0,1, 1,0,1,1,1, 0,1,0,0,1, 0,1,0,0,1, 0,1,0,0,1;
        this->patterns.push_back(pattern);

        position = Eigen::MatrixXd(4, 3);
        position << 0.0897  ,  0.0507  ,       0,
                    0.0507  ,  0.0507  ,       0,
                    0.0507   , 0.0897 ,        0,
                    0.0897    ,0.0897,         0;
        this->positions3D.push_back(position);

        //5-2
        pattern = Eigen::MatrixXd(5, 5);
        pattern << 1,0,0,0,0, 1,0,1,1,1, 1,0,0,0,0, 0,1,1,1,0, 1,0,0,0,0;
        this->patterns.push_back(pattern);

        position = Eigen::MatrixXd(4, 3);
        position << 0.0429 ,   0.0507  ,       0,
                   0.0039  ,  0.0507   ,      0,
                   0.0039  ,  0.0897   ,      0,
                   0.0429  ,  0.0897   ,      0;
        this->positions3D.push_back(position);

        //5-3
        pattern = Eigen::MatrixXd(5, 5);
        pattern << 1,0,0,0,0, 0,1,0,0,1, 0,1,1,1,0, 1,0,1,1,1, 1,0,1,1,1;
        this->patterns.push_back(pattern);

        position = Eigen::MatrixXd(4, 3);
        position << -0.0039   , 0.0507    ,     0,
                -0.0429  ,  0.0507      ,   0,
                -0.0429  ,  0.0897     ,    0,
                -0.0039  ,  0.0897      ,   0;
        this->positions3D.push_back(position);

        //5-4
        pattern = Eigen::MatrixXd(5, 5);
        pattern << 0,1,1,1,0, 0,1,1,1,0, 0,1,0,0,1, 0,1,1,1,0, 1,0,1,1,1;
        this->patterns.push_back(pattern);

        position = Eigen::MatrixXd(4, 3);
        position << -0.0507  ,  0.0507,         0,
                -0.0897  ,  0.0507   ,      0,
                -0.0897   , 0.0897   ,      0,
                -0.0507  ,  0.0897   ,      0;
        this->positions3D.push_back(position);

        //6a fila
        //6-1
        pattern = Eigen::MatrixXd(5, 5);
        pattern << 1,0,0,0,0, 0,1,0,0,1, 0,1,0,0,1, 0,1,1,1,0, 0,1,1,1,0;
        this->patterns.push_back(pattern);

        position = Eigen::MatrixXd(4, 3);
        position << 0.0897   , 0.0975    ,     0,
                   0.0507  ,  0.0975     ,    0,
                   0.0507   , 0.1365     ,    0,
                   0.0897   , 0.1365     ,    0;
        this->positions3D.push_back(position);

        //6-2
        pattern = Eigen::MatrixXd(5, 5);
        pattern << 0,1,0,0,1, 0,1,0,0,1, 0,1,0,0,1, 0,1,1,1,0, 1,0,0,0,0;
        this->patterns.push_back(pattern);

        position = Eigen::MatrixXd(4, 3);
        position << 0.0429 ,   0.0975    ,    0,
                   0.0039  ,  0.0975     ,    0,
                   0.0039  ,  0.1365    ,     0,
                   0.0429  ,  0.1365     ,    0;
        this->positions3D.push_back(position);

        //6-3
        pattern = Eigen::MatrixXd(5, 5);
        pattern << 1,0,0,0,0, 0,1,1,1,0, 0,1,0,0,1, 0,1,0,0,1, 1,0,1,1,1;
        this->patterns.push_back(pattern);

        position = Eigen::MatrixXd(4, 3);
        position << -0.0039 ,   0.0975,         0,
                -0.0429  ,  0.0975   ,      0,
                -0.0429  ,  0.1365   ,      0,
                -0.0039  ,  0.1365   ,      0;
        this->positions3D.push_back(position);

        //6-4
        pattern = Eigen::MatrixXd(5, 5);
        pattern << 1,0,1,1,1, 0,1,1,1,0, 1,0,0,0,0, 0,1,1,1,0, 1,0,1,1,1;
        this->patterns.push_back(pattern);

        position = Eigen::MatrixXd(4, 3);
        position << -0.0507 ,   0.0975 ,        0,
                -0.0897 ,   0.0975    ,     0,
               - 0.0897 ,   0.1365  ,       0,
               - 0.0507 ,   0.1365   ,      0;
        this->positions3D.push_back(position);
    }
}

