#ifndef PATTERNDETECTOR_H
#define PATTERNDETECTOR_H

//Standard headers
#include <iostream>
#include <sys/time.h>

//Opencv headers
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//Eigen headers
#include <Eigen/Dense>

namespace erle_computer_vision{

    typedef struct {
        Eigen::Vector2d p2D;
        Eigen::Vector3d p3D;
        int pattern_id;
    } TPatternPoint;

    class PatternDetector {
    public:

        /**
         * @brief Contructor PatternDetector
         * @param IMAGE_WIDTH image width
         * @param IMAGE_HEIGHT image height
         */
        public: PatternDetector(int IMAGE_WIDTH, int IMAGE_HEIGHT);

        /**
         * @brief Destructor
         */
        public: ~PatternDetector();

        /**
         * @brief Get detected patterns
         * @return Get detected patterns
         */
        public: std::vector<TPatternPoint>* getDetected();

        /**
         * @brief Detect patterns
         * @param src Gray image
         * @param src2  Color image
         * @return
         */
        public: bool detect(cv::Mat &src, cv::Mat &src2);

        /**
         * @brief delete distorsions image
         * @param src
         * @return Restore image
         */
        public: cv::Mat rectify_image(cv::Mat src);

    private:
        /**
         * @brief Search valid rectangles in image
         * @param src
         */
        private: void search_rectangles(cv::Mat &src);

        /**
         * @brief Search valid patterns from rectangles
         * @param src
         */
        void check_rectangles(cv::Mat &src);

        /**
         * @brief Get average color (white or black)
         * @param src rectangle with the pattern
         * @param row number of rows
         * @param col number of cols
         * @param size Size
         * @return 1 when white 0 when black
         */
        int get_color(cv::Mat &src, int row, int col, int size);

        /**
         * @brief Filter color image using HSV filter
         * @param src
         */

        void filter(cv::Mat &src);
        /**
         * @brief filter graye scale image. Check V and S (HSV)
         * @param src
         */
        void filter_bw(cv::Mat &src);

        /**
         * @brief initialice patterns
         */
        void init_patterns_bw();

        /**
         * @brief Search a pattern from a homography
         * @param p
         * @return
         */
        int search_pattern(Eigen::MatrixXd p);

        /**
         * @brief turn_pattern_right
         * @param pin
         */
        void turn_pattern_right(Eigen::MatrixXd &pin);

//Variables
        private: std::vector<Eigen::MatrixXd> positions3D;           ///< 3D positions of the square corner

        private: std::vector<Eigen::MatrixXd> candidates;            ///< Posible candidates
        private: std::vector<Eigen::MatrixXd> patterns;              ///< Pattern codify using 0 (black) and 1 (white)
        private: std::vector<TPatternPoint> detected;         ///< Detected patterns 2D position in the image, 3D position and ID

        private: double threshold1;                      ///< Lower Threashold to filter the image
        private: double threshold2;                      ///< higher threashold to filter the image

        private: cv::Mat filtered;                       ///< filtered image (GRAY)
        private: cv::Mat color;                          ///< copy of the image (RGB)

        private: int hsize;                              ///< Homography width and height
        private: int msize;                              ///< Pattern size

        /*Distortions*/
        private: cv::Mat mapx;
        private: cv::Mat mapy;
        private: cv::Mat camera_matrix;                  ///< camera matrix
        private: cv::Mat dist_coeffs;                    ///< distorsion matrix

        //Image size
        private: int IMAGE_WIDTH;                        ///< image width
        private: int IMAGE_HEIGHT;                       ///< image height

        private: bool DEBUG_CV;                          ///< Debug computer vision algorithm
    };
}

#endif // PATTERNDETECTOR_H
