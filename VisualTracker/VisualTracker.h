#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "layer_utils.h"
#include "disjoint_sets2.h"


class VisualTracker
{
private:
    cv::VideoCapture camera; // From camera or video
    cv::Mat input_image_rgb; // Image matrix

public:
   VisualTracker(void);
   ~VisualTracker(){ };
   //std::vector<int> get_realRobot_XY(void);
   //void get_realRobot_XY(void);
   void get_realRobot_XY(std::vector<int>&);
   void process_image(const cv::Mat&, std::vector<int>&);
   double euclidean_distance(cv::Point, cv::Point);
};