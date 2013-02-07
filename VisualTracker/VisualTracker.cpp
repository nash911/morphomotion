#include "VisualTracker.h"

#define BOX_SIZE 30

using namespace cv;

VisualTracker::VisualTracker(void)
{
   /***********************OpenCV Debugger****************************
   cv::Mat image;
   cv::namedWindow( "Display Image", CV_WINDOW_AUTOSIZE );
   cv::waitKey(0);
   /***********************OpenCV Debugger****************************/

    // Initialize to camera #0
    //camera.open(0);

}

//std::vector<int> VisualTracker::get_realRobot_XY(void)
void VisualTracker::get_realRobot_XY(std::vector<int>& XY)
{
	/*std::vector<int> XY(2);
	XY[0]=0;
	XY[1]=0;*/
	
   namedWindow("video", 1);

   camera.open(0);
   //cv::Mat input_image_rgb; // Image matrix

   // Capture a single image from camera
   camera >> input_image_rgb;

   if(!input_image_rgb.data)
   {
     std::cout << "No input file from camera to process." << std::endl;
     //return -1;
   }
   //imshow("input_image_rgb", input_image_rgb);

   // Process the captured camera image
   process_image(input_image_rgb, XY);

   waitKey(0);
   camera.release();
   
   std::cout << "Ready to return from VisualTracker::get_realRobot_XY(std::vector<int>&)" << std::endl;
   
   //return(XY);
   return;
}


void VisualTracker::process_image(const cv::Mat & input_image_rgb, std::vector<int>& XY)
{
   printf("process_image:%ix%i\n", input_image_rgb.cols, input_image_rgb.rows);
   Mat3b imageHSV;
   Mat1b result;
   Mat1b Hbuffer;
   Mat1b Sbuffer;
   Mat1b Vbuffer;

   //imshow( "imageRGB", input_image_rgb );

   // OpenCV HSV encoding
   // H: 0 - 180, S: 0 - 255, V: 0 - 255.
   cv::cvtColor(input_image_rgb, imageHSV, CV_BGR2HSV, 0);
   //cv::imshow( "imageHSV", imageHSV );

   // blue color: H=240 (120 with OpenCV), S=100 (255 with OpenCV) , V=100 (255 with OpenCV)
   //int Hmin = 100, Hmax = 150, Smin  = 200, Smax = 255, Vmin = 200, Vmax = 255;
   //int Hmin = 100, Hmax = 150, Smin  = 105, Smax = 180, Vmin = 40, Vmax = 130;

   // Color --> Parrot Green: H=63-128 (31.5-64 with OpenCV), S=20-72 (51-183.6 with OpenCV) , V=26-54 (66.3-137.7 with OpenCV)
   //int Hmin = 25, Hmax = 70, Smin  = 45, Smax = 190, Vmin = 60, Vmax = 145;
   int Hmin = 30, Hmax = 65, Smin  = 50, Smax = 180, Vmin = 65, Vmax = 140;

   image_utils::HSVfilter(imageHSV,
                          250, result,
                          Hmin, Hmax, Smin, Smax, Vmin, Vmax,
                          Hbuffer, Sbuffer, Vbuffer);

   /*cv::imshow( "Hbuffer", Hbuffer );
   cv::imshow( "Sbuffer", Sbuffer );
   cv::imshow( "Vbuffer", Vbuffer );
   cv::imshow( "result", result );*/

   // process disjoint sets
   DisjointSets2 set(result);

   // instantiate output for get_connected_components()
   int cols = imageHSV.cols;
   int rows = imageHSV.rows;
   std::vector< DisjointSets2::Comp > components_pts;
   std::vector<cv::Rect> boundingBoxes;

   // get results
   set.get_connected_components(cols, components_pts, boundingBoxes);

/***************************************** Original **************************************************
   printf("Number of components: %i\n", (int) components_pts.size());
   for(int box_idx = 0; box_idx < boundingBoxes.size(); box_idx++)
   {
      printf("Box %i (%i, %i)+(%i, %i)\n",
             box_idx,
             boundingBoxes[box_idx].x,
             boundingBoxes[box_idx].y,
             boundingBoxes[box_idx].width,
             boundingBoxes[box_idx].height);
   }
   // make an illustration of the result
   cv::Mat3b illus = input_image_rgb.clone();
   for(int box_idx = 0; box_idx < boundingBoxes.size(); box_idx++)
   {
      cv::rectangle(illus, boundingBoxes[box_idx], CV_RGB(255, 0, 0), 2);
   }
   cv::imshow( "illus", illus );
/***************************************** Original **************************************************/

/***************************** Minimum boundingBox size approach **************************************
   std::vector<cv::Rect> boundingBoxes_filtered;

   // Remove components whose bounding box size is below a set limit.
   for(int box_idx = 0; box_idx < boundingBoxes.size(); box_idx++)
   {
      if((boundingBoxes[box_idx].width * boundingBoxes[box_idx].height) >= BOX_SIZE)
      {
         //boundingBoxes.erase(box_idx);
         //box_idx--;
         boundingBoxes_filtered.push_back(boundingBoxes[box_idx]);
      }
   }

   //printf("Number of components: %i\n", (int) components_pts.size());
   printf("Number of components: %i\n", (int) boundingBoxes_filtered.size());
   for(int box_idx = 0; box_idx < boundingBoxes_filtered.size(); box_idx++)
   {
      printf("Box %i (%i, %i)+(%i, %i)\n",
             box_idx,
             boundingBoxes_filtered[box_idx].x,
             boundingBoxes_filtered[box_idx].y,
             boundingBoxes_filtered[box_idx].width,
             boundingBoxes_filtered[box_idx].height);
   }

   // make an illustration of the result
   cv::Mat3b illus = input_image_rgb.clone();
   for(int box_idx = 0; box_idx < boundingBoxes_filtered.size(); box_idx++)
   {
       cv::rectangle(illus, boundingBoxes_filtered[box_idx], CV_RGB(255, 0, 0), 2);
   }
   cv::imshow( "illus", illus );
/***************************** Minimum boundingBox size approach **************************************/

/***************************** Sort by boundingBox size approach **************************************/
   cv::Rect tempBoundingBox;

   // Sort the components based on the bounding box size.
   for(int i = 0; i < boundingBoxes.size()-1; i++)
   {
      for(int j = i+1; j < boundingBoxes.size(); j++)
      {
         if((boundingBoxes[j].width * boundingBoxes[j].height) > (boundingBoxes[i].width * boundingBoxes[i].height))
         {
            tempBoundingBox = boundingBoxes[i];
            boundingBoxes[i] = boundingBoxes[j];
            boundingBoxes[j] = tempBoundingBox;
         }
      }
   }

   printf("Total number of components: %i\n", (int) boundingBoxes.size());
   std::cout << "Top three components:" << std::endl;
   for(int box_idx = 0; box_idx < 3; box_idx++)
   {
      printf("Box %i (%i, %i)+(%i, %i)\n",
             box_idx,
             boundingBoxes[box_idx].x,
             boundingBoxes[box_idx].y,
             boundingBoxes[box_idx].width,
             boundingBoxes[box_idx].height);
   }
   // make an illustration of the result
   cv::Mat3b illus = input_image_rgb.clone();
   for(int box_idx = 0; box_idx < 3; box_idx++)
   {
      cv::rectangle(illus, boundingBoxes[box_idx], CV_RGB(255, 0, 0), 1);
   }
   //cv::imshow( "illus", illus );
/***************************** Sort by boundingBox size approach **************************************/

/***************************** Extract center point of the markers ************************************/
   std::vector<cv::Point> center(3);
   cv::Point temp_point;

   for(int box_idx = 0; box_idx < 3; box_idx++)
   {
      temp_point.x = boundingBoxes[box_idx].x + (boundingBoxes[box_idx].width/2);
      temp_point.y = boundingBoxes[box_idx].y + (boundingBoxes[box_idx].height/2);
      center[box_idx] = temp_point;
   }

   cv::Mat3b illus_triangle = illus.clone();

   cv::line(illus_triangle, center[0], center[1], CV_RGB(0, 0, 255), 1, 8, 0);
   cv::line(illus_triangle, center[1], center[2], CV_RGB(0, 0, 255), 1, 8, 0);
   cv::line(illus_triangle, center[2], center[0], CV_RGB(0, 0, 255), 1, 8, 0);

   //cv::imshow( "illus_triangle", illus_triangle );
/***************************** Extract center point of the markers ************************************/

/************************************ Detect orientation marker ***************************************/

   std::vector<double> diff_eucDistance(3);

   diff_eucDistance[0] = fabs(euclidean_distance(center[0], center[1]) - euclidean_distance(center[0], center[2]));
   diff_eucDistance[1] = fabs(euclidean_distance(center[1], center[0]) - euclidean_distance(center[1], center[2]));
   diff_eucDistance[2] = fabs(euclidean_distance(center[2], center[0]) - euclidean_distance(center[2], center[1]));

   for(int i=0; i<3; i++)
   {
      std::cout << "euclidean_distance " << i+1 << ": " << diff_eucDistance[i] << std::endl;
   }

   int orientationMarker_index = -1;
   cv::Point point2;
   /*for(int i=0; i<3; i++)
   {
      if(diff_eucDistance[i] <= 1)
      {
         orientationMarker_index = i;
         cv::rectangle(illus_triangle, boundingBoxes[i], CV_RGB(0, 255, 0), 1);

         if(i == 0)
         {
            point2.x = center[2].x + ((center[1].x - center[2].x)/2);
            point2.y = center[2].y + ((center[1].y - center[2].y)/2);
         }
         else if(i == 1)
         {
            point2.x = center[2].x + ((center[0].x - center[2].x)/2);
            point2.y = center[2].y + ((center[0].y - center[2].y)/2);
         }
         else if(i == 2)
         {
            point2.x = center[1].x + ((center[0].x - center[1].x)/2);
            point2.y = center[1].y + ((center[0].y - center[1].y)/2);
         }

         cv::line(illus_triangle, center[i], point2, CV_RGB(0, 0, 255), 1, 8, 0);
      }
   }*/

   if(diff_eucDistance[0] < diff_eucDistance[1] && diff_eucDistance[0] < diff_eucDistance[2])
   {
      orientationMarker_index = 0;
      cv::rectangle(illus_triangle, boundingBoxes[0], CV_RGB(0, 255, 0), 1);

      point2.x = center[2].x + ((center[1].x - center[2].x)/2);
      point2.y = center[2].y + ((center[1].y - center[2].y)/2);

      cv::line(illus_triangle, center[0], point2, CV_RGB(0, 0, 255), 1, 8, 0);
      
      XY[0] = boundingBoxes[0].x + (boundingBoxes[0].width/2);
      XY[1] = boundingBoxes[0].y + (boundingBoxes[0].height/2);
      
   }
   else if(diff_eucDistance[1] < diff_eucDistance[0] && diff_eucDistance[1] < diff_eucDistance[2])
   {
      orientationMarker_index = 1;
      cv::rectangle(illus_triangle, boundingBoxes[1], CV_RGB(0, 255, 0), 1);

      point2.x = center[2].x + ((center[0].x - center[2].x)/2);
      point2.y = center[2].y + ((center[0].y - center[2].y)/2);

      cv::line(illus_triangle, center[1], point2, CV_RGB(0, 0, 255), 1, 8, 0);
      
      XY[0] = boundingBoxes[1].x + (boundingBoxes[1].width/2);
      XY[1] = boundingBoxes[1].y + (boundingBoxes[1].height/2);
   }
   else if(diff_eucDistance[2] < diff_eucDistance[0] && diff_eucDistance[2] < diff_eucDistance[1])
   {
      orientationMarker_index = 2;
      cv::rectangle(illus_triangle, boundingBoxes[2], CV_RGB(0, 255, 0), 1);

      point2.x = center[1].x + ((center[0].x - center[1].x)/2);
      point2.y = center[1].y + ((center[0].y - center[1].y)/2);

      cv::line(illus_triangle, center[2], point2, CV_RGB(0, 0, 255), 1, 8, 0);
      
      XY[0] = boundingBoxes[2].x + (boundingBoxes[2].width/2);
      XY[1] = boundingBoxes[2].y + (boundingBoxes[2].height/2);
   }

   cv::imshow( "illus_triangle", illus_triangle );
/************************************ Detect orientation marker ***************************************/

/************************************** Determine orientation *****************************************
   int quad = 0;

   if(center[orientationMarker_index].x <= cols/2)
   {
      if(center[orientationMarker_index].y <= rows/2)
      {
         quad = 2;
      }
      else
      {
         quad = 3;
      }
   }
   else
   {
      if(center[orientationMarker_index].y <= rows/2)
      {
         quad = 1;
      }
      else
      {
         quad = 4;
      }
   }

   std::cout << std::endl << "Quadrant: " << quad << std::endl;




/************************************** Determine orientation *****************************************/

}

double VisualTracker::euclidean_distance(cv::Point point1, cv::Point point2)
{
   return( sqrt(((point1.x - point2.x)*(point1.x - point2.x)) + ((point1.y - point2.y)*(point1.y - point2.y))));
}
