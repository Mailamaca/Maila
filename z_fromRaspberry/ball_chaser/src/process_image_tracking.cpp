#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <image_transport/image_transport.h>
#include <list>
#include "custom_msgs/ActuatorControl.h"
namespace enc = sensor_msgs::image_encodings;

custom_msgs::ActuatorControl ctrl;
ros::Publisher  autoCmdPublisher;
std::vector<std::string> trackerTypes = {"BOOSTING", "MIL", "KCF", "TLD", "MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};

custom_msgs::ActuatorControl last_cmd;
// Fill the vector with random colors
void getRandomColors(std::vector<cv::Scalar>& colors, int numColors)
{
  cv::RNG rng(0);
  for(int i=0; i < numColors; i++)
    colors.push_back(cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255))); 
}

void process_image_callback(const sensor_msgs::ImageConstPtr& msg)
{    
    const double max_speed = 0.5;
    const double max_angular = 2.5;

    static bool initialized = false;
    static std::vector<cv::Scalar> colors;  
    static cv::Rect2d box;
    
    // Specify the tracker type
    std::string trackerType = "BOOSTING";
    // Create multitracker
    //static cv::Tracker tracker;
    static cv::Ptr<cv::Tracker> tracker =  cv::TrackerKCF::createTracker(	); //cv::Tracker::create("BOOSTING");
    // Convert to rgb      
    cv_bridge::CvImageConstPtr cv_ptr; 
    cv_ptr = cv_bridge::toCvShare(msg,  enc::BGR8);

    //cv::namedWindow( "color", CV_WINDOW_AUTOSIZE );
    //cv::imshow( "color", cv_ptr->image );
    //cv::waitKey(10);
        
    if(!initialized){
        //cv::Mat src_gray;
        //cvtColor( cv_ptr->image, src_gray, CV_BGR2GRAY );
        /// Reduce the noise so we avoid false circle detection
        //cv::GaussianBlur( src_gray, src_gray, cv::Size(9, 9), 2, 2 );

        bool showCrosshair = true;
        bool fromCenter = false;
        
        std::cout << "\n==========================================================\n";
        std::cout << "OpenCV says press c to cancel objects selection process" << std::endl;
        std::cout << "It doesn't work. Press Escape to exit selection process" << std::endl;
        std::cout << "\n==========================================================\n";
        //cv::selectROIs("MultiTracker", cv_ptr->image, bboxes, showCrosshair, fromCenter);
        box = cv::selectROI(cv_ptr->image);
        
        if(box == cv::Rect2d() ){
            return;        
        }else{
            initialized = true;
            std::cout << "Initialized" << std::endl;
        }
        
        getRandomColors(colors, 1);
        
        // Initialize multitracker
        tracker->init((cv_ptr->image), box);
    }
    
      
    //Update the tracking result with new frame    
    if(tracker->update(cv_ptr->image, box)){
        
        double mean_x = (box.x + box.width/2)/static_cast<double>(cv_ptr->image.cols);
        double mean_y = (box.y + box.height/2)/static_cast<double>(cv_ptr->image.rows);
        
        last_cmd.stamp = ros::Time::now();        
        last_cmd.dc    =  mean_y > 0.8 ? 0 : 0.1; 
        last_cmd.delta = -2*(mean_x - 0.5);
                  
        
        cv::Point2d p1(box.x, box.y);
        cv::Point2d p2(box.x + box.width, box.y + box.height);
        cv::rectangle((cv_ptr->image), p1, p2, colors[0], 2, 1);
    }
    
    // Show frame
    cv::imshow("tracker", cv_ptr->image);

    // quit on x button
    cv::waitKey(10);
    
}

int main(int argc, char** argv)
{
     std::string auto_cmd_topic_name   = "/hw/autonomous_cmd";
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/image", 1, process_image_callback);

    // Handle ROS communication events
    autoCmdPublisher = n.advertise<custom_msgs::ActuatorControl>(auto_cmd_topic_name, 1, false);
    // Handle ROS communication events
    ros::Rate r(10);
    while(ros::ok()){                
        ros::spinOnce();
        ctrl = last_cmd;
        ctrl.stamp = ros::Time::now();    
        if((ros::Time::now() - last_cmd.stamp).toSec() > 0.1){            
            ctrl.dc = 0;
        }
        autoCmdPublisher.publish(ctrl);        
        r.sleep();  
    }

    return 0;
}
