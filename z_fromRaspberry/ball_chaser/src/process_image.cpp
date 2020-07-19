#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <experimental/filesystem>

#include <image_transport/image_transport.h>
#include <list>

#include "custom_msgs/ActuatorControl.h"

custom_msgs::ActuatorControl ctrl;
ros::Publisher  autoCmdPublisher;
ros::Time last_clbk;
double cmd_v, cmd_steer;
namespace enc = sensor_msgs::image_encodings;
// Defintion of the function pickNPoints and the callback mouseCallback.
// The function pickNPoints is used to display a window with a background
// image, and to prompt the user to select n points on this image.
static cv::Mat bg_img;
static std::vector<cv::Point2f> result;
static std::string name;
static std::atomic<bool> done;
static int n;
static double show_scale = 1.0;

void mouseCallback(int event, int x, int y, int, void* p)
{
    if (event != cv::EVENT_LBUTTONDOWN || done.load()) return;

    result.emplace_back(x*show_scale, y*show_scale);
    cv::circle(bg_img, cv::Point(x,y), 20/show_scale, cv::Scalar(0,0,255), -1);
    cv::imshow(name.c_str(), bg_img);

    if (result.size() >= n) {
      usleep(500*1000);
      done.store(true);
    }
}
  
std::vector<cv::Point2f> pickNPoints(int n0, const cv::Mat& img)
{    
    result.clear();
    cv::Size small_size(img.cols/show_scale, img.rows/show_scale);
    cv::resize(img, bg_img, small_size);
    //bg_img = img.clone();
    name = "Pick " + std::to_string(n0) + " points";
    cv::imshow(name.c_str(), bg_img);
    cv::namedWindow(name.c_str());
    n = n0;

    done.store(false);

    cv::setMouseCallback(name.c_str(), &mouseCallback, nullptr);
    while (!done.load()) {
      cv::waitKey(500);
    }

    cv::destroyWindow(name.c_str());
    return result;
}
  
bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec){

    std::string config_folder = "/home/pi/workspace/maila/calibration/";
    std::string file_path = config_folder + "extrinsicCalib.csv";

    std::vector<cv::Point2f> image_points;

    if (!std::experimental::filesystem::exists(file_path)){
          
      std::experimental::filesystem::create_directories(config_folder);
      
      image_points = pickNPoints(4, img_in);
      // SAVE POINT TO FILE
      // std::cout << "IMAGE POINTS: " << std::endl;
      // for (const auto pt: image_points) {
      //   std::cout << pt << std::endl;
      // }
      std::ofstream output(file_path);
      if (!output.is_open()){
        throw std::runtime_error("Cannot write file: " + file_path);
      }
      for (const auto pt: image_points) {
        output << pt.x << " " << pt.y << std::endl;
      }
      output.close();
    }else{
      // LOAD POINT FROM FILE
      std::ifstream input(file_path);
      if (!input.is_open()){
        throw std::runtime_error("Cannot read file: " + file_path);
      }
      while (!input.eof()){
        double x, y;
        if (!(input >> x >> y)) {
          if (input.eof()) break;
          else {
            throw std::runtime_error("Malformed file: " + file_path);
          }
        }
        image_points.emplace_back(x, y);
      }
      input.close();
    }
    
    cv::Mat dist_coeffs;
    dist_coeffs   = (cv::Mat1d(1,4) << 0, 0, 0, 0, 0);
    bool ok = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);

    // cv::Mat Rt;
    // cv::Rodrigues(rvec_, Rt);
    // auto R = Rt.t();
    // auto pos = -R * tvec_;

    if (!ok) {
      std::cerr << "FAILED SOLVE_PNP" << std::endl;
    }

    return ok;
  }


void erosion( cv::Mat& img, int erosion_size )
{
  int erosion_elem = 2;
  int erosion_type = 0;
  if( erosion_elem == 0 ){ erosion_type = cv::MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = cv::MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = cv::MORPH_ELLIPSE; }

  cv::Mat element = cv::getStructuringElement( erosion_type,
                       cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                       cv::Point( erosion_size, erosion_size ) );
  /// Apply the erosion operation
  cv::erode( img, img, element );  
}

void dilation( cv::Mat& img, int dilation_size )
{
  int dilation_elem = 2;
  int dilation_type = 0;
  if( dilation_elem == 0 ){ dilation_type = cv::MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = cv::MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = cv::MORPH_ELLIPSE; }

  cv::Mat element = cv::getStructuringElement( dilation_type,
                       cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                       cv::Point( dilation_size, dilation_size ) );
  /// Apply the erosion operation
  cv::dilate( img, img, element );  
}

void process_image_callback(const sensor_msgs::ImageConstPtr& msg)
{       
    const double max_speed = 0.5;
    const double max_angular = 2.5;
    static cv::Mat rvec, tvec;
    static bool calibrated = true;
    static cv::Mat1f transf;
    
    // Convert to rgb      
    cv_bridge::CvImageConstPtr cv_ptr; 
    cv_ptr = cv_bridge::toCvShare(msg,  enc::BGR8);
     
    // Resize 
    cv::Mat small_frame, hsv_img;
    cv::resize(cv_ptr->image, small_frame, cv::Size(410, 308));
    //small_frame = cv_ptr->image;
    cv::Mat camera_matrix; //(3,3,CV_32F);
    camera_matrix = (cv::Mat1d(3,3) << 322., 0., 205., 0., 320., 154., 0., 0., 1.);    
    //std::cout << "A0" << std::endl;                         
    if(!calibrated){
        std::vector<cv::Point3f> object_points;        
        object_points.emplace_back(320,-60, 0);
        object_points.emplace_back(180,-60, 0);
        object_points.emplace_back(180, 60, 0);
        object_points.emplace_back(320, 60, 0);
        calibrated = extrinsicCalib(small_frame, object_points, camera_matrix, rvec, tvec);
        
        if(!calibrated) return;
        // project points
        cv::Mat image_points;
        std::vector<cv::Point2f> dest_image_points_plane;
        dest_image_points_plane.emplace_back(320-100,90);
        dest_image_points_plane.emplace_back(180-100, 90);
        dest_image_points_plane.emplace_back(180-100, 210);
        dest_image_points_plane.emplace_back(320-100, 210);        
        cv::projectPoints(object_points, rvec, tvec, camera_matrix, cv::Mat(), image_points);
        transf = cv::getPerspectiveTransform(image_points, dest_image_points_plane);
    
        return;
    }    
    //cv::Mat image_points;
    
    //std::cout << "HEREEEE" << std::endl;
    //cv::Mat img_out;
    //cv::warpPerspective(small_frame, img_out, transf, small_frame.size());
    //cv::imshow("img_out", img_out);
    //cv::waitKey(1);   
    
    
    // Convertire HSV
    cv::cvtColor(small_frame,hsv_img,CV_BGR2HSV);
    
    // Trova blob    
    cv::Mat red_mask_low, red_mask_high, red_mask;     
    cv::inRange(hsv_img, cv::Scalar(0, 190, 30), cv::Scalar(20, 255, 230), red_mask_low);
    cv::inRange(hsv_img, cv::Scalar(170, 190, 30), cv::Scalar(180, 255, 230), red_mask_high);
    cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_mask); 

    //cv::imshow("redMask", red_mask);
    //cv::waitKey(1);
        
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point> proj_pt;
       

    // EROSION
    erosion( red_mask, 5 );
    dilation( red_mask, 8 );
    //std::cout << "A" << std::endl;
        
    // Process red mask
    cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    drawContours(small_frame, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
    //std::cout << "N. contours: " << contours.size() << std::endl;
    bool found = false;
    double max_area = - 1;
    int max_i = 0;
    for (int i=0; i<contours.size(); ++i)
    {        
      const double area = cv::contourArea(contours[i]);
      if(area > max_area){
          found = true;
          max_area = area;
          max_i = i;
      }
    }
    
    if(!found) return;
    
    //std::cout << "STAMPA" << transf << std::endl;
    
    //cv::perspectiveTransform(contours[max_i], proj_pt, transf);
    
    cv::Point min_pt(0,0);
    cv::Point baric(0,0);
    for(auto pt : contours[max_i]){
        baric += pt;
        
        if(pt.y > min_pt.y){
            min_pt = pt;
        }
    }
    baric.x = baric.x/contours[max_i].size();
    baric.y = baric.y/contours[max_i].size();
    
    cv::circle(small_frame, min_pt, 5, cv::Scalar(255,0,0), -1);
    cv::circle(small_frame, baric, 5, cv::Scalar(255,255,0), -1);
    
    std::cout << min_pt << std::endl;
    std::cout << std::endl;
    //cv::imshow("Original", small_frame);
    //cv::waitKey(1);
    
    //cv::namedWindow( "color", CV_WINDOW_AUTOSIZE );
    //cv::imshow( "color", cv_ptr->image );
    //cv::waitKey(10);
    
    
    cmd_v = min_pt.y < 270? 0.1:0;
    cmd_steer = -(baric.x-205)/205.0;
    last_clbk = ros::Time::now();    
        
}

int main(int argc, char** argv)
{
    std::string auto_cmd_topic_name   = "/hw/autonomous_cmd";
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/image/", 1, process_image_callback); ///raspicam_node/image
    autoCmdPublisher = n.advertise<custom_msgs::ActuatorControl>(auto_cmd_topic_name, 1, false);
    // Handle ROS communication events
    ros::Rate r(10);
    while(ros::ok()){                
        ros::spinOnce();
        ctrl.stamp = ros::Time::now();    
        if((ros::Time::now() - last_clbk).toSec()<0.1){            
            ctrl.dc = cmd_v;                
        }else{
            ctrl.dc = 0;
        }
        std::cout << "AAAAA" << std::endl;
        ctrl.delta = cmd_steer;          
        autoCmdPublisher.publish(ctrl);        
        r.sleep();  
    }
    
    return 0;    
}
