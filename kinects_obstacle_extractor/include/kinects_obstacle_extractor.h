




// opencv
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// standard C
#include <stdio.h>
#include <iostream>

//ros-includes
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/sync_policies/approximate_time.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

// ROBOT 
#include "../include/matrix.hpp"
#include "../include/kinematics.h"
#include "../include/depth_processor.h"
#include "../include/robotFilter.hpp"

#define CAMERA_INTRINSIC    (0)
#define CAMERA_EXTRINSIC    (1)
#define CAMERA_WORKSPACE    (2)
#define WORKSPACE_INIT_DONE (10)

using namespace std;
using namespace ROBOT_FILTER;

typedef sensor_msgs::Image ImgMsg;
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

RobotFilter _robotFilter;


// thread for converting the sensor_msgs::Image to opencv::mat
void imgMatSaver();

// print out the robot state
void printRobotStates();

// regist subscriber callbacks
int registSubscriberCallbacks(int nb_in_topics,ros::NodeHandle nh);

// thread for extracting the distance between the obstale and robot
void extractor(const sensor_msgs::JointStateConstPtr & msg);

// get the closed distance from the obstacles to the robot links
void getTheClosedDistance();

// get robot workspace grid
int reProjectWorkspace_;
vector<vector<double> > gridCenter_;
bool getRobotWorkSpaceGrid();

// get the offline project relation
bool getOfflineRelation();

// save the camera parameter get and/or save the robot workspace 
void saveTheCameraParameter();

// initiliaze the cloud for the robot workspace
int workspaceInitiate;
void initWorkspaceCloud();
PointCloudT::Ptr workspace_free_;
PointCloudT::Ptr workspace_obstacle_;

// update the robot workspace point cloud
void updateRobotWorkspaceCloud(int i);

// check all the Occlusion points in the workspace cloud
void checkOcclusionPointsInWorkspaceCloud();

// mutex lock between ImgMatSaver() and extractor()
Mutex depthIMgMutex;

// vector of opencv::mat for depthProcessors
vector<cv::Mat> img_mat_vector_;

// vector of sensor_msgs::Image received from kinects
vector<ImgMsg::ConstPtr> img_msg_vector_;

// vector of depthAsRgb for displaying
vector<cv::Mat> depthImgAsArgb_mat_vector_;

// publishers for the processed depth images
vector<ros::Publisher> pub_;

// publishers for the processed robot workspace
vector<ros::Publisher> rwPub_;

// publisher for the robot markers
ros::Publisher rmPub_;

// ROBOT Markers
visualization_msgs::MarkerArray robotMarkers_;

void publishRobotMarkers();

// vector of processed msgses for publishing, may be local variables
vector<ImgMsg::Ptr> processed_img_;

// publish the processed depthData as rgb images with obstacles
vector<string> out_Imgs_topics_;

// subscribe the kinect depth images to get the obstacles
vector<string> in_depthImgs_topics_;

// name of the frame name of the robot base
string robot_base_frame_;

// topic of the robot joint states
string robot_joint_states_topic_;

// output point cloud, obstacle in the robot workspace
string output_topic_obstacle_cloud_;

// output free workspace
string output_topic_free_space_;

// every CYCLE cycles of received jointStates, print out it on the screen
int cunt;
#define CYCLE 30

// transformations between kinects depth optical frames and robot base frame
// get the transformation between kinects and robot base in the first cycle
bool first_frame_;
struct pclTransform{
  Eigen::Vector3d translation;
  Eigen::Quaterniond rotation;
};
vector<pclTransform> transforms_;
void getTheTransformation();

// depth images processor to get the obstacles from depht images
vector<depth_processor*> depthProcessor_;

// robot configuration
CONFIG* config_;

// robot kinematics
R_KINE* rkine_;

// robot workspace
struct robotWorkSpace
{
  double xMin;
  double xMax;
  double xNum;
  double yMin;
  double yMax;
  double yNum;
  double zMin;
  double zMax;
  double zNum;
} rWorkspace_;

// reduandant parameter
double phi = 0.0;

// end-effector of the robot
double EndEffector[4][4];

// velocity of the end effector point
double xVelocity[DOF];

// joint angle of the robot
double jointAngle[DOF];

// joint velocity of the robot
double jointVelocity[DOF];

// get link points on the robot in robot base frame
void getLinkPointsInRobotbase();

// link points on the robot in robot base space
double linkPointsInRobotbase[POINTSNUM][4];

typedef message_filters::sync_policies::ApproximateTime<ImgMsg, ImgMsg> MySyncPolicy2;
typedef message_filters::sync_policies::ApproximateTime<ImgMsg, ImgMsg, ImgMsg> MySyncPolicy3;
typedef message_filters::sync_policies::ApproximateTime<ImgMsg, ImgMsg, ImgMsg, ImgMsg> MySyncPolicy4;
typedef message_filters::sync_policies::ApproximateTime<ImgMsg, ImgMsg, ImgMsg, ImgMsg, ImgMsg> MySyncPolicy5;
typedef message_filters::sync_policies::ApproximateTime<ImgMsg, ImgMsg, ImgMsg, ImgMsg, ImgMsg, ImgMsg> MySyncPolicy6;
typedef message_filters::sync_policies::ApproximateTime<ImgMsg, ImgMsg, ImgMsg, ImgMsg, ImgMsg, ImgMsg, ImgMsg> MySyncPolicy7;
typedef message_filters::sync_policies::ApproximateTime<ImgMsg, ImgMsg, ImgMsg, ImgMsg, ImgMsg, ImgMsg, ImgMsg, ImgMsg> MySyncPolicy8;
typedef message_filters::sync_policies::ApproximateTime<ImgMsg, ImgMsg, ImgMsg, ImgMsg, ImgMsg, ImgMsg, ImgMsg, ImgMsg, ImgMsg> MySyncPolicy9;

void callback9(const ImgMsg::ConstPtr& first_msg_img, const ImgMsg::ConstPtr& second_msg_img, const ImgMsg::ConstPtr& third_msg_img,
               const ImgMsg::ConstPtr& fourth_msg_img, const ImgMsg::ConstPtr& fifth_msg_img, const ImgMsg::ConstPtr& sixth_msg_img,
               const ImgMsg::ConstPtr& seventh_msg_img, const ImgMsg::ConstPtr& eighth_msg_img, const ImgMsg::ConstPtr& ninth_msg_img );
void callback8(const ImgMsg::ConstPtr& first_msg_img, const ImgMsg::ConstPtr& second_msg_img, const ImgMsg::ConstPtr& third_msg_img, 
               const ImgMsg::ConstPtr& fourth_msg_img, const ImgMsg::ConstPtr& fifth_msg_img, const ImgMsg::ConstPtr& sixth_msg_img, 
               const ImgMsg::ConstPtr& seventh_msg_img, const ImgMsg::ConstPtr& eighth_msg_img );
void callback7(const ImgMsg::ConstPtr& first_msg_img, const ImgMsg::ConstPtr& second_msg_img, const ImgMsg::ConstPtr& third_msg_img, 
               const ImgMsg::ConstPtr& fourth_msg_img, const ImgMsg::ConstPtr& fifth_msg_img, const ImgMsg::ConstPtr& sixth_msg_img, 
               const ImgMsg::ConstPtr& seventh_msg_img );
void callback6(const ImgMsg::ConstPtr& first_msg_img, const ImgMsg::ConstPtr& second_msg_img, const ImgMsg::ConstPtr& third_msg_img, 
               const ImgMsg::ConstPtr& fourth_msg_img, const ImgMsg::ConstPtr& fifth_msg_img, const ImgMsg::ConstPtr& sixth_msg_img );
void callback5(const ImgMsg::ConstPtr& first_msg_img, const ImgMsg::ConstPtr& second_msg_img, const ImgMsg::ConstPtr& third_msg_img, 
               const ImgMsg::ConstPtr& fourth_msg_img, const ImgMsg::ConstPtr& fifth_msg_img );
void callback4(const ImgMsg::ConstPtr& first_msg_img, const ImgMsg::ConstPtr& second_msg_img, const ImgMsg::ConstPtr& third_msg_img, 
               const ImgMsg::ConstPtr& fourth_msg_img);
void callback3(const ImgMsg::ConstPtr& first_msg_img, const ImgMsg::ConstPtr& second_msg_img, const ImgMsg::ConstPtr& third_msg_img );
void callback2(const ImgMsg::ConstPtr& first_msg_img, const ImgMsg::ConstPtr& second_msg_img);
void callback1(const ImgMsg::ConstPtr& first_msg_img);