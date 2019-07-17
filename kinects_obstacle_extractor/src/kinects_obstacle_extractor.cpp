
#pragma once
#include <stdio.h>
#include <iostream>
#include "../include/kinects_obstacle_extractor.h"

using namespace std;

int main(int argc, char** argv){

  sleep(5); // wait for the gazebo to launch

  // ros initialize
  ros::init(argc, argv, "obstacle_extractor");
  ros::NodeHandle nh, nh_priv("~");
  
  // initiate global varibles
  cunt = 0;
  first_frame_ = true;
  config_ = new CONFIG;
  rkine_ = new R_KINE(config_);
  workspaceInitiate = 0;

  memset(xVelocity,0,sizeof(xVelocity));
  memset(jointAngle,0,sizeof(jointAngle));
  memset(EndEffector,0,sizeof(EndEffector));
  memset(jointVelocity,0,sizeof(jointVelocity));
  memset(linkPointsInRobotbase,0,sizeof(linkPointsInRobotbase));
  
  // Get the input depth images topics
  bool done_loading;
  int nb_in_topics = 0;
  string topic_name, temp;
  cout << "extracting obstacles from : " <<endl;
  do{
    topic_name = "kinect_topic_name"+ boost::lexical_cast<string>(nb_in_topics+1);    
    done_loading = (!nh_priv.getParam(topic_name, temp)) || (temp=="") ;
    
    if(!done_loading){
      in_depthImgs_topics_.push_back(temp);
      cout << temp <<endl;
      nb_in_topics++;
    }
  }while(!done_loading);
  
  // get the output processed_images topics
  int nb_out_topics = 0;
  cout << "output processed depth images to : " <<endl;
  do{
    topic_name = "out_topic_name"+ boost::lexical_cast<string>(nb_out_topics+1);    
    done_loading = (!nh_priv.getParam(topic_name, temp)) || (temp=="") ;
    
    if(!done_loading){
      out_Imgs_topics_.push_back(temp);
      cout << temp <<endl;
      nb_out_topics++;
    }
  }while(!done_loading);
  
  // get the robot joint states topics and base frame
  nh_priv.getParam("robot_joint_states",robot_joint_states_topic_);
  nh_priv.getParam("robot_base_frame",robot_base_frame_);
  cout << "get robot joint states from : " <<robot_joint_states_topic_<<" in frame "<< robot_base_frame_<<endl;

  // the output point cloud, obstacles in the robot workspace
  nh_priv.getParam("out_topic_obstacle_cloud",output_topic_obstacle_cloud_);
  cout<<"the obstacles in the robot workspace is publised to the topic:"<<output_topic_obstacle_cloud_<<endl;

  //the output free robot workspace
  nh_priv.getParam("out_topic_free_space",output_topic_free_space_);
  cout<<"the free workspace is published to the topic: "<<output_topic_free_space_<<endl;

  // get the robot work space
  double wsMin[3] = {0.0};
  if (nh_priv.getParam("robot_workspace/minimum",temp)){
    sscanf(temp.c_str(),"%lf,%lf,%lf",wsMin,wsMin+1,wsMin+2);
    rWorkspace_.xMin = wsMin[0]; rWorkspace_.yMin = wsMin[1]; rWorkspace_.zMin = wsMin[2];
  }else{
    cout<<"can not find the min limit of the robot workspace"<<endl;
  }

  double wsMax[3] = {0.0}; temp = "";
  if(nh_priv.getParam("robot_workspace/maximum",temp)){
    sscanf(temp.c_str(),"%lf,%lf,%lf",wsMax,wsMax+1,wsMax+2);
    rWorkspace_.xMax = wsMax[0]; rWorkspace_.yMax = wsMax[1]; rWorkspace_.zMax = wsMax[2];
  }else
  {
    cout<<"can not find the max limit of the robot workspace"<<endl;
  }

  double wsNum[3] = {0.0}; temp = "";
  if (nh_priv.getParam("robot_workspace/point_num",temp)){
    sscanf(temp.c_str(),"%lf,%lf,%lf",wsNum,wsNum+1,wsNum+2);
    rWorkspace_.xNum = wsNum[0]; rWorkspace_.yNum = wsNum[1]; rWorkspace_.zNum = wsNum[2];
  }else{
    cout<<"cant not find the discreate num of the robot workspace"<<endl;
  }

  cout<< "the robot workSpace is: "<<endl;
  cout<< "\t"<<rWorkspace_.xMin<<" "<<rWorkspace_.xMax<<" "<<rWorkspace_.xNum<<endl;
  cout<< "\t"<<rWorkspace_.yMin<<" "<<rWorkspace_.yMax<<" "<<rWorkspace_.yNum<<endl;
  cout<< "\t"<<rWorkspace_.zMin<<" "<<rWorkspace_.zMax<<" "<<rWorkspace_.zNum<<endl;

  // temp = "";
  if (nh_priv.getParam("reproject_workspace",temp)){ //有问题。。。。
    cout<<temp<<endl;
    sscanf(temp.c_str(),"%d",&reProjectWorkspace_);
    cout<<"reProjectWorkspace_: "<<reProjectWorkspace_<<endl;
    if(reProjectWorkspace_){
        cout<<"wait for calculating the robot workspace ..."<<endl;
    }else{
      cout<<"wait for reading the robot workspace"<<endl;
    }
  }else{
    reProjectWorkspace_ = 1;
    cout<<"wait for calculating the robot workspace ..."<<endl;
  }

  // check input output topics
  if (nb_in_topics != nb_out_topics){
    cout << "the numbers of kinect topics and out topics should be the same!\n"<<endl;
    cout << "PLEASE CHECK THE LAUNCH FILE!"<<endl;
    return -1; 
  }

  // transforms and depth processor
  transforms_.resize(nb_in_topics);
  depthProcessor_.resize(nb_in_topics);
  for (int i=0; i<nb_in_topics; i++){
    depthProcessor_[i] = new depth_processor();
  }

  // publishers and output msgs
  pub_.resize(nb_out_topics);
  processed_img_.resize(nb_out_topics);
  
  // Declaration Ros Publishers
  for (int i=0; i<nb_out_topics; i++){
    pub_[i] = nh.advertise<ImgMsg>(out_Imgs_topics_[i],1);
  }

  // declaration the output point cloud publishaers
  rwPub_.resize(2);
  rwPub_[0] = nh.advertise<sensor_msgs::PointCloud2> (output_topic_obstacle_cloud_, 1);
  rwPub_[1] = nh.advertise<sensor_msgs::PointCloud2> (output_topic_free_space_,1);

  // declaration the robot marker publisher
  rmPub_ = nh.advertise<visualization_msgs::MarkerArray>("Robot_Markers", 1);

  // robot joint sates call back: extractor()
  ros::Subscriber robot_state = nh.subscribe<sensor_msgs::JointState>(robot_joint_states_topic_,1,extractor);
  
  // regist input depth topics' subscriber call backs
  int result = registSubscriberCallbacks(nb_in_topics,nh);

  return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void publishRobotMarkers(){

  // robotMarkers_.markers.clear();
  // visualization_msgs::Marker marker[POINTSNUM];
  // for(int i=0; i<POINTSNUM;i++){
    
  //   marker[i].header.frame_id = robot_base_frame_;
  //   marker[i].header.stamp = ros::Time::now();
  //   marker[i].ns = "robot_kniect";
  //   marker[i].id = i;
  //   marker[i].type = visualization_msgs::Marker::SPHERE;
  //   marker[i].action = visualization_msgs::Marker::ADD;
  //   marker[i].pose.position.x = linkPointsInRobotbase[i][0]/1000.0;
  //   marker[i].pose.position.y = linkPointsInRobotbase[i][1]/1000.0;
  //   marker[i].pose.position.z = linkPointsInRobotbase[i][2]/1000.0;
  //   marker[i].scale.x = linkPointsInRobotbase[i][3]/1000.0  * 1.2;
  //   marker[i].scale.y = linkPointsInRobotbase[i][3]/1000.0  * 1.2;
  //   marker[i].scale.z = linkPointsInRobotbase[i][3]/1000.0  * 1.2;
  //   marker[i].color.a = 0.5;
  //   marker[i].color.r = 0.5;
  //   marker[i].color.g = 0.5;
  //   marker[i].color.b = 0;
  //   robotMarkers_.markers.push_back(marker[i]);
  // }


  int num = _robotFilter.getNumOfShapes();
  robotMarkers_.markers.clear();
  visualization_msgs::Marker marker[num];
  for(int i=0; i<num;i++){
    Shape temp = _robotFilter.getShape(i,jointAngle);
    
    marker[i].header.frame_id = robot_base_frame_;
    marker[i].header.stamp = ros::Time::now();
    marker[i].ns = "robot_kniect";
    marker[i].id = i;
    marker[i].type = visualization_msgs::Marker::CYLINDER;
    marker[i].action = visualization_msgs::Marker::ADD;
    marker[i].pose.position.x = temp.position.x/1000.0;
    marker[i].pose.position.y = temp.position.y/1000.0;
    marker[i].pose.position.z = temp.position.z/1000.0;
    marker[i].pose.orientation.x = temp.rotation.x;
    marker[i].pose.orientation.y = temp.rotation.y;
    marker[i].pose.orientation.z = temp.rotation.z;
    marker[i].pose.orientation.w = temp.rotation.w;

    marker[i].scale.x = 1.3*2*temp._upPoints[0].r/1000.0;
    marker[i].scale.y = 1.3*2*temp._upPoints[0].r/1000.0;
    marker[i].scale.z = 1.2*(temp._upPoints[0].z/1000.0-temp._lowPoints[0].z/1000.0);
    marker[i].color.a = 0.3;
    marker[i].color.r = 0;
    marker[i].color.g = 0.5;
    marker[i].color.b = 0;
    robotMarkers_.markers.push_back(marker[i]);
  }

  rmPub_.publish(robotMarkers_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void imgMatSaver(){
  
  // get the transfromation 
  if (first_frame_){

    // get the transformation
    getTheTransformation();

    // save the transfromation
    saveTheCameraParameter();

    //get the offline project relation between depth space and the work sapce
    getRobotWorkSpaceGrid();

    // get offline relation between kinect and robot workspace
    getOfflineRelation();
    
    // initiliaze robot workspace cloud
    initWorkspaceCloud();

    cout<<"robot workspace ready"<<endl;

    first_frame_ = false;
  }

  // print out the robot state
  // printRobotStates();

  depthIMgMutex.lock();
  {
      // publish robot markers
      publishRobotMarkers();

      // publish the last depth as rgb images
      for (int i=0; i<depthImgAsArgb_mat_vector_.size(); i++ ){
        // cv::imshow(out_Imgs_topics_[i],depthImgAsArgb_mat_vector_[i]);
        // cv::waitKey(1);
        // convert to sensor_msgs::Image
        cv_bridge::CvImage out_ros_img;
        out_ros_img.encoding =  sensor_msgs::image_encodings::RGB8;
        out_ros_img.header.seq = cunt;
        out_ros_img.header.stamp = ros::Time::now();// time
        out_ros_img.image = depthImgAsArgb_mat_vector_[i];
        sensor_msgs::Image img_msg;
        out_ros_img.toImageMsg(img_msg);
        if(pub_[i].getNumSubscribers() > 0){
          pub_[i].publish(img_msg);
        }
      }

      sensor_msgs::PointCloud2 output2;
      pcl::PCLPointCloud2::Ptr output (new pcl::PCLPointCloud2 ());

      if(rwPub_[0].getNumSubscribers() > 0){
        // Convert data type
        pcl::toPCLPointCloud2(*workspace_obstacle_, *output);
        // Convert to ROS data type
        pcl_conversions::fromPCL(*output, output2);
        output2.header.frame_id = robot_base_frame_;
        // Publish the data
        rwPub_[0].publish(output2);
      }

      if(rwPub_[1].getNumSubscribers()>0){
        // Convert data type
        pcl::toPCLPointCloud2(*workspace_free_, *output);        
        // Convert to ROS data type
        pcl_conversions::fromPCL(*output, output2);
        output2.header.frame_id = robot_base_frame_;
        // Publish the data
        rwPub_[1].publish(output2);
      }
      
      // save the new images
      img_mat_vector_.clear();
      depthImgAsArgb_mat_vector_.clear();
      for(int i=0; i<img_msg_vector_.size(); i++){
        cv::Mat pImage;
        // convert to opencv mat
        cv_bridge::CvImage::Ptr cvImg;
        cvImg = cv_bridge::toCvCopy(img_msg_vector_[i]);
        cv::Mat mat_img = cvImg->image.clone();
        // save the mat image
        img_mat_vector_.push_back(mat_img);
        depthProcessor_[i]->setDepthMat(&img_mat_vector_[i]);
        
        // remove the robot links in the depth image
        depthProcessor_[i]->removeRobotLinksFromDepth(&mat_img,linkPointsInRobotbase);

        // convert depth image to rgb image
        pImage.create(mat_img.size(),CV_8UC3);
        depthProcessor_[i]->getDepthDataAsArgbMat(&pImage,(depthProcessor_[i]->_depthMat));
        depthImgAsArgb_mat_vector_.push_back(pImage);

        // update the robot workspace point cloud
        updateRobotWorkspaceCloud(i);
      }

      // check all the Occlusion points, of which the points[pcIndex].b > 0
      checkOcclusionPointsInWorkspaceCloud();
  }
  depthIMgMutex.unlock();
  
  
  // cout<<"updated"<<endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void extractor(const sensor_msgs::JointStateConstPtr & msg){

  if (first_frame_)
    return;

  // robot kinematics
  for (int i=0; i<7; i++){jointAngle[i] = msg->position[i];}
  rkine_->fkine(EndEffector,&phi,jointAngle);

  // robot kinematics
  getLinkPointsInRobotbase();

  // get the closed distance from the obstacles to the control points on the robot links
  getTheClosedDistance();

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 最耗时的函数，如何改进？？？？  可以并行计算，因为是遍历所有像素以及所有离线映射关系
void updateRobotWorkspaceCloud(int i){

  workspaceInitiate += 1;

  double xThreshold = (rWorkspace_.xMax - rWorkspace_.xMin)/rWorkspace_.xNum;
  double yThreshold = (rWorkspace_.yMax - rWorkspace_.yMin)/rWorkspace_.yNum;
  double zThreshold = (rWorkspace_.zMax - rWorkspace_.zMin)/rWorkspace_.zNum;

  double maxThreshold = MAX(xThreshold,yThreshold); maxThreshold = MAX(maxThreshold,zThreshold);
  // 遍历tempMat，更新所有非零位置
  int depthWidth= depthProcessor_[i]->_depthMat->cols;
  int depthHeight= depthProcessor_[i]->_depthMat->rows;
  for (int v=0; v<depthHeight; v++){
    float* pDepthNew =  depthProcessor_[i]->_depthMat->ptr<float>(v); // 新图像
    float* pDepthLast = NULL;   // 老图像
    if(depthProcessor_[i]->_lastDepthMat.cols == depthWidth && depthProcessor_[i]->_lastDepthMat.rows == depthHeight){
      pDepthLast = depthProcessor_[i]->_lastDepthMat.ptr<float>(v);
    }
    cv::Vec3b* pDepthRgbRow  = depthImgAsArgb_mat_vector_[i].ptr<cv::Vec3b>(v); // 彩色图像
    for (int u=0; u<depthWidth; u++){

      // 滤除机器人
      // int temp_window = 0; // 滑动窗口
      // bool breakFlag = false;
      // for(int y=-temp_window; y < temp_window+1; y++){
      //   int tempV = v + y;  if(tempV<0) {tempV = 0;} if(tempV >= depthHeight){tempV = depthHeight-1;}
      //   float* pDepthTemp = depthProcessor_[i]->_depthMat->ptr<float>(tempV);
      //   for(int x=-temp_window; x< temp_window+1; x++){
      //     int tempU = u + x;  if(tempU<0) {tempU = 0;} if(tempU >= depthWidth){tempU = depthWidth-1;}
      //     int tempI = tempU*depthHeight + tempV;
      //     if(depthProcessor_[i]->offlineRelation_[tempI].size()>0){
      //       int pointsNum = (depthProcessor_[i]->offlineRelation_[tempI].size()-2)/4;
      //       for(int j=0; j<pointsNum; j++){
      //         double d = depthProcessor_[i]->offlineRelation_[tempI][2+j*4];
      //         if(ABS(d-pDepthTemp[tempU]*1000)<5*maxThreshold ){
      //           double Pr[3] = {depthProcessor_[i]->offlineRelation_[tempI][3+j*4],
      //                           depthProcessor_[i]->offlineRelation_[tempI][4+j*4],
      //                           depthProcessor_[i]->offlineRelation_[tempI][5+j*4]};
      //           for(int k = 0; k<_robotFilter.getNumOfShapes(); k++){
      //             if((_robotFilter.getShape(k,jointAngle)).isPointInThisShape(Pr,jointAngle)){
      //                 pDepthNew[u] = -100;
      //                 breakFlag = true;
      //                 break;
      //             }
      //           }
      //         }
      //       }
      //     }
      //     if(breakFlag) break;
      //   }
      //   if(breakFlag) break;
      // }

      double d1 = (pDepthNew[u]*1000);
      if(d1 < 0) {d1 = 10000;}

      // // 变成RGB图片显示出来
      // if (d1 <= 8000 && d1 >= 300){
      //   unsigned  char b = (256-(d1)/32);
      //   pDepthRgbRow[u] = Vec3b(b,b,b);
      // }else{
      //   if (pDepthNew[u] < 0){
      //       pDepthRgbRow[u] = Vec3b(0,255,0);  
      //   }else{
      //       pDepthRgbRow[u] = Vec3b(0,0,0);
      //   }
      // }

      // // 改进： 帧差法降低计算量
      // if(pDepthLast != NULL && workspaceInitiate == WORKSPACE_INIT_DONE){
      //   workspaceInitiate = WORKSPACE_INIT_DONE;
      //   double deltaD = ABS((pDepthNew[u]*1000.0) - (pDepthLast[u]*1000.0));
      //   if(deltaD < 2*maxThreshold){
      //     continue;
      //   }
      // }

      // 更新环境模型
      // 在offlineRelation里找[u,v]对应的[d x y z]
      int offlineIndex = u*depthHeight + v;
      if(depthProcessor_[i]->offlineRelation_[offlineIndex].size() > 0){
        int pointsNum = (depthProcessor_[i]->offlineRelation_[offlineIndex].size()-2)/4;
        for(int j=0; j<pointsNum; j++){
          double d = depthProcessor_[i]->offlineRelation_[offlineIndex][2+j*4];
          int ix = (depthProcessor_[i]->offlineRelation_[offlineIndex][3+j*4] - rWorkspace_.xMin)/xThreshold;
          int iy = (depthProcessor_[i]->offlineRelation_[offlineIndex][4+j*4] - rWorkspace_.yMin)/yThreshold;
          int iz = (depthProcessor_[i]->offlineRelation_[offlineIndex][5+j*4] - rWorkspace_.zMin)/zThreshold;
          int pcIndex = iz + iy*rWorkspace_.zNum + ix*rWorkspace_.zNum*rWorkspace_.yNum;
          if(pcIndex > workspace_free_->size()&& pcIndex < 0)
            continue;

          if(d<d1){  //无障碍点
            workspace_free_->points[pcIndex].x = depthProcessor_[i]->offlineRelation_[offlineIndex][3+j*4]/1000.0;
            workspace_free_->points[pcIndex].y = depthProcessor_[i]->offlineRelation_[offlineIndex][4+j*4]/1000.0;
            workspace_free_->points[pcIndex].z = depthProcessor_[i]->offlineRelation_[offlineIndex][5+j*4]/1000.0;

            workspace_obstacle_->points[pcIndex].x = 0;
            workspace_obstacle_->points[pcIndex].y = 0;
            workspace_obstacle_->points[pcIndex].z = 0;
            workspace_obstacle_->points[pcIndex].b = 0;//所有自由点下该之为零，被遮挡点不为零
          }else{
            workspace_obstacle_->points[pcIndex].x = depthProcessor_[i]->offlineRelation_[offlineIndex][3+j*4]/1000.0;
            workspace_obstacle_->points[pcIndex].y = depthProcessor_[i]->offlineRelation_[offlineIndex][4+j*4]/1000.0;
            workspace_obstacle_->points[pcIndex].z = depthProcessor_[i]->offlineRelation_[offlineIndex][5+j*4]/1000.0;
            workspace_obstacle_->points[pcIndex].b = workspace_obstacle_->points[pcIndex].b + 1;
            if(ABS(d-d1) < maxThreshold){workspace_obstacle_->points[pcIndex].g=0;workspace_obstacle_->points[pcIndex].a=255;}
          }
        }
      }
    }
  }
  return;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void checkOcclusionPointsInWorkspaceCloud(){
  for(int pcIndex = 0; pcIndex <  workspace_free_->size(); pcIndex++){
    if(workspace_obstacle_->points[pcIndex].b == 0){
      continue;
    }
    if(workspace_obstacle_->points[pcIndex].b >= depthProcessor_.size()){
      workspace_obstacle_->points[pcIndex].b = 0;
      workspace_obstacle_->points[pcIndex].a = 255;
      workspace_free_->points[pcIndex].x = 0;
      workspace_free_->points[pcIndex].y = 0;
      workspace_free_->points[pcIndex].z = 0;
    }else{// wrong. some cells are not in the intersection view of all kinects. 
      workspace_obstacle_->points[pcIndex].b = 0;
      workspace_obstacle_->points[pcIndex].a = 0;
      workspace_free_->points[pcIndex].x = workspace_obstacle_->points[pcIndex].x;
      workspace_free_->points[pcIndex].y = workspace_obstacle_->points[pcIndex].y;
      workspace_free_->points[pcIndex].z = workspace_obstacle_->points[pcIndex].z;
      workspace_obstacle_->points[pcIndex].x = 0;
      workspace_obstacle_->points[pcIndex].y = 0;
      workspace_obstacle_->points[pcIndex].z = 0;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void getTheClosedDistance(){

  depthIMgMutex.lock();
  {
    for (int k = 2; k<POINTSNUM-1; k++){
      double Vector[3];
      double distance = 0.0;
      int Pdepth[3] = {0};
      int hybridOdepth[3];
      for (int i=0; i<depthProcessor_.size(); i++){
        if(depthProcessor_[i]->_depthMat == NULL){continue;}
        depthProcessor_[i]->projectToDepthFromRobotbase(Pdepth,linkPointsInRobotbase[k]);
        depthProcessor_[i]->getMinimumDistanceVector(distance,Vector,hybridOdepth,Pdepth,400);
        // draw the distance vector
        cv::Point p1, p2;
        p1.x = Pdepth[0]; p1.y = Pdepth[1];
        p2.x = hybridOdepth[0]; p2.y = hybridOdepth[1];
        cv::line(depthImgAsArgb_mat_vector_[i],p1,p2,Scalar(0,0,255),2);
        // double fx = depthProcessor_[i]->_intrinsic[0][0];
	      // double fy = depthProcessor_[i]->_intrinsic[1][1];
        // p1.x = depthProcessor_[i]->linkPointsInDepth[k][0];
		    // p1.y = depthProcessor_[i]->linkPointsInDepth[k][1];
        // int zd = depthProcessor_[i]->linkPointsInDepth[k][2];
        // p1.x = MAX(p1.x,0); p1.x = MIN(p1.x,depthProcessor_[i]->_depthMat->cols-1);
        // p1.y = MAX(p1.y,0); p1.y = MIN(p1.y,depthProcessor_[i]->_depthMat->rows-1);
        // if(zd <=0){continue;}
		    // int r = 1.2*0.5*linkPointsInRobotbase[k][3]/zd*(0.5*fx+0.5*fy); // 简化处理半径
        // r = MAX(r,1); r = MIN(r,400);
        //         cv::circle(depthImgAsArgb_mat_vector_[i],p1,r,Scalar(255,0,0),2);
      }
    }
  }
  depthIMgMutex.unlock();

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void getLinkPointsInRobotbase(){
  depthIMgMutex.lock();
  memset(linkPointsInRobotbase[BASE],0,sizeof(double)*3);
  linkPointsInRobotbase[BASE][3] = 230;

  memcpy(linkPointsInRobotbase[SHOULDER],rkine_->pp[SHOULDER],sizeof(double)*3);
  linkPointsInRobotbase[SHOULDER][3] = 230;

	memcpy(linkPointsInRobotbase[ELBOW],rkine_->pp[ELBOW],sizeof(double)*3);
  linkPointsInRobotbase[ELBOW][3] = 230;

	memcpy(linkPointsInRobotbase[WRIST],rkine_->pp[WRIST],sizeof(double)*3);
  linkPointsInRobotbase[WRIST][3] = 200;

  linkPointsInRobotbase[FLANGE][0] = EndEffector[0][3];
  linkPointsInRobotbase[FLANGE][1] = EndEffector[1][3];
  linkPointsInRobotbase[FLANGE][2] = EndEffector[2][3];
  linkPointsInRobotbase[FLANGE][3] = 70;

	linkPointsInRobotbase[5][0] = 0.2*linkPointsInRobotbase[2][0] + 0.8*linkPointsInRobotbase[1][0];
	linkPointsInRobotbase[5][1] = 0.2*linkPointsInRobotbase[2][1] + 0.8*linkPointsInRobotbase[1][1];
	linkPointsInRobotbase[5][2] = 0.2*linkPointsInRobotbase[2][2] + 0.8*linkPointsInRobotbase[1][2];
  linkPointsInRobotbase[5][3] = 180;

	linkPointsInRobotbase[6][0] = 0.3*linkPointsInRobotbase[2][0] + 0.7*linkPointsInRobotbase[1][0];
	linkPointsInRobotbase[6][1] = 0.3*linkPointsInRobotbase[2][1] + 0.7*linkPointsInRobotbase[1][1];
	linkPointsInRobotbase[6][2] = 0.3*linkPointsInRobotbase[2][2] + 0.7*linkPointsInRobotbase[1][2];
  linkPointsInRobotbase[6][3] = 160;

	linkPointsInRobotbase[7][0] = 0.4*linkPointsInRobotbase[2][0] + 0.6*linkPointsInRobotbase[1][0];
	linkPointsInRobotbase[7][1] = 0.4*linkPointsInRobotbase[2][1] + 0.6*linkPointsInRobotbase[1][1];
	linkPointsInRobotbase[7][2] = 0.4*linkPointsInRobotbase[2][2] + 0.6*linkPointsInRobotbase[1][2];
  linkPointsInRobotbase[7][3] = 150;

	linkPointsInRobotbase[8][0] = 0.5*linkPointsInRobotbase[2][0] + 0.5*linkPointsInRobotbase[1][0];
	linkPointsInRobotbase[8][1] = 0.5*linkPointsInRobotbase[2][1] + 0.5*linkPointsInRobotbase[1][1];
	linkPointsInRobotbase[8][2] = 0.5*linkPointsInRobotbase[2][2] + 0.5*linkPointsInRobotbase[1][2];
  linkPointsInRobotbase[8][3] = 150;

	linkPointsInRobotbase[9][0] = 0.6*linkPointsInRobotbase[2][0] + 0.4*linkPointsInRobotbase[1][0];
	linkPointsInRobotbase[9][1] = 0.6*linkPointsInRobotbase[2][1] + 0.4*linkPointsInRobotbase[1][1];
	linkPointsInRobotbase[9][2] = 0.6*linkPointsInRobotbase[2][2] + 0.4*linkPointsInRobotbase[1][2];
  linkPointsInRobotbase[9][3] = 140;

	linkPointsInRobotbase[10][0] = 0.7*linkPointsInRobotbase[2][0] + 0.3*linkPointsInRobotbase[1][0];
	linkPointsInRobotbase[10][1] = 0.7*linkPointsInRobotbase[2][1] + 0.3*linkPointsInRobotbase[1][1];
	linkPointsInRobotbase[10][2] = 0.7*linkPointsInRobotbase[2][2] + 0.3*linkPointsInRobotbase[1][2];
  linkPointsInRobotbase[10][3] = 160;

	linkPointsInRobotbase[11][0] = 0.8*linkPointsInRobotbase[2][0] + 0.2*linkPointsInRobotbase[1][0];
	linkPointsInRobotbase[11][1] = 0.8*linkPointsInRobotbase[2][1] + 0.2*linkPointsInRobotbase[1][1];
	linkPointsInRobotbase[11][2] = 0.8*linkPointsInRobotbase[2][2] + 0.2*linkPointsInRobotbase[1][2];
  linkPointsInRobotbase[11][3] = 180;

	linkPointsInRobotbase[12][0] = 0.2*linkPointsInRobotbase[3][0] + 0.8*linkPointsInRobotbase[2][0];
	linkPointsInRobotbase[12][1] = 0.2*linkPointsInRobotbase[3][1] + 0.8*linkPointsInRobotbase[2][1];
	linkPointsInRobotbase[12][2] = 0.2*linkPointsInRobotbase[3][2] + 0.8*linkPointsInRobotbase[2][2];
  linkPointsInRobotbase[12][3] = 180;

	linkPointsInRobotbase[13][0] = 0.3*linkPointsInRobotbase[3][0] + 0.7*linkPointsInRobotbase[2][0];
	linkPointsInRobotbase[13][1] = 0.3*linkPointsInRobotbase[3][1] + 0.7*linkPointsInRobotbase[2][1];
	linkPointsInRobotbase[13][2] = 0.3*linkPointsInRobotbase[3][2] + 0.7*linkPointsInRobotbase[2][2];
  linkPointsInRobotbase[13][3] = 160;

	linkPointsInRobotbase[14][0] = 0.4*linkPointsInRobotbase[3][0] + 0.6*linkPointsInRobotbase[2][0];
	linkPointsInRobotbase[14][1] = 0.4*linkPointsInRobotbase[3][1] + 0.6*linkPointsInRobotbase[2][1];
	linkPointsInRobotbase[14][2] = 0.4*linkPointsInRobotbase[3][2] + 0.6*linkPointsInRobotbase[2][2];
  linkPointsInRobotbase[14][3] = 150;

	linkPointsInRobotbase[15][0] = 0.5*linkPointsInRobotbase[3][0] + 0.5*linkPointsInRobotbase[2][0];
	linkPointsInRobotbase[15][1] = 0.5*linkPointsInRobotbase[3][1] + 0.5*linkPointsInRobotbase[2][1];
	linkPointsInRobotbase[15][2] = 0.5*linkPointsInRobotbase[3][2] + 0.5*linkPointsInRobotbase[2][2];
  linkPointsInRobotbase[15][3] = 150;

	linkPointsInRobotbase[16][0] = 0.6*linkPointsInRobotbase[3][0] + 0.4*linkPointsInRobotbase[2][0];
	linkPointsInRobotbase[16][1] = 0.6*linkPointsInRobotbase[3][1] + 0.4*linkPointsInRobotbase[2][1];
	linkPointsInRobotbase[16][2] = 0.6*linkPointsInRobotbase[3][2] + 0.4*linkPointsInRobotbase[2][2];
  linkPointsInRobotbase[16][3] = 140;

	linkPointsInRobotbase[17][0] = 0.7*linkPointsInRobotbase[3][0] + 0.3*linkPointsInRobotbase[2][0];
	linkPointsInRobotbase[17][1] = 0.7*linkPointsInRobotbase[3][1] + 0.3*linkPointsInRobotbase[2][1];
	linkPointsInRobotbase[17][2] = 0.7*linkPointsInRobotbase[3][2] + 0.3*linkPointsInRobotbase[2][2];
  linkPointsInRobotbase[17][3] = 150;

	linkPointsInRobotbase[18][0] = 0.8*linkPointsInRobotbase[3][0] + 0.2*linkPointsInRobotbase[2][0];
	linkPointsInRobotbase[18][1] = 0.8*linkPointsInRobotbase[3][1] + 0.2*linkPointsInRobotbase[2][1];
	linkPointsInRobotbase[18][2] = 0.8*linkPointsInRobotbase[3][2] + 0.2*linkPointsInRobotbase[2][2];
  linkPointsInRobotbase[18][3] = 160;

  linkPointsInRobotbase[19][0] = 0.9*linkPointsInRobotbase[3][0] + 0.1*linkPointsInRobotbase[2][0];
	linkPointsInRobotbase[19][1] = 0.9*linkPointsInRobotbase[3][1] + 0.1*linkPointsInRobotbase[2][1];
	linkPointsInRobotbase[19][2] = 0.9*linkPointsInRobotbase[3][2] + 0.1*linkPointsInRobotbase[2][2];
  linkPointsInRobotbase[19][3] = 180;

	linkPointsInRobotbase[20][0] = 0.7*linkPointsInRobotbase[4][0] + 0.3*linkPointsInRobotbase[3][0];
	linkPointsInRobotbase[20][1] = 0.7*linkPointsInRobotbase[4][1] + 0.3*linkPointsInRobotbase[3][1];
	linkPointsInRobotbase[20][2] = 0.7*linkPointsInRobotbase[4][2] + 0.3*linkPointsInRobotbase[3][2];
  linkPointsInRobotbase[20][3] = 100;

	linkPointsInRobotbase[21][0] = 0.3*linkPointsInRobotbase[4][0] + 0.7*linkPointsInRobotbase[3][0];
	linkPointsInRobotbase[21][1] = 0.3*linkPointsInRobotbase[4][1] + 0.7*linkPointsInRobotbase[3][1];
	linkPointsInRobotbase[21][2] = 0.3*linkPointsInRobotbase[4][2] + 0.7*linkPointsInRobotbase[3][2];
  linkPointsInRobotbase[21][3] = 120;

	linkPointsInRobotbase[22][0] = 0.5*linkPointsInRobotbase[SHOULDER][0];
	linkPointsInRobotbase[22][1] = 0.5*linkPointsInRobotbase[SHOULDER][1];
	linkPointsInRobotbase[22][2] = 0.5*linkPointsInRobotbase[SHOULDER][2];
  linkPointsInRobotbase[22][3] = 200;

  depthIMgMutex.unlock();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void printRobotStates(){
  cunt ++;
  if (cunt == CYCLE)
  {
    cunt = 0;
    //cout.precision(4);
    cout<<fixed<<"the robot joint state: "    << jointAngle[0] <<"\t"<< jointAngle[1] <<"\t"<< jointAngle[2] 
       <<"\t"<< jointAngle[3] <<"\t"<< jointAngle[4] <<"\t"<< jointAngle[5] <<"\t"<< jointAngle[6] <<endl;

    cout<<"\nthe robot end effector pose: "   <<endl;
    for (int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
          cout<<fixed<<EndEffector[i][j]<<"\t";
        }
        cout<<endl;
    }

    for (int i=0; i<depthProcessor_.size(); i++){
       cout<< "\nthe transfomation between "<<robot_base_frame_<<
              " and "<<depthProcessor_[i]->_frame_id<<endl;
       for (int k=0; k<4; k++){
         for (int j=0; j<4; j++){
           cout<<fixed<<depthProcessor_[i]->_extrinsic[k][j]<<"\t";
         }
         cout<<endl;
       }

       cout<<endl;
       for (int k=0; k<4; k++){
         for (int j=0; j<4; j++){
           cout<<fixed<<depthProcessor_[i]->_extrinsicInv[k][j]<<"\t";
         }
         cout<<endl;
       }
       cout<<endl;
    }

    cout<<"the link point in robot base frame: "<<endl;
    cout<<fixed<<linkPointsInRobotbase[BASE][0]<<"\t"<<linkPointsInRobotbase[BASE][1]<<"\t"<<linkPointsInRobotbase[BASE][2]<<endl;
    cout<<fixed<<linkPointsInRobotbase[SHOULDER][0]<<"\t"<<linkPointsInRobotbase[SHOULDER][1]<<"\t"<<linkPointsInRobotbase[SHOULDER][2]<<endl;
    cout<<fixed<<linkPointsInRobotbase[ELBOW][0]<<"\t"<<linkPointsInRobotbase[ELBOW][1]<<"\t"<<linkPointsInRobotbase[ELBOW][2]<<endl;
    cout<<fixed<<linkPointsInRobotbase[WRIST][0]<<"\t"<<linkPointsInRobotbase[WRIST][1]<<"\t"<<linkPointsInRobotbase[WRIST][2]<<endl;
    cout<<fixed<<linkPointsInRobotbase[FLANGE][0]<<"\t"<<linkPointsInRobotbase[FLANGE][1]<<"\t"<<linkPointsInRobotbase[FLANGE][2]<<endl;

    for (int i=0; i< depthProcessor_.size();i++){
      cout<<"the link point in "<<depthProcessor_[i]->_frame_id<<endl;

      cout<<fixed<<depthProcessor_[i]->baseShoulderElbowWristFlangeInDepth[BASE][0]<<"\t"
      <<depthProcessor_[i]->baseShoulderElbowWristFlangeInDepth[BASE][1] <<"\t"
      <<depthProcessor_[i]->baseShoulderElbowWristFlangeInDepth[BASE][2]<<endl;

      cout<<fixed<<depthProcessor_[i]->baseShoulderElbowWristFlangeInDepth[SHOULDER][0]<<"\t"
      <<depthProcessor_[i]->baseShoulderElbowWristFlangeInDepth[SHOULDER][1]<<"\t"
      <<depthProcessor_[i]->baseShoulderElbowWristFlangeInDepth[SHOULDER][2]<<endl;

      cout<<fixed<<depthProcessor_[i]->baseShoulderElbowWristFlangeInDepth[ELBOW][0]<<"\t"
      <<depthProcessor_[i]->baseShoulderElbowWristFlangeInDepth[ELBOW][1]<<"\t"
      <<depthProcessor_[i]->baseShoulderElbowWristFlangeInDepth[ELBOW][2]<<endl;

      cout<<fixed<<depthProcessor_[i]->baseShoulderElbowWristFlangeInDepth[WRIST][0]<<"\t"
      <<depthProcessor_[i]->baseShoulderElbowWristFlangeInDepth[WRIST][1]<<"\t"
      <<depthProcessor_[i]->baseShoulderElbowWristFlangeInDepth[WRIST][2]<<endl;

      cout<<fixed<<depthProcessor_[i]->baseShoulderElbowWristFlangeInDepth[FLANGE][0]<<"\t"
      <<depthProcessor_[i]->baseShoulderElbowWristFlangeInDepth[FLANGE][1]<<"\t"
      <<depthProcessor_[i]->baseShoulderElbowWristFlangeInDepth[FLANGE][2]<<endl;
    }

    cout<<"\n\n\n"<<endl;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void getTheTransformation(){
  tf::TransformListener trans_listener;
  tf::StampedTransform stampedTransform;

  for (int i=0; i<img_msg_vector_.size();i++){

    // get the tf between knects and the robot base
    try{
      trans_listener.waitForTransform(robot_base_frame_, img_msg_vector_[i]->header.frame_id, ros::Time(0), ros::Duration(10.0));
      trans_listener.lookupTransform(robot_base_frame_, img_msg_vector_[i]->header.frame_id, ros::Time(0), stampedTransform);
    }
    catch(tf::TransformException &ex){
      cout << ex.what() << endl;
      ROS_ERROR("%s", ex.what());
    }
    tf::vectorTFToEigen(stampedTransform.getOrigin(), transforms_[i].translation);      
    tf::quaternionTFToEigen(stampedTransform.getRotation(), transforms_[i].rotation);

    // print out the tf
    cout.precision(3);
    cout<<"the transformation between "<<robot_base_frame_<< " and "<< img_msg_vector_[i]->header.frame_id<<" is: "<<endl;
    cout<<transforms_[i].translation[0]<<"\t"<<transforms_[i].translation[1]<<"\t"<<transforms_[i].translation[2]<<endl;
    cout<<transforms_[i].rotation.x()<<"\t"<<transforms_[i].rotation.y()<<"\t"
        <<transforms_[i].rotation.z()<<"\t"<<transforms_[i].rotation.w()<<"\t"<<endl;

    // change the format of the tf
    depthProcessor_[i]->_frame_id = img_msg_vector_[i]->header.frame_id;

    double temp[4][4] = {{0.0}}; temp[3][3] = 1.0;
    for (int k=0; k<3; k++) temp[k][3] = transforms_[i].translation[k]*1000.0;
    Eigen::Matrix3d R = transforms_[i].rotation.toRotationMatrix();
    for (int k=0; k<3; k++){
      for (int j=0; j<3; j++){
          temp[k][j] = R(k,j);
      }
    }

    double tempInv[4][4] = {{0.0}};
    CMatrix4d tempMatrix; 
    for (int k=0; k<4; k++){
      for (int j=0; j<4; j++){
        tempMatrix.SetElement(k,j,temp[k][j]);
      }
    }
    CMatrixXd tempMatrix1 = tempMatrix.inv();
    for (int k=0; k<4; k++){
      for (int j=0; j<4; j++){
        tempInv[k][j] = tempMatrix1.GetElement(k,j);
      }
    }

    // save the tf
    depthProcessor_[i]->setCameraExtrinsic(temp,tempInv);
    depthProcessor_[i]->setThreshold(LINKWIDTH);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void initWorkspaceCloud(){

  // Setup the cloud pointer
  workspace_obstacle_.reset (new PointCloudT);
  workspace_free_.reset(new PointCloudT);

  int num = rWorkspace_.xNum * rWorkspace_.yNum * rWorkspace_.zNum;

  // The number of points in the cloud
  workspace_obstacle_->points.resize (num);
  workspace_free_->points.resize(num);

  // Fill the cloud with some points
  for (size_t i = 0; i < workspace_obstacle_->points.size (); ++i){
      workspace_obstacle_->points[i].x = gridCenter_[i][0]/1000.0;
      workspace_obstacle_->points[i].y = gridCenter_[i][1]/1000.0;
      workspace_obstacle_->points[i].z = gridCenter_[i][2]/1000.0;
      workspace_obstacle_->points[i].r = 255;
      workspace_obstacle_->points[i].g = 0;
      workspace_obstacle_->points[i].b = 0;
      workspace_obstacle_->points[i].a = 0;  // 0：不显示盲区，255或直接屏蔽该行：显示盲区

      workspace_free_->points[i].x = 0;
      workspace_free_->points[i].y = 0;
      workspace_free_->points[i].z = 0;
      workspace_free_->points[i].r = 0;
      workspace_free_->points[i].g = 255;
      workspace_free_->points[i].b = 0;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void saveTheCameraParameter( ){
  for(int i=0; i < depthProcessor_.size(); i++){
    string fileName =  ros::package::getPath("kinects_obstacle_extractor")
                          + "/data/" + depthProcessor_[i]->_frame_id  + ".txt";

    FILE* file = fopen(fileName.c_str(),"w");
    {
      fprintf(file,"\n#############################################################################\n");
      fprintf(file,"# the intrinsic matrix is:\n");
      for (int k=0;k<3; k++){
        for (int j=0;j<3;j++){
          fprintf(file,"%.4f ",depthProcessor_[i]->_intrinsic[k][j]);
        }
        fprintf(file,"\n");
      }

      fprintf(file,"\n#############################################################################\n");
      fprintf(file,"# the extrinsic matrix is:\n");
      for (int k=0;k<4; k++){
        for (int j=0;j<4;j++){
          fprintf(file,"%.4f ",depthProcessor_[i]->_extrinsic[k][j]);
        }
        fprintf(file,"\n");
      }

      fprintf(file,"\n#############################################################################\n");
      fprintf(file,"# the extrinsic inv matrix is:\n");
      for (int k=0;k<4; k++){
        for (int j=0;j<4;j++){
          fprintf(file,"%.4f ",depthProcessor_[i]->_extrinsicInv[k][j]);
        }
        fprintf(file,"\n");
      }
    }
    fclose(file);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool getRobotWorkSpaceGrid(){
  double xResolution = (rWorkspace_.xMax - rWorkspace_.xMin) / rWorkspace_.xNum;
  double yResolution = (rWorkspace_.yMax - rWorkspace_.yMin) / rWorkspace_.yNum;
  double zResolution = (rWorkspace_.zMax - rWorkspace_.zMin) / rWorkspace_.zNum;

  gridCenter_.clear();
  for(int i=0; i<rWorkspace_.xNum; i++){
    cout<<"workspace grid: "<<(i+0.0)/(rWorkspace_.xNum + 0.0)<<endl;
    for (int j=0; j<rWorkspace_.yNum; j++){
      for (int k=0; k<rWorkspace_.zNum; k++){
          vector<double> temp;temp.clear();
          temp.push_back(rWorkspace_.xMin + xResolution*(i+0.5));
          temp.push_back(rWorkspace_.yMin + yResolution*(j+0.5));
          temp.push_back(rWorkspace_.zMin + zResolution*(k+0.5));
          gridCenter_.push_back(temp);
      }
    }
  }

  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool getOfflineRelation(){

  if(!reProjectWorkspace_){
    // 直接读取文件, 有问题
    for(int i=0; i<depthProcessor_.size(); i++){
      string fileName =  ros::package::getPath("kinects_obstacle_extractor")
                            + "/data/" + depthProcessor_[i]->_frame_id  + ".txt";
      cout<<"reading the robot work space"<<endl;
      FILE* file=fopen(fileName.c_str(),"r");
      char tempC;
      char tempS[1024];
      bool start = false;
      while(!feof(file)){
        fscanf(file,"%c",&tempC);
        if(tempC == '#'){
          fscanf(file,"%s",tempS);
          cout<<tempS<<endl;
          start = true;
        }
        if(start){
          double tempD;
          fscanf(file,"%lf",&tempD);
          cout<<tempD<<endl;
        }
      }
      fclose(file);
    }
    return true;
  }

  // 计算映射关系
  for(int i=0; i<depthProcessor_.size(); i++){
    int width = img_msg_vector_[i]->width;
    int height = img_msg_vector_[i]->height;
    depthProcessor_[i]->offlineRelation_.clear();
    depthProcessor_[i]->offlineRelation_.resize(width*height + 1);

    double index = 0;
    for(int j=0; j<gridCenter_.size(); j++){
      if ((j+0.0)/(gridCenter_.size() + 0.0) - index > 0.01){
        cout<<"reProject robot workspace: "<<(j+0.0)/(gridCenter_.size() + 0.0)<<endl;
        index = (j+0.0)/(gridCenter_.size() + 0.0);
      }
      
      // project to depth space
      int Pdepth[3] = {0};
      double pBase[3] = {0.0};
      pBase[0] = gridCenter_[j][0];pBase[1] = gridCenter_[j][1];pBase[2] = gridCenter_[j][2];
      depthProcessor_[i]->projectToDepthFromRobotbase(Pdepth,pBase);

      // save te relation
      int u = Pdepth[0]; int v = Pdepth[1];
      if (0<u && u<=width && 0<v && v<=height){
        vector<double> temp = depthProcessor_[i]->offlineRelation_[v-1 + (u-1)*height];
        if (temp.size() < 1){
          temp.push_back(u);
          temp.push_back(v);
        }
        temp.push_back(Pdepth[2]);
        temp.push_back(pBase[0]);
        temp.push_back(pBase[1]);
        temp.push_back(pBase[2]);
        depthProcessor_[i]->offlineRelation_[v-1 + (u-1)*height] = temp;
      }else{
        // vector<double> temp = depthProcessor_[i]->offlineRelation_[width*height];
        // temp.push_back(pBase[0]);
        // temp.push_back(pBase[1]);
        // temp.push_back(pBase[2]);
        // depthProcessor_[i]->offlineRelation_[width*height] = temp;
      }
    }

    // save the relation to file
    string fileName =  ros::package::getPath("kinects_obstacle_extractor")
                          + "/data/" + depthProcessor_[i]->_frame_id  + ".txt";
    FILE* file = fopen(fileName.c_str(),"a+");
    fprintf(file,"\n#############################################################################\n");
    fprintf(file,"# the offline relation is:\n");
    for(int v =0; v<depthProcessor_[i]->offlineRelation_.size(); v++){
      if (depthProcessor_[i]->offlineRelation_[v].size() > 0){
        fprintf(file,"%lu ",depthProcessor_[i]->offlineRelation_[v].size());
        for(int u=0; u<depthProcessor_[i]->offlineRelation_[v].size(); u++){
          if (u <2){
            fprintf(file,"%.0f ",depthProcessor_[i]->offlineRelation_[v][u]);
          }else{
            fprintf(file,"%.4f ",depthProcessor_[i]->offlineRelation_[v][u]);
          }
        }
        fprintf(file,"\n");
      }
    }
    fclose(file);
  }
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int registSubscriberCallbacks(int nb_in_topics,ros::NodeHandle nh){
    // Declaration Ros Subscribers
  vector<boost::shared_ptr<message_filters::Subscriber<ImgMsg> > > subs(nb_in_topics);
  for (int i=0; i<nb_in_topics; i++)
    subs[i] = boost::shared_ptr<message_filters::Subscriber<ImgMsg> >
              (new message_filters::Subscriber<ImgMsg>(nh, in_depthImgs_topics_[i],1));

  // registerCallbacks
  switch( nb_in_topics ){
    case 0: 
      ROS_ERROR("No topics to extract provided");
      return 1; break;

    case 1:{
      ros::Subscriber img_sub = nh.subscribe<ImgMsg>(in_depthImgs_topics_[0], 1, callback1);
      ros::spin();
      break;
    }

    case 2:{ 
      message_filters::Synchronizer<MySyncPolicy2> sync(MySyncPolicy2(10), *subs[0], *subs[1]);
      sync.registerCallback(boost::bind(&callback2, _1, _2));
      cout <<"callback2"<<endl;
      ros::spin();
      break;
    }

    case 3:{
      message_filters::Synchronizer<MySyncPolicy3> sync(MySyncPolicy3(10), *subs[0], *subs[1], *subs[2]);
      sync.registerCallback(boost::bind(&callback3, _1, _2, _3));
      ros::spin();
      break;
    }
    case 4:{
      message_filters::Synchronizer<MySyncPolicy4> sync(MySyncPolicy4(10), *subs[0], *subs[1], *subs[2],
                                                                           *subs[3]);
      sync.registerCallback(boost::bind(&callback4, _1, _2, _3, _4));
      ros::spin();
      break;
    }
    case 5:{
      message_filters::Synchronizer<MySyncPolicy5> sync(MySyncPolicy5(10), *subs[0], *subs[1], *subs[2], 
                                                                           *subs[3], *subs[4]);
      sync.registerCallback(boost::bind(&callback5, _1, _2, _3, _4, _5));
      ros::spin();
      break;
    }
    case 6:{
      message_filters::Synchronizer<MySyncPolicy6> sync(MySyncPolicy6(10), *subs[0], *subs[1], *subs[2],
                                                                           *subs[3], *subs[4], *subs[5]);
      sync.registerCallback(boost::bind(&callback6, _1, _2, _3, _4, _5, _6));
      ros::spin();
      break;
    }
    case 7:{
      message_filters::Synchronizer<MySyncPolicy7> sync(MySyncPolicy7(10), *subs[0], *subs[1], *subs[2],
                                                                           *subs[3], *subs[4], *subs[5],
                                                                           *subs[6]);
      sync.registerCallback(boost::bind(&callback7, _1, _2, _3, _4, _5, _6, _7));
      ros::spin();
      break;
    }
    case 8:{
      message_filters::Synchronizer<MySyncPolicy8> sync(MySyncPolicy8(10), *subs[0], *subs[1], *subs[2],
                                                                           *subs[3], *subs[4], *subs[5], 
                                                                           *subs[6], *subs[7]);
      sync.registerCallback(boost::bind(&callback8, _1, _2, _3, _4, _5, _6, _7, _8));
      ros::spin();
      break;
    }
    case 9:{
      message_filters::Synchronizer<MySyncPolicy9> sync(MySyncPolicy9(10), *subs[0], *subs[1], *subs[2],
                                                                           *subs[3], *subs[4], *subs[5],
                                                                           *subs[6], *subs[7], *subs[8]);
      sync.registerCallback(boost::bind(&callback9, _1, _2, _3, _4, _5, _6, _7, _8, _9));
      ros::spin();
      break;
    }
    default:
      ROS_ERROR("Too many topics provided. Current limitation is of 9 kinects");
      return 1;
      break;
  }

  return 0;
}

void callback1(const ImgMsg::ConstPtr& msg){
  img_msg_vector_.clear();
  img_msg_vector_.push_back(msg);
  imgMatSaver();
}

void callback2(const ImgMsg::ConstPtr& msg, const ImgMsg::ConstPtr& msg2){
  img_msg_vector_.clear();
  img_msg_vector_.push_back(msg);
  img_msg_vector_.push_back(msg2);
  imgMatSaver();
}

void callback3(const ImgMsg::ConstPtr& msg, const ImgMsg::ConstPtr& msg2, const ImgMsg::ConstPtr& msg3 ){
  img_msg_vector_.clear();
  img_msg_vector_.push_back(msg);
  img_msg_vector_.push_back(msg2);
  img_msg_vector_.push_back(msg3);
  imgMatSaver();
}
void callback4(const ImgMsg::ConstPtr& msg, const ImgMsg::ConstPtr& msg2, const ImgMsg::ConstPtr& msg3,
               const ImgMsg::ConstPtr& msg4 ){
  img_msg_vector_.clear();
  img_msg_vector_.push_back(msg);
  img_msg_vector_.push_back(msg2);
  img_msg_vector_.push_back(msg3);
  img_msg_vector_.push_back(msg4);
  imgMatSaver();
}
void callback5(const ImgMsg::ConstPtr& msg, const ImgMsg::ConstPtr& msg2, const ImgMsg::ConstPtr& msg3,
               const ImgMsg::ConstPtr& msg4, const ImgMsg::ConstPtr& msg5 ){
  img_msg_vector_.clear();
  img_msg_vector_.push_back(msg);
  img_msg_vector_.push_back(msg2);
  img_msg_vector_.push_back(msg3);
  img_msg_vector_.push_back(msg4);
  img_msg_vector_.push_back(msg5);
  imgMatSaver();
}
void callback6(const ImgMsg::ConstPtr& msg, const ImgMsg::ConstPtr& msg2, const ImgMsg::ConstPtr& msg3, 
               const ImgMsg::ConstPtr& msg4, const ImgMsg::ConstPtr& msg5, const ImgMsg::ConstPtr& msg6 ){
  img_msg_vector_.clear();
  img_msg_vector_.push_back(msg);
  img_msg_vector_.push_back(msg2);
  img_msg_vector_.push_back(msg3);
  img_msg_vector_.push_back(msg4);
  img_msg_vector_.push_back(msg5);
  img_msg_vector_.push_back(msg6);
  imgMatSaver();
}
void callback7(const ImgMsg::ConstPtr& msg, const ImgMsg::ConstPtr& msg2, const ImgMsg::ConstPtr& msg3, 
               const ImgMsg::ConstPtr& msg4, const ImgMsg::ConstPtr& msg5, const ImgMsg::ConstPtr& msg6,
               const ImgMsg::ConstPtr& msg7 ){
  img_msg_vector_.clear();
  img_msg_vector_.push_back(msg);
  img_msg_vector_.push_back(msg2);
  img_msg_vector_.push_back(msg3);
  img_msg_vector_.push_back(msg4);
  img_msg_vector_.push_back(msg5);
  img_msg_vector_.push_back(msg6);
  img_msg_vector_.push_back(msg7);
  imgMatSaver();
}
void callback8(const ImgMsg::ConstPtr& msg, const ImgMsg::ConstPtr& msg2, const ImgMsg::ConstPtr& msg3,
               const ImgMsg::ConstPtr& msg4, const ImgMsg::ConstPtr& msg5, const ImgMsg::ConstPtr& msg6,
               const ImgMsg::ConstPtr& msg7, const ImgMsg::ConstPtr& msg8 ){
  img_msg_vector_.clear();
  img_msg_vector_.push_back(msg);
  img_msg_vector_.push_back(msg2);
  img_msg_vector_.push_back(msg3);
  img_msg_vector_.push_back(msg4);
  img_msg_vector_.push_back(msg5);
  img_msg_vector_.push_back(msg6);
  img_msg_vector_.push_back(msg7);
  img_msg_vector_.push_back(msg8);
  imgMatSaver();
}
void callback9(const ImgMsg::ConstPtr& msg, const ImgMsg::ConstPtr& msg2, const ImgMsg::ConstPtr& msg3, 
               const ImgMsg::ConstPtr& msg4, const ImgMsg::ConstPtr& msg5, const ImgMsg::ConstPtr& msg6, 
               const ImgMsg::ConstPtr& msg7, const ImgMsg::ConstPtr& msg8, const ImgMsg::ConstPtr& msg9 ){
  img_msg_vector_.clear();
  img_msg_vector_.push_back(msg);
  img_msg_vector_.push_back(msg2);
  img_msg_vector_.push_back(msg3);
  img_msg_vector_.push_back(msg4);
  img_msg_vector_.push_back(msg5);
  img_msg_vector_.push_back(msg6);
  img_msg_vector_.push_back(msg7);
  img_msg_vector_.push_back(msg8);
  img_msg_vector_.push_back(msg9);
  imgMatSaver();
}