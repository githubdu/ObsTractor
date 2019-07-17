


#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
// #include <cv.h>
#include <ros/ros.h>
#include "robotMath.h"

using namespace cv;

#define POINTSNUM	(23)
#define LINKWIDTH	(125)

class depth_processor
{
	
public:
	// get the offline project relation
	vector<vector<double> > offlineRelation_;

public:

    // frame id
	string _frame_id;

	// depth image
	Mat*	_depthMat;
	Mat    _lastDepthMat;

	// intrinsic of the camera
	double  _intrinsic[3][3];

	// extrinsic of the camera
	double  _extrinsic[4][4];
	double  _extrinsicInv[4][4];

    // threshold of the distance
	double _threshold;

	// define the region of surveillance, S
	// in which, the objects are considerate as obstacles
	// each control point has a S defined by a cube of side 2*rho
	// centered at this control point
	// S is in sensor depth space, [-u, u] [-v v] [-d d]
	double _rho;	// mm

public:
	// link width for robot filter
	double linkWidth;

	// object distance closed to the control points
	// the distance is in sensor Cartesian space
	// calculated in sensor depth space
	double Distance[POINTSNUM];

	// mean object vector to the control points
	// the vector is in sensor Cartesian space
	// calculated in sensor depth space
	double ObstacleVector[POINTSNUM][3];

	// closed objects to the control points
	// the objects are in sensor depth space
	int obstaclePointsInDepth[POINTSNUM][3];

	// project link points on the robot to sensor depth space
	int linkPointsInDepth[POINTSNUM][3];

	int baseShoulderElbowWristFlangeInDepth[5][3];
	
public:

	depth_processor();
	~depth_processor();

	// set the depth image to be processed
	bool setDepthMat(Mat* depthMat);

	// set the extrinsic of the sensor or camera
	bool setCameraExtrinsic(double extrinsic[4][4],double extrinsicInv[4][4]);

	// set the threshold of the distance
	bool setThreshold(double threshold);

public:
	// coordinate transformation
	bool projectToDepthFromRobotbase(int Pdepth[3],double pointInWord[3])const;
	bool projectToRobotbaseFromDepth(double pointInWord[3],int Pdepth[3])const;	
	bool projectToDepthFromCamera(int Pdepth[3],double pointInCamera[3])const;
	bool projectToCameraFromDepth(double pointInCamera[3],int Pdepth[3])const;
	bool projectToRobotbaseFromCamera(double pointInWord[3],double pointInCamera[3])const;
	bool projectToCameraFromRobotbase(double pointInCamera[3],double pointInWord[3])const;

public:
	// distance and obstacles vector calculation in depth image
	bool checkRegionOfSurveillance(int Odepth[3],int S[3][2])const;
	bool getRegionOfSurveillance(int S[3][2],int Pdepth[3],double rho)const;
	bool getDistanceVectorInDepthSpace(double& Distance, double Vector[3],int Odepth[3], int Pdepth[3])const;
	bool getMinimumDistanceVector(double& Distance, double Vector[3],int miniOdepth[3],int Pdepth[3],double rho)const;
	bool getMeanDistanceVector(double& Distance, double Vector[3],int meanOdepth[3], int Pdepth[3], double rho)const;
	bool getHybridDistanceVector(double& Distance, double Vector[3],int hybridOdepth[3], int Pdepth[3],double rho)const;

public:
	// convert the depht image into rgb image
	void getDepthDataAsArgbMat(cv::Mat* pImage, cv::Mat* pDepthOrigionData);

public:
	// project the control points of the robot the sensor depth space
	bool getLinkPointsInDepth(double linkPointsInRobotbase[][4]);

	// remove the robot links from the depth image	
	// When the control point belongs to a real object which is also detected by the depth
	// sensor, it is important to remove it from the depth image. Without removing the
	// control point, the minimum distance to the object would always be equal to zero.
	bool removeRobotLinksFromDepth(Mat* depthMat,double linkPointsInRobotbase[][4]);


private:
	double PointToSegDist(int point[2],int A[2],int B[2])const;

};