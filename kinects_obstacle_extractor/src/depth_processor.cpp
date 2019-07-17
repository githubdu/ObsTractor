


#include "../include/kinematics.h"
#include "../include/depth_processor.h"

/**********************************************************************************************************
// constructor of the depth_processor
***********************************************************************************************************/
depth_processor::depth_processor()
{
	_rho = 300;
	_depthMat = NULL;
	_frame_id = "unknown";
	_threshold = LINKWIDTH;

	memset(_intrinsic,0,sizeof(_intrinsic));
	memset(_extrinsic,0,sizeof(_extrinsic));
	memset(_extrinsicInv,0,sizeof(_extrinsicInv));

	double intrinsic[3][3] = 
	{
		{554.2547,	0,			320},
		{0,			554.2547,	240},
		{0,			0,			1}
		// {524.0785,	0,		325.5579},
		// {0,			524.262,243.9417},
		// {0,			0,		1}
	};

	memcpy(_intrinsic,intrinsic,sizeof(_intrinsic));

	linkWidth = LINKWIDTH;
	
	memset(Distance,10000,sizeof(Distance));
	memset(ObstacleVector,0,sizeof(ObstacleVector));
	memset(linkPointsInDepth,0,sizeof(linkPointsInDepth));
	memset(obstaclePointsInDepth,0,sizeof(obstaclePointsInDepth));
	memset(baseShoulderElbowWristFlangeInDepth,0,sizeof(baseShoulderElbowWristFlangeInDepth));
}

/**********************************************************************************************************
// destructor of the depth_processor
***********************************************************************************************************/
depth_processor::~depth_processor(){}

/**********************************************************************************************************
// set the depth image to be processed
***********************************************************************************************************/
bool depth_processor::setDepthMat(Mat* depthMat)
{
	if(_depthMat != NULL){
		_depthMat->copyTo(_lastDepthMat);
	}
	
	_depthMat = depthMat;

	return true;
}

/**********************************************************************************************************
// set the extrinsic of the sensor or camera
***********************************************************************************************************/
bool depth_processor::setCameraExtrinsic(double extrinsic[4][4],double extrinsicInv[4][4])
{
	memcpy(_extrinsic,extrinsic,sizeof(_extrinsic));
	memcpy(_extrinsicInv,extrinsicInv,sizeof(_extrinsicInv));
	return true;
}

/**********************************************************************************************************
// set the threshold of the distance
***********************************************************************************************************/
bool depth_processor::setThreshold(double threshold)
{
	_threshold = threshold;
	return true;
}

/**********************************************************************************************************
// if the Odepth is in S or not
//		S is the region of surveillance, in which, the objects are considerate as obstacles
// Inputs:
//   Odepth, object in depth space
//   S,      region of surveillance in depth space
// Output:
//   ret,    0, the Odpeth is NOT in S
//           1, the Odepth is in S
***********************************************************************************************************/
bool depth_processor::checkRegionOfSurveillance(int Odepth[3],int S[3][2])const
{
	bool ret = true;

	for (int i=0; i<3; i++)
	{
		if ((Odepth[i] < S[i][0])||(Odepth[i] > S[i][1]))
		{
			ret = false;
			return ret;
		}
	}

	return ret;
}

/**********************************************************************************************************
// get the region of surveillance, defined by a cube of side 2*rho centered at this control point
//      S is in sensor depth space, [-u, u] [-v v] [-d d]
// Inputs:
//   Pdepth, control point in depth space
//   rho,    side length of a cube
// Output:
//   S,      cartesian region of surveillance, made by a cube of side 2*rho,
***********************************************************************************************************/
bool depth_processor::getRegionOfSurveillance(int S[3][2],int Pdepth[3],double rho)const
{
	double px = Pdepth[0];
	double py = Pdepth[1];
	double dp = Pdepth[2];

	double fx = _intrinsic[0][0];
	double fy = _intrinsic[1][1];
	double cx = _intrinsic[0][2];
	double cy = _intrinsic[1][2];

	double temp = dp - rho;
	if (temp < 0)
	{
		return false;
	}

	double xs = (rho*fx)/temp;
	double ys = (rho*fy)/temp;

	S[0][0] = std::max<int>(20,px - 0.5*xs);
	S[0][1] = std::min<int>(620,px + 0.5*xs);

	S[1][0] = std::max<int>(20,py - 0.5*ys);
	S[1][1] = std::min<int>(460,py + 0.5*ys);

	S[2][0] = std::max<int>(500,MIN(dp - rho,dp+rho));
	S[2][1] = std::min<int>(10000,MAX(dp + rho,dp - rho));

	return true;
}

/**********************************************************************************************************
//   Evaluate distances between the control point Pdepth and the obstacle
//       points Odepth in the depth space, [pixel_x,pixel_y,depth]
//
//		the two points are in the sensor's depth space
//		while, the distance and vector are in the sensor's Cartesian space	
//
//   Inputs:
//       Odepth, object in depth space
//       Pdepth, control point in depth space
//       global fx fy cx cy, camera intrinsic parameter
// Outputs:
//       D: distance between Odepth and Pdepth in depth space,	(mm)
//       V: norm vector between Odepth and Pdepth in sensor Cartesian space
// bool, true: valid distance was found, false: no valid distance found
***********************************************************************************************************/
bool depth_processor::getDistanceVectorInDepthSpace(double& distance, double Vector[3],int Odepth[3], int Pdepth[3])const
{
	double ox = Odepth[0];
	double oy = Odepth[1];
	double od = Odepth[2];

	double px = Pdepth[0];
	double py = Pdepth[1];
	double pd = Pdepth[2];

	double fx = _intrinsic[0][0];
	double fy = _intrinsic[1][1];
	double cx = _intrinsic[0][2];
	double cy = _intrinsic[1][2];

	double ox_hat = ox;
	double oy_hat = oy;

	if (px < ox)
	{
		ox_hat = ox;
	}
	else if (px > (ox + 1))
	{
		ox_hat = ox + 1;
	}
	else
	{
		ox_hat = px;
	}

	if (py < oy)
	{
		oy_hat = oy;
	}
	else if (py > oy + 1)
	{
		oy_hat = oy + 1;
	}
	else
	{
		oy_hat = py;
	}

	if ( od < pd)
	{
		od = pd;
	}

	Vector[0] = ((ox_hat - cx)*od - (px - cx)*pd)/fx;
	Vector[1] = ((oy_hat - cy)*od - (py - cy)*pd)/fy;
	Vector[2] = od - pd;

	distance = norm3(Vector);

	if (distance < _threshold)
	{
		distance = 0;
		memset(Vector,0,sizeof(double)*3);
		return false;
	}
	else
	{
		Vector[0] = Vector[0]/distance;
		Vector[1] = Vector[1]/distance;
		Vector[2] = Vector[2]/distance;

		distance = distance - _threshold;
		
		return true;
	}
}

/**********************************************************************************************************
//   Consider a Cartesian region of surveillance S, made by a cube of side
//   2*rho centered at control point Pdepth, where the presence of obstacles
//   must be detected. Then the minimum distance can be calculated in S.
//
//	 Calculate the minimum distance between a control point and object, (mm)
//
//	 for noise reduction, more than tempNUM/2 closed objects were considerate.
//
//   Inputs:
//       Pdepth, control point in depth
//       rho,    side length of the cube
//       _depthMat, depth image from KINECT or any other rgb-d sensor
//   Outputs:
//       D,      minimum distance of the objects in depth image to Pdepth, (mm)
//       V,      the norm vector of the minimum distance object to Pdepth in sensor Cartesian space
//		 Odepth, the closed obstacle to Pdepth 
// bool, true: valid distance was found, false: no valid distance found
***********************************************************************************************************/
bool depth_processor::getMinimumDistanceVector(double& Distance, double Vector[3],int miniOdepth[3],int Pdepth[3],double rho)const
{
	Distance = 10000;
	int Odepth[3] = {0};
	double tempD = 10000;

	#define tempNUM	(500)

	int tempIndex = 0;
	int tempMiniOdepth[tempNUM][3] = {{0.0}};
	
	double tempDistance[tempNUM] = {0.0};
	double tempVector[tempNUM][3] = {{0.0}};

	for (int i=0; i<tempNUM; i++)
	{
		tempDistance[i] = Distance;
	}

	for (int k=0; k<3; k++)
	{
		miniOdepth[k] = Pdepth[k];
		Vector[k] = 0.0;
	}

	if (Pdepth[0] <= 1 || Pdepth[1] <= 1 || Pdepth[0] >= _depthMat->cols || Pdepth[1] >= _depthMat->rows)
	{
		return false;
	}

	int S[3][2] = {{0.0}};

	if(!getRegionOfSurveillance(S,Pdepth,rho))
	{
		return false;
	}

	int j = S[1][0];

	while(j < S[1][1]-1 && j>=0)
	{
		j ++;
		int i = S[0][0];
		// const ushort* pDepthRow = _depthMat->ptr<ushort>(j);
		const float* pDepthRow = _depthMat->ptr<float>(j);

		while(i < S[0][1]-1)
		{
			i ++;

			if (i <=1 || i>= _depthMat->cols
				|| j <=1 || j>= _depthMat->rows)
			{
				continue;
			}

			Odepth[0] = i; Odepth[1] = j; Odepth[2] = (int)(pDepthRow[i]*1000);

			if (Odepth[2] > 10000 || Odepth[2] <= 0)
			{
				continue;
			}

			if (!checkRegionOfSurveillance(Odepth,S))
			{
				continue;
			}
			else
			{
				if (!getDistanceVectorInDepthSpace(tempD,Vector,Odepth,Pdepth) || tempD <= 0)
				{
					continue;
				}

				// only one closed point were considerate---------------------------------------
				//if (tempD < rho)
				//{
				//	rho = tempD;
				//	getRegionOfSurveillance(S,Pdepth,rho);
				//}

				//if (tempD < Distance)
				//{
				//	Distance = tempD;
				//	memcpy(miniOdepth,Odepth,sizeof(Odepth));
				//}
				// only one closed point were considerate---------------------------------------

				// more the tempNUM/2 closed points were considerate----------------------------
				for (int k = 0; k<tempNUM; k++)
				{
					if (tempD < tempDistance[k] )
					{
						if (tempDistance[k] == Distance)
						{
							tempIndex++;
							tempDistance[k] = tempD;
							memcpy(tempVector[k],Vector,sizeof(double)*3);
							memcpy(tempMiniOdepth[k],Odepth,sizeof(Odepth));
						}
						else
						{
							if (tempIndex > tempNUM - 1)
							{
								tempDistance[k] = tempD;
								memcpy(tempVector[k],Vector,sizeof(double)*3);
								memcpy(tempMiniOdepth[k],Odepth,sizeof(Odepth));
							}
							else
							{
								tempDistance[tempIndex] = tempD;
								memcpy(tempVector[tempIndex],Vector,sizeof(double)*3);
								memcpy(tempMiniOdepth[tempIndex],Odepth,sizeof(Odepth));
								tempIndex++;
							}
						}
						
						break;
					}
				}
				// more the tempNUM/2 closed points were considerate----------------------------
			}
		}
	}

	double sumDistance = 0;
	double sumVector[3] = {0.0};

	int	   sumIndex = 0;
	int	   sumOdepth[3] = {0};

	for (int i=0; i<tempNUM; i++)
	{
		if (tempDistance[i] < Distance)
		{
			sumDistance += tempDistance[i];

			sumVector[0] += tempVector[i][0];
			sumVector[1] += tempVector[i][1];
			sumVector[2] += tempVector[i][2];

			sumOdepth[0] += tempMiniOdepth[i][0];
			sumOdepth[1] += tempMiniOdepth[i][1];
			sumOdepth[2] += tempMiniOdepth[i][2];
			sumIndex++;
		}
	}

	double tempDis = norm3(sumVector);

	if(tempDis == 0) {return false;}

	if (sumIndex > tempNUM*0.8)
	{
		Distance = sumDistance/sumIndex;

		Vector[0] = sumVector[0]/tempDis;
		Vector[1] = sumVector[1]/tempDis;
		Vector[2] = sumVector[2]/tempDis;

		miniOdepth[0] = sumOdepth[0]/sumIndex;
		miniOdepth[1] = sumOdepth[1]/sumIndex;
		miniOdepth[2] = sumOdepth[2]/sumIndex;
	}

	if (Distance == 0)
	{
		return false;
	}

	return true;
}

/**********************************************************************************************************
// In some cases, we would like to have a single distance information about
// all objects surrounding the control point. A possible aggregation method
// is to compute the mean distance.

// Inputs:
//   Pdpeth, control point in the depth space 
//   rho,    side length of the cube defines the surveillance area
//   _depthMat, depth image from KINECT or any other rgb-d sensor
// Outputs:
//   D,      mean distance of the objects in depth image to Pdepth, (mm)
//   V,      the norm vector of the minimum distance object to Pdepth in sensor Cartesian space
//	 Odepth, the mean closed obstacle to Pdepth 
***********************************************************************************************************/
bool depth_processor::getMeanDistanceVector(double& Distance, double Vector[3],int meanOdepth[3], int Pdepth[3], double rho)const
{
	Distance = 10000;
	int Odepth[3] = {0};
	int sumOdepth[3] = {0};

	int N = 0;
	double SumD = 0.0;
	double SumV[3] = {0.0};
	double tempV[3] = {0.0};
	double tempD = Distance;

	for (int k=0; k<3; k++)
	{
		meanOdepth[k] = Pdepth[k];
		Vector[k] = 0.0;
	}

	if (Pdepth[0] <= 1 || Pdepth[1] <= 1)
	{
		return false;
	}

	int S[3][2] = {{0.0}};

	if(!getRegionOfSurveillance(S,Pdepth,rho))
	{
		return false;
	}


	int j = S[1][0];

	while(j < S[1][1]-1)
	{
		j ++;
		int i = S[0][0];
		// const ushort* pDepthRow = _depthMat->ptr<ushort>(j);
		float* pDepthRow = _depthMat->ptr<float>(j);

		while(i < S[0][1]-1)
		{
			i ++;

			if (i <=1 || i>= _depthMat->cols
				|| j <=1 || j>= _depthMat->rows)
			{
				continue;
			}

			Odepth[0] = i; Odepth[1] = j; Odepth[2] = (int)(pDepthRow[i]*1000);

			if (Odepth[2] == 65535 || Odepth[2] == 0)
			{
				continue;
			}

			// Odepth[2] = Odepth[2]>>3;

			if (!checkRegionOfSurveillance(Odepth,S))
			{
				continue;
			}
			else
			{
				if (!getDistanceVectorInDepthSpace(tempD,tempV,Odepth,Pdepth))
				{
					continue;
				}
				
				SumD = SumD + tempD;
				for (int k=0; k<3; k++)
				{
					SumV[k] = SumV[k] + tempV[k];
					sumOdepth[k] = sumOdepth[k] + Odepth[k];
				}
				N = N + 1;
			}
		}
	}

	if (N <= tempNUM)
	{
		return false;
	}

	Distance = SumD / N;

	double temp = norm3(SumV);

	if (temp == 0)
	{
		return false;
	}

	for (int k=0; k<3; k++)
	{
		Vector[k] = SumV[k] / temp;
	}

	for (int k=0; k<3; k++)
	{
		meanOdepth[k] = sumOdepth[k] / N;
	}

	return true;
}

/**********************************************************************************************************
// a hybrid distance and vector, this allows to collision avoidance reacts
// according to the nearest object for the intensity, while taking into 
// consideration all the objects in the surveillance area for the reaction
// direction

// Inputs:
//   Pdpeth, control point in the depth space 
//   rho,    side length of the cube defines the surveillance area
//   _depthMat, depth image from KINECT or any other rgb-d sensor
// Outputs:
//   D,      closed distance of the objects in depth image to Pdepth, (mm)
//   V,      the norm vector of the minimum distance object to Pdepth in sensor Cartesian space
//	 Odepth, the mean closed obstacle to Pdepth 
***********************************************************************************************************/
bool depth_processor::getHybridDistanceVector(double& Distance, double Vector[3],int hybridOdepth[3], int Pdepth[3],double rho)const
{
	Distance = 0;
	memcpy(hybridOdepth,Pdepth,sizeof(int)*3);

	double tempV[3] = {0.0};
	int tempOdepth[3] = {0};

	bool flag = getMinimumDistanceVector(Distance,tempV,hybridOdepth ,Pdepth,rho);

	double tempD = 0.0;
	flag = flag && getMeanDistanceVector(tempD,Vector,tempOdepth,Pdepth,rho);

	if (Distance == 0 || !flag || tempD == 0)
	{
		return false;
	}

	return true;
}

/**********************************************************************************************************
// coordinate system transformation
***********************************************************************************************************/
bool depth_processor::projectToDepthFromCamera(int Pdepth[3],double pointInCamera[3])const
{
	double fx = _intrinsic[0][0];
	double fy = _intrinsic[1][1];
	double cx = _intrinsic[0][2];
	double cy = _intrinsic[1][2];

	Pdepth[0] = ceil(pointInCamera[0]*fx/pointInCamera[2] + cx);
	Pdepth[1] = ceil(pointInCamera[1]*fy/pointInCamera[2] + cy);
	Pdepth[2] = round(pointInCamera[2]) ;

	return true;
}

/**********************************************************************************************************
// coordinate system transformation
***********************************************************************************************************/
bool depth_processor::projectToCameraFromDepth(double pointInCamera[3],int Pdepth[3])const
{
	double fx = _intrinsic[0][0];
	double fy = _intrinsic[1][1];
	double cx = _intrinsic[0][2];
	double cy = _intrinsic[1][2];

	pointInCamera[2] = Pdepth[2];
	pointInCamera[1] = (Pdepth[1] - cy)*pointInCamera[2]/fy;
	pointInCamera[0] = (Pdepth[0] - cx)*pointInCamera[2]/fx;
	return true;
}

/**********************************************************************************************************
// coordinate system transformation
***********************************************************************************************************/
bool depth_processor::projectToCameraFromRobotbase(double pointInCamera[3],double pointInWord[3])const
{
	for (int i=0; i<3; i++)
	{
		pointInCamera[i] = _extrinsicInv[i][0]*pointInWord[0] 
						 + _extrinsicInv[i][1]*pointInWord[1]
						 + _extrinsicInv[i][2]*pointInWord[2]
						 + _extrinsicInv[i][3]*1.0;
	}


	return true;
}

/**********************************************************************************************************
// coordinate system transformation
***********************************************************************************************************/
bool depth_processor::projectToRobotbaseFromCamera(double pointInWord[3],double pointInCamera[3])const
{
	for (int i=0; i<3; i++)
	{
		pointInWord[i] = _extrinsic[i][0]*pointInCamera[0]
					   + _extrinsic[i][1]*pointInCamera[1]
					   + _extrinsic[i][2]*pointInCamera[2]
					   + _extrinsic[i][3]*1.0;
	}

	return true;
}

/**********************************************************************************************************
// coordinate system transformation
***********************************************************************************************************/
bool depth_processor::projectToDepthFromRobotbase(int Pdepth[3],double pointInWord[3])const
{
	double temp[3] = {0.0};

	projectToCameraFromRobotbase(temp,pointInWord);

	projectToDepthFromCamera(Pdepth,temp);

	return true;
}

/**********************************************************************************************************
// coordinate system transformation
***********************************************************************************************************/
bool depth_processor::projectToRobotbaseFromDepth(double pointInWord[3],int Pdepth[3])const
{
	double temp[3] = {0.0};

	projectToCameraFromDepth(temp,Pdepth);

	projectToRobotbaseFromCamera(pointInWord,temp);

	return true;
}

/**********************************************************************************************************
// convert the depht image into rgb image
***********************************************************************************************************/
void depth_processor::getDepthDataAsArgbMat(cv::Mat* pImage, cv::Mat* pDepthOrigionData){

  int depthWidth, depthHeight;
  depthWidth = pDepthOrigionData->cols;
  depthHeight = pDepthOrigionData->rows;

  for (int y=0; y<depthHeight; y++){
    // const ushort* pDepthRow = pDepthOrigionData->ptr<ushort>(y);
    const float* pDepthRow = pDepthOrigionData->ptr<float>(y);
    cv::Vec3b* pDepthRgbRow = pImage->ptr<Vec3b>(y);
    for (int x=0; x<depthWidth; x++){
        // ushort raw_depth = pDepthRow[x];
        int raw_depth = pDepthRow[x]*1000;
        if (raw_depth <= 8000 && raw_depth >= 300){
          unsigned  char b = (256-(raw_depth)/32);
          pDepthRgbRow[x] = Vec3b(b,b,b);
        }else{
          if (pDepthRow[x] < 0){
              pDepthRgbRow[x] = Vec3b(0,255,0);  
          }else{
              pDepthRgbRow[x] = Vec3b(0,0,0);
          }
        }
    }
  }
}


/**********************************************************************************************************
// get the coordinates of link points on the robot the depth space
// project the link points on the robot to the depth image
***********************************************************************************************************/
bool   depth_processor::getLinkPointsInDepth(double linkPointsInRobotbase[][4]){
	for (int i=0; i< POINTSNUM;i++){
		projectToDepthFromRobotbase(linkPointsInDepth[i],linkPointsInRobotbase[i]);
	}

	return true;
}


/**********************************************************************************************************
// Assuming that the robot links are cylinders with round heads, the centers are A and B, the radius is
//   LinkWidth, so to filter out the robot links, all points covered by the cylinders should be removed
// Inputs:
//   the key robot link points, base/shoulder/elbow/wrist/flange,
//	  linkWidth, the radius of the cylinders
// Output:
//   true,	filter done
***********************************************************************************************************/
bool depth_processor::removeRobotLinksFromDepth(Mat* depthMat,double linkPointsInRobotbase[][4]){
	
	getLinkPointsInDepth(linkPointsInRobotbase);

	memcpy(baseShoulderElbowWristFlangeInDepth[BASE],linkPointsInDepth[BASE],sizeof(int)*3);
	memcpy(baseShoulderElbowWristFlangeInDepth[SHOULDER],linkPointsInDepth[SHOULDER],sizeof(int)*3);
	memcpy(baseShoulderElbowWristFlangeInDepth[ELBOW],linkPointsInDepth[ELBOW],sizeof(int)*3);
	memcpy(baseShoulderElbowWristFlangeInDepth[WRIST],linkPointsInDepth[WRIST],sizeof(int)*3);
	memcpy(baseShoulderElbowWristFlangeInDepth[FLANGE],linkPointsInDepth[FLANGE],sizeof(int)*3);

	ushort depthWidth = depthMat->cols, depthHeight = depthMat->rows;

	double fx = _intrinsic[0][0];
	double fy = _intrinsic[1][1];
	double cx = _intrinsic[0][2];
	double cy = _intrinsic[1][2];

	// 方法一：包络球
	for(int k=0; k<POINTSNUM;k++)
	{
		int xu = linkPointsInDepth[k][0];
		int yv = linkPointsInDepth[k][1];
		int zd = linkPointsInDepth[k][2];
		double zr = linkPointsInRobotbase[k][3];
		if(zd == 0){continue;}
		int r = 1.2*0.5*zr/zd*(0.5*fx+0.5*fy) ; // 简化处理半径

		int minX = xu - r, maxX = xu + r, minY = yv - r, maxY = yv + r;

		minX = MAX(minX,0); maxX = MIN(maxX,depthWidth-1);
		minY = MAX(minY,0); maxY = MIN(maxY,depthHeight-1);

		float* pDepthRow;
		for (int y = minY; y < maxY; y++)
		{
			double yr = sqrt(zr*zr - (y-yv)*(y-yv));
			pDepthRow = _depthMat->ptr<float>(y);
			for(int x = minX; x < maxX; x++)
			{
				int temp = (pDepthRow[x]*1000);
				double xr = sqrt(yr*yr - (x-xu)*(x-xu));
				if(temp > zd - xr && temp < zd + xr)
				{
					pDepthRow[x] = -100;
				}
			}
		}
	}

	return true;

	// 方法二，球头圆柱

	int offset = 1;
	// 4 links, base-shoulder  shoulder-elbow  elbow-wrist wrist-flange
	for (int k=4; k>0; k--) 
	{
		// endpoints of the cylinder(robot links)
		int A[3] = {0}, B[3] = {0};

		// get the Minimum Enclosing Rectangle to reduce filtering time
		// the MER in depth image is determined by minX maxX minY and maxY
		int minX,maxX,minY,maxY;

		int sRowX = baseShoulderElbowWristFlangeInDepth[k][0];
		int eRowX = baseShoulderElbowWristFlangeInDepth[k-1][0];

		int sColY = baseShoulderElbowWristFlangeInDepth[k][1];
		int eColY = baseShoulderElbowWristFlangeInDepth[k-1][1];

		int sD = baseShoulderElbowWristFlangeInDepth[k][2];
		int eD = baseShoulderElbowWristFlangeInDepth[k-1][2];

		A[2] = sD;
		B[2] = eD;

		if (sRowX < eRowX)
		{
			minX = MAX(sRowX - fx/sD*linkWidth,offset);
			maxX = MIN(eRowX + fx/eD*linkWidth,depthWidth-offset);
			A[0] = MAX(sRowX,offset);
			B[0] = MIN(eRowX,depthWidth-offset);
		}
		else
		{
			minX = MAX(eRowX - fx/eD*linkWidth,offset);
			maxX = MIN(sRowX + fx/sD*linkWidth,depthWidth-offset);
			A[0] = MIN(sRowX,depthWidth-offset);
			B[0] = MAX(eRowX,offset);
		}

		if (sColY < eColY)
		{
			minY = MAX(sColY - fx/sD*linkWidth,offset);
			maxY = MIN(eColY+fx/eD*linkWidth,depthHeight-offset);
			A[1] = MAX(sColY,offset);
			B[1] = MIN(eColY,depthHeight-offset);
		}
		else
		{
			minY = MAX(eColY - fx/eD*linkWidth,offset);
			maxY = MIN(sColY + fx/sD*linkWidth,depthHeight-offset);
			A[1] = MIN(sColY,depthHeight-offset);
			B[1] = MAX(eColY,offset);
		}

		// USHORT* pDepthRow0;
		float* pDepthRow0;
		for (int y = minY; y < maxY; y++)
		{
			int dy = 0,disY = 0;
			if((B[1] - A[1]) !=0)
				disY = (eD-sD)*(y-A[1])/(B[1] - A[1]) + sD;

			if(disY!=0)
				dy = fy/disY*linkWidth;

			// pDepthRow0 = depthMat->ptr<USHORT>(y);
			pDepthRow0 = depthMat->ptr<float>(y);
			for (int x = minX; x < maxX; x++)
			{

				int dx = 0,disX = 0;

				if((B[0] - A[0]) !=0)
					disX = (eD-sD)*(x-A[0])/(B[0] - A[0]) + sD;

				if(disX!=0)
					dx = fy/disX*linkWidth;

				// int P[3] = {x,y,pDepthRow0[x]>>3};
				int P[3] = {x,y,(pDepthRow0[x]*1000)};

				double dis = PointToSegDist(P,A,B);
				if (dis>dx && dis > dy)
					continue;

				// int temp = (pDepthRow0[x]>>3);
				int temp = (pDepthRow0[x]*1000);
				if ((temp > disY + linkWidth || temp < disY - linkWidth)
					&&(temp > disX + linkWidth || temp < disX - linkWidth)
					&&(0!=disY)&&(0!=disX))
					continue;

				if ((disY==0)&&(temp > disX + linkWidth || temp < disX - linkWidth))
					continue;

				if ((0==disX)&&(temp > disY + linkWidth || temp < disY - linkWidth))
					continue;

				pDepthRow0[x] = -100;
			}
		}
	}

	return true;
}


/**********************************************************************************************************
// distance from a point to a segment of line
***********************************************************************************************************/
double depth_processor::PointToSegDist(int point[2],int A[2],int B[2])const
{
	double error = 1e10;

	double cross = (B[0] - A[0] + 0.0) * (point[0] - A[0] + 0.0) + (B[1] - A[1] + 0.0) * (point[1] - A[1] + 0.0);

	if (cross <= 0)
		return sqrt((point[0] - A[0] + 0.0 ) * (point[0] - A[0] + 0.0 ) + (point[1] - A[1] + 0.0 ) * (point[1] - A[1] +0.0));

	double d2 = (B[0] - A[0] + 0.0 ) * (B[0] - A[0] +0.0 ) + (B[1] - A[1] + 0.0) * (B[1] - A[1] + 0.0);

	if (cross >= d2)
		return sqrt((point[0] - B[0] + 0.0) * (point[0] - B[0]+0.0) + (point[1] - B[1] +0.0) * (point[1] - B[1] +0.0));

	if (ABS(d2) < ACCURACY_FACTOR)
	{
		return error;
	}

	double r = cross / d2;
	double px = A[0] + (B[0] - A[0]) * r;
	double py = A[1] + (B[1] - A[1]) * r;

	double temp = (point[0] - px + 0.0) * (point[0] - px + 0.0) + (py - point[1] +0.0) * (py - point[1] +0.0);

	if (ABS(temp) < ACCURACY_FACTOR)
	{
		return 0;
	}

	return sqrt(ABS(temp));
}