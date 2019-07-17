

#pragma once
#include "matrix.h"
#include "kinematics.h"
#include "depth_processor.h"


namespace ROBOT_FILTER{

    enum SHAPE_TYPE{SPHERE,CYLINDER,PRISM,CONE,PYRAMID,SPHERE_CYLINDER};	//球体，平头圆柱/圆台，平头棱柱/棱台，圆锥，棱锥,圆头圆柱

    class RobotForwardKinematics{

        public:
            double T01[4][4];
            double T02[4][4];
            double T03[4][4];
            double T04[4][4];
            double T05[4][4];
            double T06[4][4];
            double T07[4][4];
            double T0t[4][4];

            double T12[4][4],T23[4][4],T34[4][4],T45[4][4],T56[4][4],T67[4][4],T7t[4][4];

            double l[4];

            double jointAngle[DOF];
        public: 
            RobotForwardKinematics(){
                l[0] = LBS;
                l[1] = LSE;
                l[2] = LEW;
                l[3] = LWT;
            }

            ~RobotForwardKinematics(){}

            bool updateTransforms(double r[DOF]){

                bool flag = true;
                for(int i=0; i<DOF; i++){
                    if(ABS(jointAngle[i] - r[i]) > 1e-5){
                        flag = false;
                        continue;
                    }
                }

                if(flag == true){
                    return true;; 
                }else{
                    for(int i=0; i<DOF;i++){
                        jointAngle[i] = r[i];
                    }
                }

                double L01 = l[0];
                double L34 = l[1];
                double L45 = l[2];
                double L7t = l[3];
    

                double l_tmp[3];
                double R1_tmp[4][4],R2_tmp[4][4],R3_tmp[4][4],R4_tmp[4][4];

                // step 1: �����������α任����&ĩ��λ��
                // ����T01:	T01 = trans(0,0,l_01)*rot_z(r(1));
                l_tmp[0]=0, l_tmp[1]=0, l_tmp[2]=L01;
                trans(R1_tmp,l_tmp);
                rot_z(R2_tmp,r[0]);
                M4p4(T01,R1_tmp,R2_tmp);
                // ����T12: T12 = rot_x(-pi/2)*rot_z(-pi/2)*rot_z(r(2));
                rot_x(R1_tmp,-PI/2);
                rot_z(R2_tmp,-PI/2);
                rot_z(R3_tmp,r[1]);
                M4p4p4(T12,R1_tmp,R2_tmp,R3_tmp);
                // ����T23: T23 = rot_y(pi/2)*rot_z(pi/2)*rot_z(r(3));
                rot_y(R1_tmp,PI/2);
                rot_z(R2_tmp,PI/2);
                rot_z(R3_tmp,r[2]);
                M4p4p4(T23,R1_tmp,R2_tmp,R3_tmp);
                // ����T34:	T34 = trans(0,0,l_34)*rot_x(-pi/2)*rot_z(-pi/2)*rot_z(r(4));
                l_tmp[0]=0, l_tmp[1]=0, l_tmp[2]=L34;
                trans(R1_tmp,l_tmp);
                rot_x(R2_tmp,-PI/2);
                rot_z(R3_tmp,-PI/2);
                M4p4p4(R4_tmp,R1_tmp,R2_tmp,R3_tmp);
                rot_z(R1_tmp,-r[3]);// iiwa�ĵ������������Ƿ���
                M4p4(T34,R4_tmp,R1_tmp);
                // ����T45:	T45 = trans(l_45,0,0)*rot_y(pi/2)*rot_z(pi/2)*rot_z(r(5));
                l_tmp[0]=L45, l_tmp[1]=0, l_tmp[2]=0;
                trans(R1_tmp,l_tmp);
                rot_y(R2_tmp,PI/2);
                rot_z(R3_tmp,PI/2);
                M4p4p4(R4_tmp,R1_tmp,R2_tmp,R3_tmp);
                rot_z(R1_tmp,r[4]);
                M4p4(T45,R4_tmp,R1_tmp);
                // ����T56:	T56 = rot_x(-pi/2)*rot_z(-pi/2)*rot_z(r(6));
                rot_x(R1_tmp,-PI/2);
                rot_z(R2_tmp,-PI/2);
                rot_z(R3_tmp,r[5]);
                M4p4p4(T56,R1_tmp,R2_tmp,R3_tmp);
                // ����T67:	T67 = rot_y(pi/2)*rot_z(pi/2)*rot_z(r(7));
                rot_y(R1_tmp,PI/2);
                rot_z(R2_tmp,PI/2);
                rot_z(R3_tmp,r[6]);
                M4p4p4(T67,R1_tmp,R2_tmp,R3_tmp);
                // ����T7t:	T7t = trans(0,0,l_7t);
                l_tmp[0]=0, l_tmp[1]=0, l_tmp[2]=L7t;
                trans(T7t,l_tmp);

                // ����T03��T04��T05��T0t
                M4p4(T02,T01,T12);
                M4p4(T03,T02,T23);
                M4p4(T04,T03,T34);
                M4p4(T05,T04,T45);
                M4p4(T06,T05,T56);
                M4p4(T07,T06,T67);
                M4p4(T0t,T07,T7t);

                return true;
            }
    };

    RobotForwardKinematics rfk;

    struct Point{
        double x;
        double y;
        double z;
        double r;
        Point(double X,double Y,double Z,double R){
            x = X; y = Y; z = Z; r = R;
        }
        Point(){x=0;y=0;z=0;r=0;}
    };

    struct POSITION{
        double x,y,z;
        POSITION(double X,double Y,double Z){x=X;y=Y;z=Z;}
        POSITION(){x=0;y=0;z=0;}
    };

    struct ROTATION{
        double x,y,z,w;
        ROTATION(double X,double Y,double Z,double W){x = X; y=Y;z=Z;w=W;}
        ROTATION(){x=0;y=0;z=0;w=1;}
        ROTATION(double T[4][4]){
            double Qxx = T[0][0];
            double Qxy = T[0][1];
            double Qxz = T[0][2];

            double Qyx = T[1][0];
            double Qyy = T[1][1];
            double Qyz = T[1][2];

            double Qzx = T[2][0];
            double Qzy = T[2][1];
            double Qzz = T[2][2];

            double r = 0.0;
            double s = 0.0;

            double t = Qxx + Qyy + Qzz;
            double maxv = MAX(Qxx, MAX(Qyy, Qzz));

            if (t >= 0)
            {
                r = sqrt(1 + t);
                s = 0.5 / r;
                w = 0.5*r;
                x = (Qzy - Qyz)*s;
                y = (Qxz - Qzx)*s;
                z = (Qyx - Qxy)*s;
            }
            else
            {
                if (maxv == Qxx)
                {
                    r = sqrt(1 + Qxx - Qyy - Qzz);
                    s = 0.5 / r;
                    w = (Qzy - Qyz)*s;
                    x = 0.5*r;
                    y = (Qyx + Qxy)*s;
                    z = (Qxz + Qzx)*s;
                }
                else
                {
                    if (maxv == Qyy)
                    {
                        r = sqrt(1 + Qyy - Qxx - Qzz);
                        s = 0.5 / r;
                        w = (Qxz - Qzx)*s;
                        x = (Qyx + Qxy)*s;
                        y = 0.5*r;
                        z = (Qzy + Qyz)*s;
                    }
                    else
                    {
                        r = sqrt(1 + Qzz - Qxx - Qyy);
                        s = 0.5 / r;
                        w = (Qyx - Qxy)*s;
                        x = (Qxz + Qzx)*s;
                        y = (Qzy + Qyz)*s;
                        z = 0.5*r;
                    }
                }
            }




        }   // 齐次矩阵转四元素
    };

    class Shape{
        public:
            Shape(SHAPE_TYPE type,vector<Point> upPoints,vector<Point> lowPoints,
            bool (*transform)(double T[][4],double jointAngle[])){
                _type = type;
                _upPoints.assign(upPoints.begin(),upPoints.end()); // 深拷贝1
                _lowPoints.swap(lowPoints); // 深拷贝2
                _transform = transform;
                _Tinv.IdentityInitialize();
                first_flag = true;
            }
            ~Shape(){}

            bool isPointInThisShape(double point[3],double jointAngle[DOF]){

                updateTransfrom(jointAngle);

                // point 为机器人基座坐标系下的点
                Matrix<double> tempP(4,1);
                tempP.SetElement(3,0,1);
                for(int i=0; i<3; i++){
                    tempP.SetElement(i,0,point[i]);
                }
                tempP = _Tinv*tempP;

                Point p0;
                p0.x = tempP.GetElement(0,0);
                p0.y = tempP.GetElement(1,0);
                p0.z = tempP.GetElement(2,0);

                // 判断局部点p0 是不是在几何体内
                switch(_type){
                    case SPHERE:    //球体
                        return isInSphere(p0);
                    break;
                    case CYLINDER:
                        return isInCylinder(p0);
                    break;
                    case PRISM:
                        
                    break;
                    case CONE:
                    break;
                    case PYRAMID:
                    break;
                    case SPHERE_CYLINDER:
                        return isInCylinder(p0);
                    break;
                }

                return false;
            }
            
            bool updateTransfrom(double jointAngle[]){

                bool flag = true;
                for(int i=0; i<DOF; i++){
                    if(ABS(jointAngle[i] - _jointAngle[i]) > 1e-5){
                        flag = false;
                        continue;
                    }
                }

                if(flag == true && !first_flag){
                    return true;; 
                }else{
                    for(int i=0; i<DOF;i++){
                        _jointAngle[i] = jointAngle[i];
                    }
                }

                // 首先，函数_transfrom()得到的使该几何体局部坐标系在机器人基座坐标系中的位姿
                (*_transform)(_T,jointAngle);

                position.x = _T[0][3] ;
                position.y = _T[1][3] ;
                position.z = _T[2][3] ;

                double tempYangle = atan((_upPoints[0].y - _lowPoints[0].y)/(_upPoints[0].z-_lowPoints[0].z));

                double tempXR[4][4] = {0.0};

                rot_x( tempXR, -tempYangle);

                double tempXT[4][4] = {0.0};

                M4p4(tempXT,_T,tempXR);

                ROTATION tempR(tempXT);

                rotation = tempR;

                // 其次，需要将point转换到该几何体的局部坐标系内
                Matrix<double> tempT(4,4);
                
                for(int i=0; i<4; i++){
                    for(int j=0; j<4; j++){
                        tempT.SetElement(i,j,_T[i][j]);
                    }
                }
                
                _Tinv = tempT.inv();

                first_flag = false;

                return true;//_Tinv.isValid();
            }

            POSITION position;
            ROTATION rotation;

        private:

            double _jointAngle[DOF];

            // 几何体类型，球体，圆柱/圆台，棱柱/棱台，圆锥，棱锥
            SHAPE_TYPE _type;

            // 几何体的关键点，定义在几何体局部坐标系下,如下圆柱，P1为upPoints，P2为lowPoint
            // 几何体局部坐标系可以定义在任意处，其与世界坐标系之间的位姿关系有机器人关节角通过函数_transform得到
            //      ---P1---
            //      |      |
            //      |      |
            //      |      |
            //      ---P2---

        public:
            vector<Point> _upPoints;
            vector<Point> _lowPoints;
        private:
            // 函数指针，得到该几何体局部坐标系在世界坐标系下的位姿
            bool (*_transform)(double T[][4],double jointAngle[]);

            // 临时数据，局部坐标系在世界坐标系下的位姿
            double _T[4][4];
            Matrix<double> _Tinv;

            bool first_flag;


            ////////////////////////////////////////////////////////////////////////////////
            bool isInSphere(Point p0){
                double d = distance(p0,_upPoints[0]);
                if(d > _upPoints[0].r*1.2){
                    return false;
                }
                return true;
            }

            ////////////////////////////////////////////////////////////////////////////////
            bool isInCylinder(Point p0){

                if( p0.z > _upPoints[0].z*1.2 || p0.z < _lowPoints[0].z*1.2){
                    return false;
                }

                Point temp;
                temp.z = p0.z;
                temp.x = (p0.z - _lowPoints[0].z)*(_upPoints[0].x-_lowPoints[0].x)/(_upPoints[0].z-_lowPoints[0].z) + _lowPoints[0].x;
                temp.y = (p0.z - _lowPoints[0].z)*(_upPoints[0].y-_lowPoints[0].y)/(_upPoints[0].z-_lowPoints[0].z) + _lowPoints[0].y;
                double d = distance(temp,p0);

                if(_upPoints[0].r != _lowPoints[0].r){
                    double r = (_lowPoints[0].r-_upPoints[0].r)/(_upPoints[0].z-_lowPoints[0].z)*(_upPoints[0].z - p0.z) + _upPoints[0].r;
                    if(d > r*1.3){
                        return false;
                    }
                }else{
                    if(d > _upPoints[0].r*1.3){
                        return false;
                    }
                }

                return true;
            }

            ////////////////////////////////////////////////////////////////////////////////
            bool isInSphereCylinder(Point p0){

                bool flag1 = isInCylinder(p0);
                bool flag2 = isInSphere(p0);
                
                Point tempP = _upPoints[0];
                _upPoints[0] = _lowPoints[0];
                bool flag3 = isInSphere(p0);
                _upPoints[0] = tempP;

                return (flag1||flag2||flag3);
            }

            ////////////////////////////////////////////////////////////////////////////////
            double distance(Point p0,Point p1){
                return sqrt( (p0.x-p1.x)*(p0.x-p1.x)+
                             (p0.y-p1.y)*(p0.y-p1.y)+
                             (p0.z-p1.z)*(p0.z-p1.z));
            }

            ////////////////////////////////////////////////////////////////////////////////
            double point2line(Point p,Point p1,Point p2){ // 点到直线的距离，直线为p1和p2
                Point zero; zero.x = 0; zero.y = 0; zero.z = 0;
                Point v12; v12.x = p2.x-p1.x; v12.y = p2.y-p1.y; v12.z = p2.z-p1.z;
                Point v1p; v1p.x = p.x-p1.x;  v1p.y = p.y-p1.y;  v1p.z = p.z-p1.z;
                double dp = distance(v1p,zero);
                double cos = ABS(distance(v12,v1p)) / (distance(v12,zero)*dp);
                double d0 = dp*cos;
                double d = sqrt(dp*dp - d0*d0);
                return d;
            }
    };

    ////////////////////////////////////////////////////////////////////////////////
    // cell0, 圆台与基座坐标系固连
    bool transform0(double T[][4],double jointAngle[DOF]){
        for(int i=0; i<4; i++){
            for (int j=0; j<4;j++){
                T[i][j] = 0;
                if(i==j){T[i][j] = 1;}
            }
        }

        T[2][3] = 85.0;

        return true;
    };

    ////////////////////////////////////////////////////////////////////////////////
    // cell1: 右上斜圆柱
    bool transform1(double T[][4],double jointAngle[DOF]){

        double dis[3] = {0,0,215};
        double R1_tmp[4][4] = {0.0};
        double R2_tmp[4][4] = {0.0};
        trans(R1_tmp,dis);
        rot_z(R2_tmp,jointAngle[0]);
        double R_modify[4][4] = {0.0};
        dis[0] = 0.0; dis[1] = -15.0; dis[2] = 0.0;
        trans(R_modify,dis);

        M4p4p4(T,R1_tmp,R2_tmp,R_modify);

        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // cell2: 圆头圆柱
    bool transform2(double T[][4],double jointAngle[DOF]){
        rfk.updateTransforms(jointAngle);
        for(int i=0; i<4; i++){
            for(int j=0; j<4; j++){
                T[i][j] = rfk.T02[i][j];
            }
        }

        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // cell3: 右下斜圆柱
    bool transform3(double T[][4], double jointAngle[DOF]){

        rfk.updateTransforms(jointAngle);

        double R1_tmp[4][4] = {0.0};
        double R2_tmp[4][4] = {0.0};
        double R3_tmp[4][4] = {0.0};

        rot_y(R1_tmp,PI/2);
        rot_z(R2_tmp,PI/2);

        M4p4(R3_tmp,R1_tmp,R2_tmp);

        double T3_tmp[4][4] = {0.0};

        M4p4(T3_tmp,rfk.T02,R3_tmp);

        double dis[3] = {0,0,110};
        
        trans(R1_tmp,dis);

        // M4p4(T,rfk.T03,R1_tmp);

        double R_modify[4][4] = {0.0};
        dis[0] = 0.0; dis[1] = 25.0; dis[2] = 0.0;
        trans(R_modify,dis);

        M4p4p4(T,T3_tmp,R1_tmp,R_modify);

        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // cell4: 平头圆柱
    bool transform4(double T[][4], double jointAngle[DOF]){
        rfk.updateTransforms(jointAngle);

        double dis[3] = {0,0,200};
        double R1_tmp[4][4] = {0.0};
        trans(R1_tmp,dis);

        M4p4(T,rfk.T03,R1_tmp);

        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // cell5: 右上斜圆柱
    bool transform5(double T[][4], double jointAngle[DOF]){
        rfk.updateTransforms(jointAngle);

        double dis[3] = {0,0,290};
        double R1_tmp[4][4] = {0.0};
        trans(R1_tmp,dis);

        // M4p4(T,rfk.T03,R1_tmp);

        double R_modify[4][4] = {0.0};
        dis[0] = 0.0; dis[1] = 20.0; dis[2] = 0.0;
        trans(R_modify,dis);

        M4p4p4(T,rfk.T03,R1_tmp,R_modify);

        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // cell6: 圆头圆柱
    bool transform6(double T[][4], double jointAngle[DOF]){
        rfk.updateTransforms(jointAngle);

        double dis[3] = {0,0,0};
        double R1_tmp[4][4] = {0.0};
        trans(R1_tmp,dis);

        M4p4(T,rfk.T04,R1_tmp);

        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // cell7: 左下斜圆柱
    bool transform7(double T[][4], double jointAngle[DOF]){
        rfk.updateTransforms(jointAngle);

        double Rtemp[4][4];
        rot_z(Rtemp,-jointAngle[4]);

        double T05[4][4] = {0.0};

        M4p4(T05,rfk.T05,Rtemp);

        double dis[3] = {0,0,-290};
        double R1_tmp[4][4] = {0.0};
        trans(R1_tmp,dis);

        // M4p4(T,rfk.T05,R1_tmp);

        double R_modify[4][4] = {0.0};
        dis[0] = 0.0; dis[1] = -15.0; dis[2] = 0.0;
        trans(R_modify,dis);

        M4p4p4(T,T05,R1_tmp,R_modify);

        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // cell8: 平头圆柱
    bool transform8(double T[][4], double jointAngle[DOF]){
        rfk.updateTransforms(jointAngle);

        double dis[3] = {0,0,-200};
        double R1_tmp[4][4] = {0.0};
        trans(R1_tmp,dis);

        M4p4(T,rfk.T05,R1_tmp);

        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // cell9: 左上斜圆柱
    bool transform9(double T[][4], double jointAngle[DOF]){
        rfk.updateTransforms(jointAngle);

        double dis[3] = {0,0,-110};
        double R1_tmp[4][4] = {0.0};
        trans(R1_tmp,dis);

        // M4p4(T,rfk.T05,R1_tmp);

        double R_modify[4][4] = {0.0};
        dis[0] = 0.0; dis[1] = -15.0; dis[2] = 0.0;
        trans(R_modify,dis);

        M4p4p4(T,rfk.T05,R1_tmp,R_modify);

        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // cell10: 平头圆柱  （改成长方体）
    bool transform10(double T[][4], double jointAngle[DOF]){
        rfk.updateTransforms(jointAngle);

        double dis[3] = {0,-20,0};
        double R1_tmp[4][4] = {0.0};
        trans(R1_tmp,dis);

        M4p4(T,rfk.T07,R1_tmp);

        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // cell11: 平头圆柱  
    bool transform11(double T[][4], double jointAngle[DOF]){
        rfk.updateTransforms(jointAngle);

        double dis[3] = {0,0,100};
        double R1_tmp[4][4] = {0.0};
        trans(R1_tmp,dis);

        M4p4(T,rfk.T07,R1_tmp);

        return true;
    }
    
    class RobotFilter{

        public:
            RobotFilter(){

                // cell0 圆台
                Point upPoint0(0,0,85.0,120), lowPoint0(0,0,-85.0,120);
                vector<Point> upPoints0; upPoints0.push_back(upPoint0);
                vector<Point> lowPoints0; lowPoints0.push_back(lowPoint0);
                Shape cell0(CYLINDER,upPoints0,lowPoints0,transform0);

                // cell1 右上斜圆柱
                upPoint0.x = 0; upPoint0.y = -25.5; upPoint0.z = 80; upPoint0.r = 150/2.0;
                lowPoint0.x = 0; lowPoint0.y = 25.5; lowPoint0.z = -80; lowPoint0.r = 150/2.0;
                upPoints0.clear();upPoints0.push_back(upPoint0);
                lowPoints0.clear(); lowPoints0.push_back(lowPoint0);
                Shape cell1(CYLINDER,upPoints0,lowPoints0,transform1);

                // cell2: 圆头圆柱
                upPoint0.x = 0; upPoint0.y = 0; upPoint0.z = 110; upPoint0.r = 145/2.0;
                lowPoint0.x = 0; lowPoint0.y = 0; lowPoint0.z = -110; lowPoint0.r = 145/2.0;
                upPoints0.clear();upPoints0.push_back(upPoint0);
                lowPoints0.clear(); lowPoints0.push_back(lowPoint0);
                Shape cell2(CYLINDER,upPoints0,lowPoints0,transform2);

                // cell3: 右下斜圆柱
                upPoint0.x = 0; upPoint0.y = -25.5; upPoint0.z = 80; upPoint0.r = 150/2.0;
                lowPoint0.x = 0; lowPoint0.y = 25.5; lowPoint0.z = -80; lowPoint0.r = 150/2.0;
                upPoints0.clear();upPoints0.push_back(upPoint0);
                lowPoints0.clear(); lowPoints0.push_back(lowPoint0);
                Shape cell3(CYLINDER,upPoints0,lowPoints0,transform3);

                // cell4: 平头圆柱
                upPoint0.x = 0; upPoint0.y = 0; upPoint0.z = 40; upPoint0.r = 150/2.0;
                lowPoint0.x = 0; lowPoint0.y = 0; lowPoint0.z = -40; lowPoint0.r = 150/2.0;
                upPoints0.clear();upPoints0.push_back(upPoint0);
                lowPoints0.clear(); lowPoints0.push_back(lowPoint0);
                Shape cell4(CYLINDER,upPoints0,lowPoints0,transform4);

                // cell5: 右上斜圆柱
                upPoint0.x = 0; upPoint0.y = 25.5; upPoint0.z = 80; upPoint0.r = 150/2.0;
                lowPoint0.x = 0; lowPoint0.y = -25.5; lowPoint0.z = -80; lowPoint0.r = 150/2.0;
                upPoints0.clear();upPoints0.push_back(upPoint0);
                lowPoints0.clear(); lowPoints0.push_back(lowPoint0);
                Shape cell5(CYLINDER,upPoints0,lowPoints0,transform5);

                // cell6: 圆头圆柱
                upPoint0.x = 0; upPoint0.y = 0; upPoint0.z = 110; upPoint0.r = 145/2.0;
                lowPoint0.x = 0; lowPoint0.y = 0; lowPoint0.z = -110; lowPoint0.r = 145/2.0;
                upPoints0.clear();upPoints0.push_back(upPoint0);
                lowPoints0.clear(); lowPoints0.push_back(lowPoint0);
                Shape cell6(CYLINDER,upPoints0,lowPoints0,transform6);

                // cell7: 左下斜圆柱
                upPoint0.x = 0; upPoint0.y = 25.5; upPoint0.z = 80; upPoint0.r = 150/2.0;
                lowPoint0.x = 0; lowPoint0.y = -25.5; lowPoint0.z = -80; lowPoint0.r = 150/2.0;
                upPoints0.clear();upPoints0.push_back(upPoint0);
                lowPoints0.clear(); lowPoints0.push_back(lowPoint0);
                Shape cell7(CYLINDER,upPoints0,lowPoints0,transform7);

                // cell8: 平头圆柱
                upPoint0.x = 0; upPoint0.y = 0; upPoint0.z = 40; upPoint0.r = 150/2.0;
                lowPoint0.x = 0; lowPoint0.y = 0; lowPoint0.z = -40; lowPoint0.r = 150/2.0;
                upPoints0.clear();upPoints0.push_back(upPoint0);
                lowPoints0.clear(); lowPoints0.push_back(lowPoint0);
                Shape cell8(CYLINDER,upPoints0,lowPoints0,transform8);

                // cell9: 左上斜圆柱
                upPoint0.x = 0; upPoint0.y = -25.5; upPoint0.z = 80; upPoint0.r = 150/2.0;
                lowPoint0.x = 0; lowPoint0.y = 25.5; lowPoint0.z = -80; lowPoint0.r = 150/2.0;
                upPoints0.clear();upPoints0.push_back(upPoint0);
                lowPoints0.clear(); lowPoints0.push_back(lowPoint0);
                Shape cell9(CYLINDER,upPoints0,lowPoints0,transform9);

                // cell10: 平头圆柱  （改成长方体）
                upPoint0.x = 0; upPoint0.y = 0; upPoint0.z = 70; upPoint0.r = 180/2.0;
                lowPoint0.x = 0; lowPoint0.y = 0; lowPoint0.z = -100; lowPoint0.r = 180/2.0;
                upPoints0.clear();upPoints0.push_back(upPoint0);
                lowPoints0.clear(); lowPoints0.push_back(lowPoint0);
                Shape cell10(CYLINDER,upPoints0,lowPoints0,transform10);

                // cell11: 平头圆柱  
                upPoint0.x = 0; upPoint0.y = 0; upPoint0.z = 60; upPoint0.r = 110/2.0;
                lowPoint0.x = 0; lowPoint0.y = 0; lowPoint0.z = -40; lowPoint0.r = 110/2.0;
                upPoints0.clear();upPoints0.push_back(upPoint0);
                lowPoints0.clear(); lowPoints0.push_back(lowPoint0);
                Shape cell11(CYLINDER,upPoints0,lowPoints0,transform11);

                _shapes.clear();
                _shapes.push_back(cell0);_shapes.push_back(cell1);_shapes.push_back(cell2);
                _shapes.push_back(cell3);_shapes.push_back(cell4);_shapes.push_back(cell5);
                _shapes.push_back(cell6);_shapes.push_back(cell7);_shapes.push_back(cell8);
                _shapes.push_back(cell9);_shapes.push_back(cell10);_shapes.push_back(cell11);

            }
            ~RobotFilter(){}

            int getNumOfShapes(){return _shapes.size();}
            Shape getShape(int i,double jointAngle[DOF]){
                _shapes[i].updateTransfrom(jointAngle);
                return _shapes[i];
            }


        private:
            vector<Shape> _shapes;
            double _jointAngle[DOF];
    };

}


