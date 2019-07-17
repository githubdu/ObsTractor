#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include <math.h>

#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

#define    PI    (3.1415926)

class TeleopHuman
{
public:
  TeleopHuman();
  void keyLoop();

private:
  ros::NodeHandle nh_;
  double linear_, angular_, l_scale_, a_scale_;
  ros::Publisher twist_pub_;
};

TeleopHuman::TeleopHuman():
  linear_(0),
  angular_(0),
  l_scale_(0.2),
  a_scale_(0.2)
{
  nh_.param("scale_angular",a_scale_,a_scale_);
  nh_.param("scale_linear",l_scale_,l_scale_);

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void) sig;
  tcsetattr(kfd,TCSANOW,&cooked);
  ros::shutdown();
  exit(0);
}



int main(int argc, char **argv)
{
      ros::init(argc, argv, "human_controller");
      TeleopHuman teleop_human;
      signal(SIGINT,quit);
      teleop_human.keyLoop();

      /*
      ros::NodeHandle n;
      ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",100);
      ros::Rate loop_rate(10);
      int count = 0;
      while (ros::ok())
      {
            geometry_msgs::Twist msg;
             count ++;
             geometry_msgs::Vector3  v;
             v.x = 1.0*sin(count/100.0*2.0*PI);
             v.y = 1.0*cos(count/100.0*2.0*PI);
             v.z = 0.0;
             msg.linear = v;
             v.x = 0.0;
             v.y = 0.0;
             v.z = 0.0;
             msg.angular = v;
            chatter_pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
      }*/


      return 0;
}


void TeleopHuman::keyLoop()
{
  char c;
  bool dirty=false;
  ros::Rate loop_rate(100);

  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle.");


  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        angular_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        angular_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        linear_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        linear_ = -1.0;
        dirty = true;
        break;

      case 'w':
        ROS_DEBUG("ACC");
	l_scale_+=0.1;
  	a_scale_+=0.1;
	break;
      case 's':
	ROS_DEBUG("DEC");
	l_scale_-=0.1;
	a_scale_-=0.1;
	break;
    }
   

    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_*angular_;
    twist.linear.y = -l_scale_*linear_;
    if(dirty ==true)
    {
      twist_pub_.publish(twist);    
      dirty=false;
    }
                
    ros::spinOnce();
    loop_rate.sleep();

    twist.angular.z = 0.0;
    twist.linear.y = 0.0;
    twist_pub_.publish(twist);   
 
  }


  return;
}





