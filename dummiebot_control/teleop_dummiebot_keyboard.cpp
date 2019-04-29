#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <iostream>

#define PI 3.14159

class KeyboardControl{

public:
  KeyboardControl(){

    bool ARROW_KEY = false;

    pub_y_rotation = nh.advertise<std_msgs::Float64>("/dummiebot/y_rotation_controller/command", 1);
    pub_z_rotation = nh.advertise<std_msgs::Float64>("/dummiebot/z_rotation_controller/command", 1);
    pub_wheel = nh.advertise<geometry_msgs::Twist>("/dummiebot/mobile_base_controller/cmd_vel", 1);

  }

  void mainLoop(){

    while(ros::ok()){

      char c = getch();

      switch(c){
        case 65: //UP
          wheel_msg.linear.x = 1.5;
          wheel_msg.angular.z = 0.0;
          break;
        case 66: //DOWN
          wheel_msg.linear.x = -1.5;
          wheel_msg.angular.z = 0.0;
          break;
        case 67: //RIGHT
          //wheel_msg.linear.x = 0.0;
          wheel_msg.angular.z = -2.0;
          break;
        case 68: //LEFT
          //wheel_msg.linear.x = 0.0;
          wheel_msg.angular.z = 2.0;
          break;
        case 'z':
          if(y_rot_msg.data <= PI/4){
            y_rot_msg.data += 0.04;
            pub_y_rotation.publish(y_rot_msg);
          }
          break;
        case 's':
          if(y_rot_msg.data >= -PI/4){
            y_rot_msg.data -= 0.04;
            pub_y_rotation.publish(y_rot_msg);
          }
          break;
        case 'd':
          if(z_rot_msg.data >= -PI/2){
            z_rot_msg.data -= 0.04;
            pub_z_rotation.publish(z_rot_msg);
          }
          break;
        case 'q':
          if(z_rot_msg.data <= PI/2){
            z_rot_msg.data += 0.04;
            pub_z_rotation.publish(z_rot_msg);
          }
          break;
        default:
          break;
      }

      //if(ARROW_KEY){
        pub_wheel.publish(wheel_msg);
      //}

      ros::spinOnce();
    }
  }

  //Version of getchar that disable the buffering of the terminal so it is a non-blocking getchar.
  int getch(){
    static struct termios oldTerm, newTerm;
    tcgetattr(STDIN_FILENO, &oldTerm);
    newTerm = oldTerm;
    newTerm.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newTerm);

    int c = getchar();
    if(c==27){ //If the character is a special input
      getchar();
      c = getchar();
      //ARROW_KEY = true;
    }
    else{
      //ARROW_KEY = false;
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldTerm);
    return c;
  }

private:
  bool ARROW_KEY;

  ros::NodeHandle nh;
  ros::Publisher pub_y_rotation;
  ros::Publisher pub_z_rotation;
  ros::Publisher pub_wheel;

  std_msgs::Float64 y_rot_msg, z_rot_msg;
  geometry_msgs::Twist wheel_msg;
};




int main(int argc, char *argv[]) {

  ros::init(argc, argv, "keyboard_control_node");

  KeyboardControl kb;

  kb.mainLoop();

  return 0;
}
