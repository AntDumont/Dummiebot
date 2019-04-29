#include <ros/ros.h>
#include <dummiebot_control/Dummie.h>


#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42

int main(int argc, char const *argv[]) {

  ros::init(argc, argv, "keyboard_control_node");

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<dummiebot_control::Dummie>("dummie_move", 1000);
  ros::Rate loop_rate(10);

  while(ros::ok()){

    dummiebot_control::Dummie msg;

    switch(getchar()){
      case 27:
        getchar();
        switch(getchar()){
          case 65: //UP
            msg.linear = 1.0;
            break;
          case 66: //DOWN
            msg.linear = -1.0;
            break;
          case 67: //RIGHT
            msg.angular = -1.0;
            break;
          case 68: //LEFT
            msg.angular = 1.0;
            break;
          default:
            msg.liner = msg.angular = 0;
        }
        break;
      case 'z':
        msg.body_pitch = 1.0;
        break;
      case 's':
        msg.body_pitch = -1.0;
        break;
      case 'd':
        msg.body_yaw = -1.0;
        break;
      case 'q':
        msg.body_yaw = 1.0;
        break;
      default:
        msg.body_pitch = msg.body_yaw = 0;
    }

    pub.publish(msg);

    ros::spinOnce();
  }


  return 0;
}
