#include <ros/ros.h>
#include <dummiebot_control/Dummie.h>
#include <termios.h>
#include <iostream>

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
  }

  tcsetattr(STDIN_FILENO, TCSANOW, &oldTerm);
  return c;
}

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "keyboard_control_node");

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<dummiebot_control::Dummie>("dummie_move", 1000);
  ros::Rate loop_rate(10);

  while(ros::ok()){

    dummiebot_control::Dummie msg;

    pub.publish(msg);
    ros::spinOnce();

    char c = getch();

    switch(c){
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
        msg.linear = msg.angular = msg.body_pitch = msg.body_yaw = 0;
    }

    pub.publish(msg);
    ros::spinOnce();
  }


  return 0;
}
