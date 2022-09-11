#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <fstream>
#include <iostream>
#include<sstream>
#include<stdlib.h>
#include<time.h>
#include<math.h>
#include<vector>
#include<algorithm>

using namespace std;

const int N= 6;
const int C = 5;
const int T = 5;
const double cross_rate = 00.75;
const double muta_rate = 0.1;
const int I =500;

double citys_position[15][3];

void readPoint()
{
  char data[100];

  // 以读模式打开文件
  ifstream infile; 
  infile.open("/home/zhx/unitree_sim/src/simple_navigation_goals/src/point.txt"); 
  string line;
  int i = 0;
  while(getline(infile,line)&&(i < C))
  {
    stringstream s(line);
    while(s>>citys_position[i][0] >> citys_position[i][1] >> citys_position[i][2] )
    {
        cout <<citys_position[i][0] <<" " << citys_position[i][1] << " " << citys_position[i][2]<< endl;
        i++;
    }
  }
   // 关闭打开的文件
   infile.close();

}

void start()
{
    srand((unsigned)time(NULL));
	readPoint();
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
move_base_msgs::MoveBaseGoal goal[C];

int main(int argc, char** argv)
{
	ros::init(argc, argv, "simple_navigation_goals");
	ros::NodeHandle n;
	start();
    MoveBaseClient ac("move_base", true);

    for(int i = 0; i < C ; i++)
	{
		
		goal[i].target_pose.header.frame_id = "map";
  		goal[i].target_pose.header.stamp = ros::Time::now();
			
		goal[i].target_pose.pose.position.x = citys_position[i][0];
		goal[i].target_pose.pose.position.y = citys_position[i][1];
		goal[i].target_pose.pose.orientation.z = 0;
		goal[i].target_pose.pose.orientation.w = 1;
        cout << goal[i].target_pose.pose.position.x << goal[i].target_pose.pose.position.y << endl;
		ac.sendGoal(goal[i]);
		ac.waitForResult();

		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("----------reach goal");
        }
    }
}

// int main(int argc, char** argv)
// {
// 	ros::init(argc, argv, "simple_navigation_goals");
// 	ros::NodeHandle n;

// 	MoveBaseClient ac("move_base", true);

// 	move_base_msgs::MoveBaseGoal tmp_goal[4];


// 	tmp_goal[0].target_pose.header.frame_id = "map";
// 	tmp_goal[0].target_pose.header.stamp = ros::Time::now();
// 	tmp_goal[0].target_pose.pose.position.x = 3.96;
// 	tmp_goal[0].target_pose.pose.position.y = 0.91;
// 	tmp_goal[0].target_pose.pose.orientation.z = 0;
// 	tmp_goal[0].target_pose.pose.orientation.w = 1;

// 	tmp_goal[1].target_pose.header.frame_id = "map";
// 	tmp_goal[1].target_pose.header.stamp = ros::Time::now();
// 	tmp_goal[1].target_pose.pose.position.x = 6.1;
// 	tmp_goal[1].target_pose.pose.position.y = 0.67;
// 	tmp_goal[1].target_pose.pose.orientation.z = 0;
// 	tmp_goal[1].target_pose.pose.orientation.w = 1;

// 	tmp_goal[2].target_pose.header.frame_id = "map";
// 	tmp_goal[2].target_pose.header.stamp = ros::Time::now();
// 	tmp_goal[2].target_pose.pose.position.x = 6.37;
// 	tmp_goal[2].target_pose.pose.position.y = -2.9;
// 	tmp_goal[2].target_pose.pose.orientation.z = 0;
// 	tmp_goal[2].target_pose.pose.orientation.w = 1;

// 	tmp_goal[3].target_pose.header.frame_id = "map";
// 	tmp_goal[3].target_pose.header.stamp = ros::Time::now();
// 	tmp_goal[3].target_pose.pose.position.x = 5.6;
// 	tmp_goal[3].target_pose.pose.position.y = -3.00;
// 	tmp_goal[3].target_pose.pose.orientation.z = 1;
// 	tmp_goal[3].target_pose.pose.orientation.w = 0;
// 	while(!ac.waitForServer(ros::Duration(5.0))){
// 	ROS_INFO("Waiting for the move_base action server to come up");
// 	}
// 	for(int i = 0; i < 4; ++i) {
// 		MoveBaseClient ac("move_base", true);
// 		cout << "sending" <<endl;
// 		ac.sendGoal(tmp_goal[i]);
// 		cout << "waiting" <<endl;
// 		ac.waitForResult();

// 		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
// 			ros::Duration(2).sleep();
// 		}
// 	}

// }