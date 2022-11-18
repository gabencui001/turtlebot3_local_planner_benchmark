#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include <fstream>
#include <vector>
#include <tf/tf.h>
//#include <pair>

using namespace std;

/*
Todo:

Count recovery status,
Update global struct and print at the end of simulation (after global plan coords).

*/

/* GLOBAL CONSTANTS AND STRUCTURES */

struct data {

  geometry_msgs::Point position_cur;
  float travelled_path = 0;
  float global_plan_length = 0;
  bool is_goal_reached = 0;
  bool is_goal_set = 0;
  vector<pair<float,float>> GlobalPlan;
  float OdomFluctuationRatio = 0;
  int odomCounter = 0;

} dataFile;

geometry_msgs::Point position_old;
double yaw_old = 0;
float fluctuation_sum = 0;
bool global_path_init = 0;
fstream fs;
bool is_goal_reached = 0;

/* Goal Status Subscriber Function */

void getGoalStatus(const actionlib_msgs::GoalStatusArray &msg){

/* Wait for Goal_Pub */

if(!msg.status_list.empty()){

switch (msg.status_list[0].status) {
  case 3: //SUCCEEDED
    
  if(!is_goal_reached){
    is_goal_reached = 1;
    //Print out global plan coords
    fs.open("coords.txt", fstream::in | fstream::out | fstream::app); //This is generated in dir: ./.ros/
    fs << "Global plan coords: x;y" << endl;

    for(int i = 0; i < dataFile.GlobalPlan.size(); i++){

    fs << dataFile.GlobalPlan[i].first<<";"<<dataFile.GlobalPlan[i].second<<endl;

    }
    fs.close();
  }

  ros::shutdown();
  break;

  case 1: //ACTIVE
    //Set global constant
  dataFile.is_goal_set = 1;
  break;

  case 4: //ABORTED
    //Print out global plan coords
  fs.open("coords.txt", fstream::in | fstream::out | fstream::app);
  fs << "Global plan coords: x;y" << endl;

  for(int i = 0; i < dataFile.GlobalPlan.size(); i++){

  fs << dataFile.GlobalPlan[i].first<<";"<<dataFile.GlobalPlan[i].second<<endl;
  
  }

  fs << "ABORTED!";
  fs.close();
  ros::shutdown();
  break;

}
}
}

/* Odometry Subscriber Function */

void calculate_travelled_path(const nav_msgs::Odometry &msg){

  geometry_msgs::Point position_cur = msg.pose.pose.position;
  geometry_msgs::Quaternion orientation_cur = msg.pose.pose.orientation;
  dataFile.position_cur = position_cur;
  double yaw_cur = tf::getYaw(msg.pose.pose.orientation);

  /* Wait for first odom data sample to be fetched */

  if(dataFile.odomCounter != 0){

  if(position_old != position_cur)dataFile.travelled_path += sqrt(pow((position_old.x - position_cur.x),2) + pow((position_old.y - position_cur.y), 2));

  /*
  Real Fluctuation Ratio Calculation
  */

  fluctuation_sum += abs(yaw_cur - yaw_old);
}

position_old = position_cur;
yaw_old = yaw_cur;

/* Don't divide by zero! */
if(dataFile.travelled_path != 0)dataFile.OdomFluctuationRatio = fluctuation_sum / dataFile.travelled_path;

dataFile.odomCounter++;
}

/* Global Plan Path Subscriber Function */

void calculateLength(const nav_msgs::Path &msg){

auto array_length = end(msg.poses) - begin(msg.poses);
float sum = 0;
float fluctuation_sum = 0;

/* Wait for proper path */

if(array_length > 1){
for (int i =0 ; i< array_length - 1; i++){

  geometry_msgs::Point position_a = msg.poses[i].pose.position;
  geometry_msgs::Point position_b = msg.poses[i+1].pose.position;

  //Calculate CD (Cumulative Distance) and FR (Fluctuation Ratio) - Global Planner
  if(position_a != position_b){

    sum += sqrt(pow((position_a.x - position_b.x),2) + pow((position_a.y - position_b.y), 2));
    
    }
  
}
}

//Update global struct
dataFile.global_plan_length = sum;

/* Fetch initial Global Plan Coords */

if(!global_path_init){
  //Save initial global plan length
  global_path_init = 1;

  //Save global planner coords into .csv file
  vector<pair<float,float>> w;
  pair<float,float> p;

  for(int i = 0; i< array_length; i++){
    p.first = msg.poses[i].pose.position.x;
    p.second = msg.poses[i].pose.position.y;
    w.push_back(p);
  }
  dataFile.GlobalPlan = w;
}

}

/* --------- Main Application ---------*/

int main(int argc, char **argv)
{

  /* Initialise output file stream */
  fs.open ("coords.txt", fstream::in | fstream::out | fstream::trunc);
  fs << "id;time;x_odom;y_odom;fr_odom;travelled_path;global_plan_length;global_plan_fr" << endl;
  fs.close();

  ros::init(argc, argv, "debug_service");
  ros::NodeHandle n;
  ros::Rate loop_rate(5);

/* Odom sub */
  ros::Subscriber sub1 = n.subscribe("odom", 100, &calculate_travelled_path);

/* Global Plan sub */
  ros::Subscriber sub2 = n.subscribe("move_base/NavfnROS/plan", 100, &calculateLength);

/* Goal Status sub */
  ros::Subscriber sub3 = n.subscribe("move_base/status", 100, &getGoalStatus);

  ROS_INFO("Debug service is running!");

/* Loop Counter */
int counter = 0;

/* File stream Loop */
while (ros::ok()){
  if(dataFile.is_goal_set == 1){
  counter++;
  fs.open ("coords.txt", fstream::in | fstream::out | fstream::app);
  fs << counter << ";" << ros::Time::now() << ";" << dataFile.position_cur.x << ";" << dataFile.position_cur.y << ";" << dataFile.OdomFluctuationRatio << ";" << dataFile.travelled_path << ";" << dataFile.global_plan_length << endl;
  fs.close();
  }

  ros::spinOnce();
  loop_rate.sleep();
}

return 0;
}