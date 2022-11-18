#include "ros/ros.h"
#include "move_base_msgs/MoveBaseActionGoal.h"

/**
 * Set goal for House World
 */
int main(int argc, char **argv)
{
  /* Default */
  float position_x = -13.7;
  float position_y = 0;

  ros::init(argc, argv, "goal_pub");

  ros::NodeHandle n("~");

  ros::Publisher pub = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10);

  ros::Rate loop_rate(1);

  ros::Rate poll_rate(100);

  n.getParam("x", position_x);
  n.getParam("y", position_y);

  ROS_INFO("Got parameter: x %f y %f", position_x, position_y);


  while(pub.getNumSubscribers() == 0)
    poll_rate.sleep();

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
   

    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    move_base_msgs::MoveBaseActionGoal msg;
    
    msg.goal.target_pose.header.frame_id="map";

    /* House map setting */
    //msg.goal.target_pose.pose.position.x=5.87;
    //msg.goal.target_pose.pose.position.y=3.98;
    //msg.goal.target_pose.pose.orientation.w=1.0;
    /* Room map setting*/
    msg.goal.target_pose.pose.position.x=position_x;
    msg.goal.target_pose.pose.position.y=position_y;
    msg.goal.target_pose.pose.orientation.w=1.0;    

    ROS_INFO("Target goal set: x = %f, y = %f", msg.goal.target_pose.pose.position.x, msg.goal.target_pose.pose.position.y);
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    
  return 0;
}

/* 

Topic: /move_base/goal 

Type: move_base_msgs/MoveBaseActionGoal

Details:

header: 
  seq: 6
  stamp: 
    secs: 420
    nsecs: 867000000
  frame_id: ''
goal_id: 
  stamp: 
    secs: 0
    nsecs:         0
  id: ''
goal: 
  target_pose: 
    header: 
      seq: 6
      stamp: 
        secs: 420
        nsecs: 867000000
      frame_id: "map"
    pose: 
      position: 
        x: 5.869647026062012
        y: 3.9774563312530518
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
*/