#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/RandomPosition.h"
#include "rt2_assignment1/RobotpositionAction.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include <geometry_msgs/Twist.h>

bool start = false;

bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    return true;
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   ros::Publisher pub_vel= n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
   
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;

   actionlib::SimpleActionClient <rt2_assignment1::RobotpositionAction> ac("/go_to_point", true);
   rt2_assignment1::RobotpositionGoal goal;
   geometry_msgs::Twist velocity;

   bool goal_reached=false;

   
   while(ros::ok()){
   	ros::spinOnce();
   	if (start){
   		client_rp.call(rp);
		goal.x=rp.response.x;
		goal.y=rp.response.y;
		goal.theta=rp.response.theta;

		ac.sendGoal(goal);

        while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
           ros::spinOnce();
           if(start==false)
           {
           
              ac.cancelAllGoals();
              goal_reached=false;

              break;

           }
            goal_reached=true;
        }
	
    }

    else
    {
 
            velocity.linear.x=0;
            velocity.angular.z=0;
            pub_vel.publish(velocity);

    }

    if(goal_reached)
    {
         std::cout << "\nPosition reached" << std::endl;
         goal_reached=false;
    }
           
   	}
   return 0;
}
