/* Copyright (C) 2015-2017 Michele Colledanchise - All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <behavior_tree_core/BTAction.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#include "handle_tracking/objectfinder.h"

#include <string>

enum Status {RUNNING, SUCCESS, FAILURE};


bool received_bool=false;

class BTAction
{
protected:
    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise strange errors may occur.
    actionlib::SimpleActionServer<behavior_tree_core::BTAction> as_;
    std::string action_name_;
    bool Oncetrue;

    // create messages that are used to published feedback/result
    behavior_tree_core::BTFeedback feedback_;
    behavior_tree_core::BTResult result_;

public:
    explicit BTAction(std::string name) :
        as_(nh_, name, boost::bind(&BTAction::execute_callback, this, _1), false),
        action_name_(name)
    {
        // start the action server (action in sense of Actionlib not BT action)
        as_.start();
        ROS_INFO("Condition Server Started");
    }

    ~BTAction(void)
    { }
    void execute_callback(const behavior_tree_core::BTGoalConstPtr &goal)
    {
        ROS_INFO("Condition_checker");
        // ros::Publisher Pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/detected_handle_pos_condition",50,true);
        // ros::ServiceClient client_handlefinder = nh_.serviceClient<handle_tracking::objectfinder>("track_handle");
        // handle_tracking::objectfinder track_srv;
        // track_srv.request.entry=false;

        bool  Is_found=false;

        if(as_.isPreemptRequested())
        {
            ROS_INFO("Action Halted");
            // set the action state to preempted
            as_.setPreempted();
            // break;
        }

        boost::shared_ptr<std_msgs::Int8 const> sharedPtr;
        sharedPtr  = ros::topic::waitForMessage<std_msgs::Int8>("/detection/number_of_detected_human", ros::Duration(10));
        std_msgs::Int8 human_num = (*sharedPtr);

        int humannum=static_cast<int>(human_num.data);

        // ROS_INFO("human num : %d",humannum);

        std::cout<<"human num : "<< humannum<<std::endl;
        if(humannum>0)
        {
            set_status(SUCCESS);
        }
        else{

             set_status(FAILURE);
        }


        //previous code---------------------------------------------------------
        // if(client_handlefinder.call(track_srv)){

        //     Is_found = track_srv.response.handle_is_found;
        //     geometry_msgs::PoseStamped handlepose = track_srv.response.best_grasp_pose;
        //     Pose_pub.publish(handlepose);
        //     ROS_INFO("service Succeeded");
            
        //     set_status(SUCCESS);

        //     // set_status(SUCCESS);
        //     // if (Is_found)
        //     // {
        //     //     set_status(SUCCESS);
        //     // }
        //     // else
        //     // {
        //     //     set_status(FAILURE);
        //     // }
        // }
        // else
        // {   
        //     set_status(FAILURE);
        // }
        //---------------------------------------------------------previous code
     
    }

    //  returns the status to the client (Behavior Tree)
    void set_status(int status)
    {
        // Set The feedback and result of BT.action
        feedback_.status = status;
        result_.status = feedback_.status;
        // publish the feedback
        as_.publishFeedback(feedback_);
        // setSucceeded means that it has finished the action (it has returned SUCCESS or FAILURE).
        as_.setSucceeded(result_);

        switch (status)  // Print for convenience
        {
        case SUCCESS:
            ROS_INFO("Condition %s Succeeded", ros::this_node::getName().c_str() );
            break;
        case FAILURE:
            ROS_INFO("Condition %s Failed", ros::this_node::getName().c_str() );
            break;
        default:
            break;
        }
    }
};

void localization_callback(const std_msgs::Bool::ConstPtr& msg)
{
    // ROS_INFO("Recieved_callback");
    // received_bool=msg->data;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bh_localization");
    ROS_INFO("Enum: %d", RUNNING);
    ROS_INFO("condition Ready for Ticks");
    BTAction bt_action(ros::this_node::getName());
    
    // ros::NodeHandle n;
    // ros::Rate loop_rate(50);

    // while (ros::ok())
    // {

    //  ros::spinOnce();
    //  loop_rate.sleep();  


    // }

    ros::spin();



    return 0;
}
