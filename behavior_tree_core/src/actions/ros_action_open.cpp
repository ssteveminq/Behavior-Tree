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

#include <actions/ros_action_open.h>
#include <string>

BT::ROSAction_Open::ROSAction_Open(std::string name) :
  ActionNode::ActionNode(name),
  action_client_(name, true)
{
    actionlib::SimpleActionClient<behavior_tree_core::BTAction> action_client_(get_name(), true);
    ROS_INFO("Waiting For the Acutator named %s to start", get_name().c_str());
    action_client_.waitForServer();  // will wait for infinite time until the server starts
    ROS_INFO("The Acutator %s has started", get_name().c_str());
    // thread_ start
    thread_ = std::thread(&ROSAction_Open::WaitForTick, this);
}

BT::ROSAction_Open::~ROSAction_Open() {}

void BT::ROSAction_Open::WaitForTick()
{
    while (true)
    {
        // Waiting for a tick to come
        tick_engine.Wait();

        // Running state
        node_result.status = BT::RUNNING;
        set_status(BT::RUNNING);
        // Perform action...
        ROS_INFO("I am running the request to %s", get_name().c_str());
        action_client_.sendGoal(goal);
        do
        {
            node_result = *(action_client_.getResult());  // checking the result
        }
        while (node_result.status == BT::RUNNING && get_status() != BT::HALTED);

        if (get_status() == BT::HALTED)
        {
            ROS_INFO("The Node is Halted");
            ROS_INFO("I am Halting the client");
            action_client_.cancelGoal();
        }
        else
        {
            ROS_INFO("The Server Has Replied");
            // Set this node status according to what the external node replied
            set_status((ReturnStatus)node_result.status);
        }
    }
}


void BT::ROSAction_Open::Halt()
{
    set_status(BT::HALTED);
}
