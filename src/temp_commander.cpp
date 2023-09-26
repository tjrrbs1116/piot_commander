#include "temp_commander.hpp"
#include "temp_navigator.hpp"
#define degtorad 0.0174533

using namespace std::chrono_literals;

temp_commander::temp_commander(const rclcpp::NodeOptions & options)
: Node("temp_commander" , options)

{
    auto custom_qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(1));   
    auto subscriber_options = rclcpp::SubscriptionOptions();  
    RCLCPP_INFO(get_logger(), "temp_commander start");
    amcl_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose",custom_qos, std::bind(&temp_commander::amcl_pose_callback,this,std::placeholders::_1),subscriber_options);
    initial_pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", rclcpp::QoS(10));
    timer_ = this->create_wall_timer(100ms,std::bind(&temp_commander::timerCallback,this));
    // timer2_ = this->create_wall_timer(50ms,std::bind(&temp_commander::timerCallback2,this));
    // nodes = rclcpp::Node::make_shared("add_two_ints_client");
    // rclcpp::Client<nav2_msgs::action::NavigateToPose>::SharedPtr pp =
    // nodes->create_client<nav2_msgs::action::NavigateToPose>("add_two_ints");
    client_node_= rclcpp::Node::make_shared("yes");
    client_ptr_ =rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(client_node_, "navigate_to_pose");
    // server_timeout_ = 100;
        process = gosome;


}



using namespace std::placeholders;

void temp_commander::amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped& in){
    
    robot_pose_.x = in.pose.pose.position.x;
    robot_pose_.y = in.pose.pose.position.y;
    float x = in.pose.pose.orientation.x;
    float y = in.pose.pose.orientation.y;
    float z = in.pose.pose.orientation.z;
    float w = in.pose.pose.orientation.w;
    robot_pose_.deg = temp_commander::get_yaw(x,y,z,w);

    
    initialpose_callback_flag = true;

}





void temp_commander::setInitialPose_(float x_, float y_ , float deg_)
{
    RCLCPP_INFO(get_logger(), "set initial_pose");
    tf2::Quaternion myquat;
    
    myquat.setRPY(0,0,deg_*degtorad);
    myquat=myquat.normalize();

    rclcpp::Time rclcpp_time = now();
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = rclcpp_time;
    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    msg.pose.pose.position.z = 0.;
    msg.pose.pose.orientation.x = myquat[0];
    msg.pose.pose.orientation.y = myquat[1];
    msg.pose.pose.orientation.z = myquat[2];
    msg.pose.pose.orientation.w = myquat[3];

    initial_pose_pub->publish(msg);

    RCLCPP_INFO(get_logger(), "set initial_pose good");

}







void temp_commander::timerCallback(){
    //////////////////////////////////////////////////////
    /////// decide process algorithm /////////////////////
    //////////////////////////////////////////////////////

    switch(process){

        case 0 :
        {   
            if(!initialpose_callback_flag){
                temp_commander::setInitialPose_(3,4,0);
            }
            else{initialpose_callback_flag=true; process= neutral;}
            break;
        }

        case 1:
        
        {
            RCLCPP_INFO(get_logger(),"go home5");
            
            if(!client_res){
                 RCLCPP_INFO(get_logger(),"go home");
                temp_commander::goTopose(3,4 ,40);
            }
            else{
                if(clinet_result!=1){RCLCPP_INFO(get_logger(),"go home");}
            }

            break;
        }

        case 2:{
                isTaskComplete();
            if(!client_res){
             temp_commander::goTopose(1.74,-0.65 ,40);
            }

            else { }
            break;
        }

        case neutral:{
            RCLCPP_INFO(get_logger(),"now");
            temp_commander::goTopose(1.74,-0.65 ,40);

            break ; 
        }
        case gocharge: {


            break;
        }



    }



}


void temp_commander::isTaskComplete(){

// int d = navigation_goal_handle_->get_status();
// RCLCPP_INFO(get_logger(),"%d",d);

}

void temp_commander::timerCallback2(){
    RCLCPP_INFO(get_logger(), "this is timer2");
}

void temp_commander::goToHome()
{
    if(!client_ptr_->wait_for_action_server(1s))
    {
        RCLCPP_INFO(get_logger(), "waiting ");
    }

    else{

        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();

        tf2::Quaternion myquat;
        myquat.setRPY(0,0,0.0*degtorad);
        myquat=myquat.normalize();

        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.pose.position.x = 3.0;
        goal_msg.pose.pose.position.y = 4.0;
        goal_msg.pose.pose.orientation.x = myquat[0];
        goal_msg.pose.pose.orientation.y = myquat[1];
        goal_msg.pose.pose.orientation.z = myquat[2];
        goal_msg.pose.pose.orientation.w = myquat[3];

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        // auto send_goal_options =rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();


        // send_goal_options.goal_response_callback = 
        // std::bind(&temp_commander::goal_response_callback,this,_1);
        // send_goal_options.feedback_callback =
        // std::bind(&temp_commander::feedback_callback, this, _1, _2);
        // send_goal_options.result_callback =
        // std::bind(&temp_commander::result_callback, this, _1);
        // auto future= client_ptr_->async_send_goal(goal_msg,send_goal_options);



        auto send_goal_options =rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
         std::bind(&temp_commander::goal_response_callback,this,_1);
        send_goal_options.result_callback =
        std::bind(&temp_commander::result_callback, this, _1);
        send_goal_options.feedback_callback =
        std::bind(&temp_commander::feedback_callback, this, _1, _2);


        auto future_goal_handle =client_ptr_->async_send_goal(goal_msg, send_goal_options);

}
}



void temp_commander::goTopose(float x_ , float y_, float deg_){

    if(!client_ptr_->wait_for_action_server(1s))
    {
        RCLCPP_INFO(get_logger(), "waiting ");
    }
    else{

        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();

        tf2::Quaternion myquat;
        myquat.setRPY(0,0,deg_*degtorad);
        myquat=myquat.normalize();

        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.pose.position.x = x_;
        goal_msg.pose.pose.position.y = y_;
        goal_msg.pose.pose.orientation.x = myquat[0];
        goal_msg.pose.pose.orientation.y = myquat[1];
        goal_msg.pose.pose.orientation.z = myquat[2];
        goal_msg.pose.pose.orientation.w = myquat[3];

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        // auto send_goal_options =rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();


        // send_goal_options.goal_response_callback = 
        // std::bind(&temp_commander::goal_response_callback,this,_1);
        // send_goal_options.feedback_callback =
        // std::bind(&temp_commander::feedback_callback, this, _1, _2);
        // send_goal_options.result_callback =
        // std::bind(&temp_commander::result_callback, this, _1);
        // auto future= client_ptr_->async_send_goal(goal_msg,send_goal_options);



        auto send_goal_options =rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
         std::bind(&temp_commander::goal_response_callback,this,_1);
        // send_goal_options.result_callback =
        // std::bind(&temp_commander::result_callback, this, _1);
          send_goal_options.result_callback = [this](auto) {
            navigation_goal_handle_.reset();};
        send_goal_options.feedback_callback =
        std::bind(&temp_commander::feedback_callback, this, _1, _2);


        future_goal_handle =client_ptr_->async_send_goal(goal_msg, send_goal_options);


         if (rclcpp::spin_until_future_complete(client_node_, future_goal_handle) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(),"gick");
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
        }

       
        navigation_goal_handle_ = future_goal_handle.get();
        if (!navigation_goal_handle_) {
            RCLCPP_ERROR(client_node_->get_logger(), "Goal was rejected by server");
            return;
        }
    }       


}








// void temp_commander::goal_response_callback(std::shared_future<NavigationGoalHandle::SharedPtr> future){}



void temp_commander::goal_response_callback(const NavigationGoalHandle::SharedPtr & future){
    

    // if (!future) {
    //     client_res =false;
    //   RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    // } else {
    //     client_res =true;
    //   RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    // }
}


void temp_commander::feedback_callback(NavigationGoalHandle::SharedPtr,const std::shared_ptr<const Nav2Pose::Feedback> feedback){

    // current_feedback = *feedback;
    // float x, y;
    // x = feedback->current_pose.pose.position.x ;
    // y = feedback->current_pose.pose.position.y ;
    // RCLCPP_INFO(client_node_->get_logger(),"feedback");


}

void temp_commander::result_callback(const NavigationGoalHandle::WrappedResult & result){
    
    // switch (result.code) {
    //   case rclcpp_action::ResultCode::SUCCEEDED:
    //   clinet_result =1;
    //   RCLCPP_INFO(client_node_->get_logger(), "SUCCEEDED");
    //     break;
    //   case rclcpp_action::ResultCode::ABORTED:
    //     clinet_result =2;
    //     RCLCPP_INFO(client_node_->get_logger(), "Goal was aborted");
    //     return;
    //   case rclcpp_action::ResultCode::CANCELED:
    //     clinet_result =3;
    //     RCLCPP_INFO(client_node_->get_logger(), "Goal was canceled");
    //     return;
    //   default:
    //     RCLCPP_INFO(client_node_->get_logger(), "Unknown result code");
    //     return;
    // }

}

