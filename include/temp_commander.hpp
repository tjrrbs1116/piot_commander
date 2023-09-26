#include "rclcpp/rclcpp.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "message_filters/subscriber.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <chrono>

#define set_pose 0
#define gohome 1
#define gosome 2
#define neutral 3
#define gocharge 5



struct robot_pose{

    float x ;
    float y ;
    float deg;

};


class temp_commander : public rclcpp::Node{

    public:

    using Nav2Pose = nav2_msgs::action::NavigateToPose;
    using NavigationGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
        explicit temp_commander ( const rclcpp::NodeOptions &);

    bool initialpose_callback_flag = false;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer2_;


    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_ptr_;

    std::shared_ptr<rclcpp::Node> nodes;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr pp;
    std::chrono::milliseconds server_timeout_ ;
    NavigationGoalHandle::SharedPtr navigation_goal_handle_;
    void setInitialPose_(float x , float y ,float deg);
    bool setInitialPose(float x, float y, float deg );
    void amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped& in);
    void timerCallback();
    void timerCallback2();
    void goTopose(float x, float y, float deg);
    void goToHome();
    void isTaskComplete();
    
    bool one_time=false;

    void goal_response_callback(const NavigationGoalHandle::SharedPtr &);
    void feedback_callback(NavigationGoalHandle::SharedPtr,const std::shared_ptr<const Nav2Pose::Feedback>);
    void result_callback(const NavigationGoalHandle::WrappedResult & result);


    std::shared_future<std::shared_ptr<temp_commander::NavigationGoalHandle>> future_goal_handle;

    rclcpp::Node::SharedPtr client_node_;
    inline float get_yaw(float x, float y, float z, float w){
      float roll , pitch ,yaw;
                float t0 = 2.0 * (w *x + y*z);
                float t1 = 1.0 - 2.0 * (x*x + y*y);
                roll = atan2(t0,t1);

                float t2 = 2.0 * (w*z + x*y);
                if(t2> 1.0){ t2 = 1.0; }
                if(t2<-1.0){ t2 = -1.0;}
                pitch = asin(t2);

                float t3 = 2.0 * (w*z + x*y);
                float t4 = 1.0 - 2.0 * (y*y + z*z);
                yaw = atan2(t3,t4);

return yaw;}



    private:
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub; 
        geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
        robot_pose robot_pose_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub;
        int process;
        bool flag =false;
        Nav2Pose::Feedback current_feedback;
        rclcpp::Time now_;
        bool client_res= false;
        unsigned char clinet_result = 255;
        // std::shared_future<std::shared_ptr<temp_commander::NavigationGoalHandle>> tt;
        // NavigationGoalHandle navi_handle;
        // NavigationGoalHandle::SharedPtr t;
        int i;
        
};

