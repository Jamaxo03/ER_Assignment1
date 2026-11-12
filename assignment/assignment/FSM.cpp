#include <memory>
#include <chrono>
#include <cmath>
#include <set>

#include "rclcpp/rclcpp.hpp"
#include "assignment_interfaces/msg/marker_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MarkerSubscriber : public rclcpp::Node
{
public:
    MarkerSubscriber();

private:

// variables to save msg /MarkerArray
    int current_id_;
    double current_u_;
    double current_v_;
    double current_side_;
    bool marker_visible_ = false;

// set to save already executed markers
std::set<int> executed_markers_;

// state variable 
    enum State
    {
        START,
        MOVE,
        ALIGN,
        APPROACH,
        EXECUTE,
        ROTATE,
        FINISH
    };

    State current_state_;

// rotation angle variable 
    double rotation_angle_;

//robot current position variables
    double robot_x_; 
    double robot_y_;
    double robot_yaw_;

// markerCallback declaration
    void markerCallback(const assignment_interfaces::msg::MarkerArray::SharedPtr msg);

// odomCallback declaration
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

// fsm declaration
    void fsm();

// robot motion functions declarations

// move forward function  
    void moveForward(double speed);

// rotate function
    void rotate(double speed);

// stop robot function
    void stopRobot();

// markers array subscriber
    rclcpp::Subscription<assignment_interfaces::msg::MarkerArray>::SharedPtr marker_array_subscription_;

// odom subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

// timer subscriber
    rclcpp::TimerBase::SharedPtr timer_;

// /cmd_vel publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

};

// Marker subscriber constructor
MarkerSubscriber::MarkerSubscriber()
: Node("marker_subscriber"), 
    current_id_(0),
    current_u_(0.0),
    current_v_(0.0),
    current_side_(0.0),
    marker_visible_(false),
    current_state_(START),
    rotation_angle_(0.0),
    robot_x_(0.0),
    robot_y_(0.0),
    robot_yaw_(0.0)
    {
        marker_array_subscription_ =
            this->create_subscription<
                assignment_interfaces::msg::MarkerArray>(
                    "/marker_array",
                    10,
                    std::bind(
                        &MarkerSubscriber::markerCallback,
                        this,
                        std::placeholders::_1));

        odom_subscription_ =
            this->create_subscription<
                nav_msgs::msg::Odometry>(
                    "/odom",
                    10,
                    std::bind(
                        &MarkerSubscriber::odomCallback,
                        this,
                        std::placeholders::_1));

        timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&MarkerSubscriber::fsm, this));

        cmd_vel_pub_ =
            this->create_publisher<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                10);

    }

// markerCallback definition
void MarkerSubscriber::markerCallback(
    const assignment_interfaces::msg::MarkerArray::SharedPtr msg)
{
    marker_visible_ = false;

    double best_side = -1.0;

    for(const auto& marker : msg->markers)
    {
        //bool already_executed =
        //    executed_markers_.count(marker.id);

        if(executed_markers_.find(marker.id) == executed_markers_.end() &&
        marker.side > best_side)
        {
            best_side = marker.side;

            current_id_ = marker.id;
            current_u_ = marker.u;
            current_v_ = marker.v;
            current_side_ = marker.side;

            marker_visible_ = true;
        }
    }
}

// odomCallback definition
void MarkerSubscriber::odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;

        // conversione quaternion -> yaw
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;

        robot_yaw_ =
            std::atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy * qy + qz * qz)
            );

    }

// fsm definition
void MarkerSubscriber::fsm(){

    switch(current_state_){

        case START:

            RCLCPP_INFO(
                this->get_logger(),
                "Robot started. Entering MOVE state.");

            current_state_ = MOVE;
        break;

        case MOVE:
            // muovo il robot avanti dopo aver eseguito un istruzione (o in partenza)
            // robot fermo
            // deve ripartire (comando ripartire) 
            if(marker_visible_)
            {
                stopRobot();
                current_state_ = ALIGN;
            }
            else
            {
                moveForward(0.2);
            }

        break;

        case ALIGN:
        {
            // formula allineamento con il centro del marker    

            const double cx = 320.0;
            const double tolerance = 20.0;

            double error = current_u_ - cx;

            if(std::abs(error) < tolerance)
            {
                stopRobot();
                current_state_ = APPROACH;
            }
            else
            {
                double omega = -0.002 * error;
                rotate(omega);
            }


        break;
        }
        
        case APPROACH:
        {
            // 166 px distanza per eseguire istruzione

            double desired_side = 166.0;
            double tolerance = 5.0;

            if(current_id_ == 4)
            {
                desired_side = 168.0;
                tolerance = 0.5;
            }

            double error = desired_side - current_side_;

            if(std::abs(error) < tolerance)
            {
                stopRobot();
                current_state_ = EXECUTE;
            }
            else
            {
                double v = 0.005 * error;
                moveForward(v);
            }

        break;
        }
        
        case EXECUTE:

            // id recognition(switch)

            // update already used markers
            executed_markers_.insert(current_id_);

            switch(current_id_)
            {
                case 1:
                    // id 1 =  90° right
                    RCLCPP_INFO(
                    this->get_logger(),
                    "Marker 1 detected. Rotating 90 degrees right.");
                    rotation_angle_ = -M_PI / 2.0;
                    current_state_ = ROTATE;
                break;
                
                case 2:
                    // id 2 =  180°
                    RCLCPP_INFO(
                    this->get_logger(),
                    "Marker 2 detected. Rotating 180 degrees.");
                    rotation_angle_ = M_PI;
                    current_state_ = ROTATE;
                break;

                case 3:
                    // id 3 = 90° left
                    RCLCPP_INFO(
                    this->get_logger(),
                    "Marker 3 detected. Rotating 90 degrees left.");
                    rotation_angle_ = M_PI / 2.0;
                    current_state_ = ROTATE;
                break;

                case 4:
                  // id 4 = FINISH 
                  RCLCPP_INFO(
                    this->get_logger(),
                    "Marker 4 detected. Mission completed.");
                  current_state_ = FINISH;

                  timer_->cancel();

                break;

                default:
                    current_state_ = MOVE;
                break;
            }
                        
        break;

        case ROTATE:
        {
            // state to handle rotations
            static bool initialized = false;
            static double target_yaw;

            if(!initialized)
            {
                target_yaw =
                    std::atan2(
                        std::sin(robot_yaw_ + rotation_angle_),
                        std::cos(robot_yaw_ + rotation_angle_));
                initialized = true;
            }

            double error =
                std::atan2(
                    std::sin(target_yaw - robot_yaw_),
                    std::cos(target_yaw - robot_yaw_)
                );

            if(std::abs(error) < 0.05)
            {
                stopRobot();

                initialized = false;
                current_state_ = MOVE;
            }
            else
            {
                rotate(0.8 * error);
            }

        break;
        }

        case FINISH:
        {
            // stops the robot
            stopRobot();
            RCLCPP_INFO(
                this->get_logger(),
                "Robot stopped. Assignment completed.");

            timer_->cancel();

            break;
        }

        default:
        break;
    }  
 }



// function to move forward definition
void MarkerSubscriber::moveForward(double speed)
{
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = speed;
    cmd_vel_pub_->publish(cmd);
}

// function to rotate robot definition
void MarkerSubscriber::rotate(double speed)
{
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = speed;
    cmd_vel_pub_->publish(cmd);
}

// function to stop robot definition
void MarkerSubscriber::stopRobot()
{
    geometry_msgs::msg::Twist cmd;
    cmd_vel_pub_->publish(cmd);
}



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MarkerSubscriber>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
