#include <memory>
#include <chrono>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "assignment_interfaces/msg/marker_array.hpp"
#include "assignment_interfaces/srv/take_photo.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/camera_info.hpp"


using TakePhoto = assignment_interfaces::srv::TakePhoto;

class MarkerSubscriber : public rclcpp::Node
{
public:
    MarkerSubscriber();

private:

// variables to save msg /MarkerArray
    int target_id_;
    double current_u_;
    double current_v_;
    double current_side_;
    bool marker_visible_ = false;
    bool service_response_received_ = false;
    bool service_request_sent_ = false;
    bool service_response_success_ = false;

    // becomes true at the first /odom message: the simulation is running
    bool odom_received_ = false;

    // camera intrinsics from camera_info (fx = K[0], cx = K[2])
    double fx_ = 0.0;
    double cx_ = 0.0;
    bool camera_info_received_ = false;

    // ROS parameters (see config/sim.yaml and config/rosbot.yaml)
    double reference_side_;
    double reference_fx_;
    double forward_speed_;
    double k_omega_;
    double k_v_;
    double u_tolerance_;
    double side_tolerance_;
    double k_rot_;
    double yaw_tolerance_;

// state variable 
    enum State
    {
        START,
        MOVE,
        ALIGN_AND_APPROACH,
        TAKE_PHOTO,
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

// cameraInfoCallback declaration
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

// fsm declaration
    void fsm();

// robot motion functions declarations

// move forward function  
    void moveForward(double speed);

// rotate function
    void rotate(double speed);

// general twist function
    void twist(double v, double omega);

// stop robot function
    void stopRobot();

// Sends request to take_photo_service to take photo of and publish it modfied
    void take_photo_request();

// markers array subscriber
    rclcpp::Subscription<assignment_interfaces::msg::MarkerArray>::SharedPtr marker_array_subscription_;

// odom subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

// camera_info subscriber
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;

// timer subscriber
    rclcpp::TimerBase::SharedPtr timer_;

// /cmd_vel publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

// service take_photo
    rclcpp::Client<TakePhoto>::SharedPtr take_photo_client_;

};

// Marker subscriber constructor
MarkerSubscriber::MarkerSubscriber()
: Node("marker_subscriber"), 
    target_id_(1),
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

        // ROS parameters: defaults = simulation, overridden by config/sim.yaml or config/rosbot.yaml
        const std::string marker_array_topic =
            this->declare_parameter<std::string>("marker_array_topic", "/marker_array");
        const std::string odom_topic =
            this->declare_parameter<std::string>("odom_topic", "/odometry/filtered");
        const std::string cmd_vel_topic =
            this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        const std::string camera_info_topic =
            this->declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");

        reference_side_ = this->declare_parameter<double>("reference_side", 166.0);
        reference_fx_   = this->declare_parameter<double>("reference_fx", 381.36);
        forward_speed_  = this->declare_parameter<double>("forward_speed", 0.2);
        k_omega_        = this->declare_parameter<double>("k_omega", 0.002);
        k_v_            = this->declare_parameter<double>("k_v", 0.005);
        u_tolerance_    = this->declare_parameter<double>("u_tolerance", 20.0);
        side_tolerance_ = this->declare_parameter<double>("side_tolerance", 5.0);
        k_rot_          = this->declare_parameter<double>("k_rot", 0.8);
        yaw_tolerance_  = this->declare_parameter<double>("yaw_tolerance", 0.05);


        marker_array_subscription_ =
            this->create_subscription<
                assignment_interfaces::msg::MarkerArray>(
                    marker_array_topic,
                    10,
                    std::bind(
                        &MarkerSubscriber::markerCallback,
                        this,
                        std::placeholders::_1));

        odom_subscription_ =
            this->create_subscription<
                nav_msgs::msg::Odometry>(
                    odom_topic,
                    rclcpp::SensorDataQoS(),
                    std::bind(
                        &MarkerSubscriber::odomCallback,
                        this,
                        std::placeholders::_1));

        camera_info_subscription_ =
            this->create_subscription<
                sensor_msgs::msg::CameraInfo>(
                    camera_info_topic,
                    rclcpp::SensorDataQoS(),
                    std::bind(
                        &MarkerSubscriber::cameraInfoCallback,
                        this,
                        std::placeholders::_1));

        timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&MarkerSubscriber::fsm, this));

        cmd_vel_pub_ =
            this->create_publisher<geometry_msgs::msg::Twist>(
                cmd_vel_topic,
                10);

        take_photo_client_ = this->create_client<TakePhoto>("take_photo");
    }

// markerCallback definition
void MarkerSubscriber::markerCallback(
    const assignment_interfaces::msg::MarkerArray::SharedPtr msg)
{
    marker_visible_ = false;

    for(const auto& marker : msg->markers)
    {

        if(marker.id == target_id_)
        {
            current_u_ = marker.u;
            current_v_ = marker.v;
            current_side_ = marker.side;

            marker_visible_ = true;
            break;
        }
    }
}

// odomCallback definition
void MarkerSubscriber::odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        odom_received_ = true;  // robot spawned and odometry available

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

// cameraInfoCallback definition: intrinsics of the camera actually in use
void MarkerSubscriber::cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    fx_ = msg->k[0];
    cx_ = msg->k[2];
    camera_info_received_ = true;
}

// fsm definition
void MarkerSubscriber::fsm(){

    switch(current_state_){

        case START:

            // wait until Gazebo is running and the robot publishes odometry,
            // otherwise the first commands are lost and the yaw is not valid
            // wait also for camera_info to be received
            if(!odom_received_ || !camera_info_received_)
            {
                break;
            }

            RCLCPP_INFO(
                this->get_logger(),
                "Robot started. Entering MOVE state.");

            current_state_ = MOVE;
        break;

        case MOVE:

                RCLCPP_INFO(
                this->get_logger(),
                "MOVE state, target_id: %d.", target_id_);

            // muovo il robot avanti dopo aver eseguito un istruzione (o in partenza)
            // robot fermo
            // deve ripartire (comando ripartire) 
            if(marker_visible_)
            {
                current_state_ = ALIGN_AND_APPROACH;
            }
            else
            {
                moveForward(forward_speed_);
            }

        break;

        case ALIGN_AND_APPROACH:
        {

            RCLCPP_INFO(
                this->get_logger(),
                "ALIGN_AND_APPROACH state, target_id: %d.", target_id_);

            // marker lost
            if(!marker_visible_)
            {
                current_state_ = MOVE;
                break;
            }

            // desired side scaled with the focal length: sd = sd_ref * fx / fx_ref
            // (in simulation fx == reference_fx, so sd = 166 px as before)
            const double scale = fx_ / reference_fx_;
            double desired_side = reference_side_ * scale;
            double side_tolerance = side_tolerance_;

            if(target_id_ == 4)
            {
                // diminuita distanza in px per garantire ultimo marker venga visto
                desired_side = 168.0 * scale;
                side_tolerance = 0.5;
            }

            double u_error = current_u_ - cx_;
            double side_error = desired_side - current_side_;

             if(std::abs(u_error) < u_tolerance_ && std::abs(side_error) < side_tolerance)
            {
                stopRobot();
                current_state_ = TAKE_PHOTO;
            }
            else
            {
                double omega = -k_omega_ * u_error;
                double v = k_v_ * side_error;
                twist(v, omega);
            }

        break;
        }

        case TAKE_PHOTO:
        {
            if (service_response_received_) {
                service_response_received_ = false;
                service_request_sent_ = false;
                current_state_ = EXECUTE;
            } else if (!service_request_sent_) {
                take_photo_request();
            }
            break;
        }
        
        case EXECUTE:

            RCLCPP_INFO(
                this->get_logger(),
                "EXECUTE state, target_id: %d.", target_id_);


            service_response_received_ = false;

            switch(target_id_)
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

            // increment target_id_ to move to the next marker
            target_id_++;  
                        
        break;

        case ROTATE:
        {

            RCLCPP_INFO(
                this->get_logger(),
                "ROTATE state, target_id: %d.", target_id_);

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

            if(std::abs(error) < yaw_tolerance_)
            {
                stopRobot();

                initialized = false;
                current_state_ = MOVE;
            }
            else
            {
                rotate(k_rot_ * error);
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

// function general twist definition
void MarkerSubscriber::twist(double v, double omega)
{
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = omega;
    cmd_vel_pub_->publish(cmd);
}

// function to stop robot definition
void MarkerSubscriber::stopRobot()
{
    geometry_msgs::msg::Twist cmd;
    cmd_vel_pub_->publish(cmd);
}

// Sends request to take_photo_service to take photo of and publish it modfied
void MarkerSubscriber::take_photo_request()
{
    if (!take_photo_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "Waiting for take-photo service...");
        return;
    }

    auto request = std::make_shared<TakePhoto::Request>();
    request->request = true;

    take_photo_client_->async_send_request(
        request,
        [this](rclcpp::Client<TakePhoto>::SharedFuture future) {
            auto response = future.get();
            service_response_success_ = response->success;
            if (response->success) {
                RCLCPP_INFO(
                    this->get_logger(), "Take-photo succeeded; detected marker ID: %d",
                    response->marker_id);
            } else {
                RCLCPP_WARN(this->get_logger(), "Take-photo service did not detect a marker.");
            }
            service_response_received_ = true;
        }
    );
    service_request_sent_ = true;
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MarkerSubscriber>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
