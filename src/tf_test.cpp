#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <mutex>
#include <memory>
#include <math.h>  
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
// #include <tf2_geometry_msgs.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include "geometry.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>


// using namespace std::chrono;
using namespace std::chrono_literals;


class TFTest : public rclcpp::Node {
public:
	TFTest() : Node("TFTest") {
		
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    geometry_msgs::msg::TransformStamped drone_tf;

    while(true) {

        try {

            drone_tf = tf_buffer_->lookupTransform("drone", "world", tf2::TimePointZero);

            RCLCPP_INFO(this->get_logger(), "Found transform drone->world");
            break;

        } catch(tf2::TransformException & ex) {

            RCLCPP_INFO(this->get_logger(), "Could not get transform drone->world, trying again...");

        }

    }

		input_point_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/orig_point", 10,
            std::bind(&TFTest::transform_points_to_world, this, std::placeholders::_1));


		output_point_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/out_point", 10);
    original_point_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/orig_point", 10);


		auto timer_callback = [this]() -> void {
      counter += 1;
			geometry_msgs::msg::PointStamped msg;
      msg.header.frame_id = "drone";
      msg.header.stamp = this->get_clock()->now();
      msg.point.x = sin(float(counter/10.));
      msg.point.y = cos(float(counter/10.));
      msg.point.z = 0;
      original_point_pub->publish(msg);
		};


		timer_ = this->create_wall_timer(30ms, timer_callback);


	}


	~TFTest() {
		RCLCPP_INFO(this->get_logger(),  "Shutting down..");
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}



private:

  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_tf;

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr original_point_pub;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr output_point_pub;

	rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr input_point_sub;

  long counter = 0;

  void transform_points_to_world(const geometry_msgs::msg::PointStamped::SharedPtr msg);

};




void TFTest::transform_points_to_world(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(),  "Got point");

  geometry_msgs::msg::TransformStamped t;

  try {
    if (tf_buffer_->canTransform("world", "drone", tf2::TimePointZero))
    {
      
      t = tf_buffer_->lookupTransform(
      "world",
      "drone",
      tf2::TimePointZero);

      std::cout << "T is:" << std::endl;
      
      // RCLCPP_INFO(this->get_logger(),  "Transform is: \n T \t %f \t %f \t %f \n R: \t %f \t %f \t %f \t %f \n", 
      //   t.transform.translation.x, t.transform.translation.y, t.transform.translation.z,
      //   t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
      
    }
    else {
      std::cout << "Cannot transform" << std::endl;
      return;
    }
    
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      this->get_logger(), "Could not transform: %s", ex.what());
    return;
  }


	// make transform drone->world

	vector_t t_xyz;
	t_xyz(0) = t.transform.translation.x;
	t_xyz(1) = t.transform.translation.y;
	t_xyz(2) = t.transform.translation.z;

  quat_t t_rot;
	t_rot(0) = t.transform.rotation.x;
	t_rot(1) = t.transform.rotation.y;
	t_rot(2) = t.transform.rotation.z;
  t_rot(3) = t.transform.rotation.w;

  rotation_matrix_t test_R = quatToMat(t_rot);

  RCLCPP_INFO(this->get_logger(),  "R is: \n  %f \t %f \t %f \n %f \t %f \t %f \n %f \t %f \t %f \n", 
        test_R(0,0),test_R(0,1),test_R(0,2),
        test_R(1,0),test_R(1,1),test_R(1,2),
        test_R(2,0),test_R(2,1),test_R(2,2));

  quat_t quat_test = matToQuat(test_R);

  RCLCPP_INFO(this->get_logger(),  "Q is: \n  %f \t %f \t %f \t %f \n", 
        quat_test(0),quat_test(1),quat_test(2),quat_test(3));



  transform_t d_to_w_t = getTransformMatrix(t_xyz, t_rot);


  // transform point from local to world frame

  quat_t point_local; // only quat to get 4 vector
	point_local(0) = msg->point.x;
	point_local(1) = msg->point.y;
	point_local(2) = msg->point.z;
  point_local(3) = 1;

  RCLCPP_INFO(this->get_logger(),  "Homog T is: \n  %f \t %f \t %f \t %f \n %f \t %f \t %f \t %f \n %f \t %f \t %f \t %f \n %f \t %f \t %f \t %f \n", 
        d_to_w_t(0,0),d_to_w_t(0,1),d_to_w_t(0,2),d_to_w_t(0,3),
        d_to_w_t(1,0),d_to_w_t(1,1),d_to_w_t(1,2),d_to_w_t(1,3),
        d_to_w_t(2,0),d_to_w_t(2,1),d_to_w_t(2,2),d_to_w_t(2,3),
        d_to_w_t(3,0),d_to_w_t(3,1),d_to_w_t(3,2),d_to_w_t(3,3));

  RCLCPP_INFO(this->get_logger(),  "Point is: \n  %f \n %f \n %f \n %f \n", 
        point_local(0), point_local(1), point_local(2), point_local(3));

	quat_t point_world = d_to_w_t * point_local; // sort of works

  RCLCPP_INFO(this->get_logger(),  "Point world coordinates are: \n X: %f \t Y: %f \t Z: %f \n ", 
        point_world(0), point_world(1), point_world(2));

	geometry_msgs::msg::PointStamped msg_out;
	msg_out.header.frame_id = "world";
  msg_out.header.stamp = this->get_clock()->now();
	msg_out.point.x = point_world(0);
	msg_out.point.y = point_world(1);
	msg_out.point.z = point_world(2);
	output_point_pub->publish(msg_out);

}


int main(int argc, char ** argv)
{
	std::cout << "Starting..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TFTest>());

	rclcpp::shutdown();
	return 0;
}
