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

    input_pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/iwr6843_pcl", 10,
            std::bind(&TFTest::transform_pointcloud_to_world, this, std::placeholders::_1));


		output_point_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/out_point", 10);
    original_point_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/orig_point", 10);
    output_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/world_pcl", 10);

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
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_pointcloud_pub;

	rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr input_point_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_pointcloud_sub;

  long counter = 0;

  void transform_points_to_world(const geometry_msgs::msg::PointStamped::SharedPtr msg);

  void transform_pointcloud_to_world(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void read_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, Eigen::MatrixXf * data_holder);

  void create_pointcloud_msg(int pointcloud_size, Eigen::MatrixXf * data, auto * pcl_msg);

};


void TFTest::create_pointcloud_msg(int pcl_size, Eigen::MatrixXf * data, auto * pcl_msg) {

  // create PointCloud2 msg
	//https://github.com/ros-drivers/velodyne/blob/master/velodyne_laserscan/tests/system.cpp
	auto pcl2_msg = sensor_msgs::msg::PointCloud2();
	pcl_msg->header = std_msgs::msg::Header();
	pcl_msg->header.stamp = this->now();
	std::string frameID = "world";
	pcl_msg->header.frame_id = frameID;
	pcl_msg->fields.resize(3);
	pcl_msg->fields[0].name = 'x';
	pcl_msg->fields[0].offset = 0;
	pcl_msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
	pcl_msg->fields[0].count = 1;
	pcl_msg->fields[1].name = 'y';
	pcl_msg->fields[1].offset = 4;
	pcl_msg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
	pcl_msg->fields[1].count = 1;
	pcl_msg->fields[2].name = 'z';
	pcl_msg->fields[2].offset = 8;
	pcl_msg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
	pcl_msg->fields[2].count = 1;
	const uint32_t POINT_STEP = 12;
	if(pcl_size > 0){
		pcl_msg->data.resize(std::max((size_t)1, (size_t)pcl_size) * POINT_STEP, 0x00);
	}
	pcl_msg->point_step = POINT_STEP; // size (bytes) of 1 point (float32 * dimensions (3 when xyz))
	pcl_msg->row_step = pcl_msg->data.size();//pcl_msg->point_step * pcl_msg->width; // only 1 row because unordered
	pcl_msg->height = 1;  // because unordered cloud
	pcl_msg->width = pcl_msg->row_step / POINT_STEP; // number of points in cloud
	pcl_msg->is_dense = false; // there may be invalid points

	// fill PointCloud2 msg data
	if(pcl_size > 0){
		uint8_t *ptr = pcl_msg->data.data();
		for (size_t i = 0; i < pcl_size; i++)
		{
			*(reinterpret_cast<float*>(ptr + 0)) = data->coeffRef(0,i);
			*(reinterpret_cast<float*>(ptr + 4)) = data->coeffRef(1,i);
			*(reinterpret_cast<float*>(ptr + 8)) = data->coeffRef(2,i);
			ptr += POINT_STEP;
		}
	}

}


void TFTest::read_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, Eigen::MatrixXf * data_holder) {

  // read PointCloud2 msg data
  int pcl_size = msg->width;
  uint8_t *ptr = msg->data.data();
  const uint32_t POINT_STEP = 12;

  // Eigen::MatrixXf local_points(4,pcl_size);

  for (size_t i = 0; i < pcl_size; i++) {

      // // filter points based on diagonal distance
      // if(sqrt( pow(point(0),2) + pow(point(1),2) + pow(point(2),2) ) < 0.25 ) {
      //     continue;
      // }

      data_holder->coeffRef(0,i) = *(reinterpret_cast<float*>(ptr + 0));
      data_holder->coeffRef(1,i) = *(reinterpret_cast<float*>(ptr + 4));
      data_holder->coeffRef(2,i) = *(reinterpret_cast<float*>(ptr + 8));
      data_holder->coeffRef(3,i) = 1.0;

      ptr += POINT_STEP;

  }

}


void TFTest::transform_pointcloud_to_world(const sensor_msgs::msg::PointCloud2::SharedPtr msg){

  geometry_msgs::msg::TransformStamped t;

  try {
    if (tf_buffer_->canTransform("world", "drone", tf2::TimePointZero))
    {
      t = tf_buffer_->lookupTransform("world","drone",tf2::TimePointZero);
    }
    else {
      RCLCPP_INFO(
      this->get_logger(), "Can not transform");
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

  homog_transform_t world_to_drone = getTransformMatrix(t_xyz, t_rot);

  int pcl_size = msg->width;

  Eigen::MatrixXf local_points(4,pcl_size);

  TFTest::read_pointcloud(msg, &local_points);


  Eigen::MatrixXf world_points(4,pcl_size);

  world_points = world_to_drone * local_points;

  

  // // create PointCloud2 msg
	// //https://github.com/ros-drivers/velodyne/blob/master/velodyne_laserscan/tests/system.cpp
	// auto pcl2_msg = sensor_msgs::msg::PointCloud2();
	// pcl2_msg.header = std_msgs::msg::Header();
	// pcl2_msg.header.stamp = this->now();
	// std::string frameID = "world";
	// pcl2_msg.header.frame_id = frameID;
	// pcl2_msg.fields.resize(3);
	// pcl2_msg.fields[0].name = 'x';
	// pcl2_msg.fields[0].offset = 0;
	// pcl2_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
	// pcl2_msg.fields[0].count = 1;
	// pcl2_msg.fields[1].name = 'y';
	// pcl2_msg.fields[1].offset = 4;
	// pcl2_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
	// pcl2_msg.fields[1].count = 1;
	// pcl2_msg.fields[2].name = 'z';
	// pcl2_msg.fields[2].offset = 8;
	// pcl2_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
	// pcl2_msg.fields[2].count = 1;
	// const uint32_t POINT_STEP = 12;
	// if(pcl_size > 0){
	// 	pcl2_msg.data.resize(std::max((size_t)1, (size_t)pcl_size) * POINT_STEP, 0x00);
	// }
	// pcl2_msg.point_step = POINT_STEP; // size (bytes) of 1 point (float32 * dimensions (3 when xyz))
	// pcl2_msg.row_step = pcl2_msg.data.size();//pcl2_msg.point_step * pcl2_msg.width; // only 1 row because unordered
	// pcl2_msg.height = 1;  // because unordered cloud
	// pcl2_msg.width = pcl2_msg.row_step / POINT_STEP; // number of points in cloud
	// pcl2_msg.is_dense = false; // there may be invalid points

	// // fill PointCloud2 msg data
	// if(pcl_size > 0){
	// 	uint8_t *ptr = pcl2_msg.data.data();
	// 	for (size_t i = 0; i < pcl_size; i++)
	// 	{
	// 		*(reinterpret_cast<float*>(ptr + 0)) = world_points(0,i);
	// 		*(reinterpret_cast<float*>(ptr + 4)) = world_points(1,i);
	// 		*(reinterpret_cast<float*>(ptr + 8)) = world_points(2,i);
	// 		ptr += POINT_STEP;
	// 	}
	// }
	// publish PointCloud2 msg

  auto pcl_msg = sensor_msgs::msg::PointCloud2();

  TFTest::create_pointcloud_msg(pcl_size, &world_points, &pcl_msg);

	output_pointcloud_pub->publish(pcl_msg);  

}



void TFTest::transform_points_to_world(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped t;

  try {
    if (tf_buffer_->canTransform("world", "drone", tf2::TimePointZero))
    {
      t = tf_buffer_->lookupTransform("world","drone",tf2::TimePointZero);
    }
    else {
      RCLCPP_INFO(
      this->get_logger(), "Can not transform");
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

  homog_transform_t world_to_drone = getTransformMatrix(t_xyz, t_rot);

  // transform point from local to world frame

  homog_point_t point_local; 
	point_local(0) = msg->point.x;
	point_local(1) = msg->point.y;
	point_local(2) = msg->point.z;
  point_local(3) = 1;

  // RCLCPP_INFO(this->get_logger(),  "Homog T is: \n  %f \t %f \t %f \t %f \n %f \t %f \t %f \t %f \n %f \t %f \t %f \t %f \n %f \t %f \t %f \t %f \n", 
  //       world_to_drone(0,0),world_to_drone(0,1),world_to_drone(0,2),world_to_drone(0,3),
  //       world_to_drone(1,0),world_to_drone(1,1),world_to_drone(1,2),world_to_drone(1,3),
  //       world_to_drone(2,0),world_to_drone(2,1),world_to_drone(2,2),world_to_drone(2,3),
  //       world_to_drone(3,0),world_to_drone(3,1),world_to_drone(3,2),world_to_drone(3,3));

  // RCLCPP_INFO(this->get_logger(),  "Point is: \n  %f \n %f \n %f \n %f \n", 
  //       point_local(0), point_local(1), point_local(2), point_local(3));

	homog_point_t point_world = world_to_drone * point_local; // works

  // RCLCPP_INFO(this->get_logger(),  "Point world coordinates are: \n X: %f \t Y: %f \t Z: %f \n ", 
  //       point_world(0), point_world(1), point_world(2));

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
