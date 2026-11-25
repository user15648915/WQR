#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <algorithm>

class StepDetectorNode : public rclcpp::Node {
public:
    StepDetectorNode() : Node("step_detector_node") {
        pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/detected_obstacle", 10);
        sub1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points", rclcpp::SensorDataQoS(),
            std::bind(&StepDetectorNode::cloud_cb, this, std::placeholders::_1));
        sub2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/points", rclcpp::SensorDataQoS(),
            std::bind(&StepDetectorNode::cloud_cb, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "3-Sector PCL Vision Node active (listening on /camera/points & /camera/depth/color/points)");
    }

private:
    void cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        float floor_y = 0.34f;
        for (const auto& pt : cloud->points) {
            if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) continue;
            if (pt.z > 0.20f && pt.z < 1.80f && std::abs(pt.x) < 0.35f) {
                if (pt.y > floor_y && pt.y < 0.48f) floor_y = pt.y;
            }
        }

        float min_z_center = 99.0f;
        float min_z_left   = 99.0f;
        float min_z_right  = 99.0f;
        int center_pts = 0;

        for (const auto& pt : cloud->points) {
            if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) continue;
            if (pt.z < 0.12f || pt.z > 2.50f) continue;

            float h = floor_y - pt.y;
            if (h > 0.035f && h < 1.80f) {
                if (std::abs(pt.x) <= 0.25f) {
                    center_pts++;
                    if (pt.z < min_z_center) min_z_center = pt.z;
                } else if (pt.x < -0.25f && pt.x > -0.85f && h > 0.14f) {
                    if (pt.z < min_z_left) min_z_left = pt.z;
                } else if (pt.x > 0.25f && pt.x < 0.85f && h > 0.14f) {
                    if (pt.z < min_z_right) min_z_right = pt.z;
                }
            }
        }

        float nearest_obj_height = 0.0f;
        if (center_pts > 10 && min_z_center < 2.4f) {
            for (const auto& pt : cloud->points) {
                if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) continue;
                if (std::abs(pt.x) <= 0.25f && pt.z >= min_z_center && pt.z <= (min_z_center + 0.22f)) {
                    float h = floor_y - pt.y;
                    if (h > nearest_obj_height && h < 1.80f) nearest_obj_height = h;
                }
            }
        } else {
            min_z_center = 99.0f;
        }

        std_msgs::msg::Float32MultiArray out;
        out.data = {min_z_center, nearest_obj_height, min_z_left, min_z_right};
        pub_->publish(out);
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub1_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub2_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StepDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
