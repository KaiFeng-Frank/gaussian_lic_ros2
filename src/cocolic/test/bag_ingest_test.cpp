// Port S5 ingest primitive test: verify rosbag2_cpp::Reader + CDR deserialization
// can read the CBD_Building_01 frontend_raw db3 (the ROS2-native bag) and extract
// /imu (sensor_msgs/Imu) + /livox/lidar (PointCloud2, Livox fields) — the data
// the ported msg_manager will feed into the Coco-LIC pipeline. Standalone: no
// estimator yet, just de-risking the ingest path before the full RunBag port.
#include <cstdio>
#include <string>
#include <memory>

#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

int main(int argc, char ** argv)
{
  const std::string bag = (argc > 1)
    ? argv[1]
    : "/home/frank/data/fast_livo/CBD_Building_01_frontend_raw";

  rosbag2_cpp::Reader reader;
  reader.open(bag);

  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_ser;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc_ser;

  size_t n_imu = 0, n_lidar = 0;
  double imu_t0 = 0, imu_t1 = 0, lidar_t0 = 0, lidar_t1 = 0;
  size_t lidar_pts_first = 0;
  std::string lidar_fields;

  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    const std::string & topic = bag_msg->topic_name;
    rclcpp::SerializedMessage ser(*bag_msg->serialized_data);

    if (topic == "/imu") {
      sensor_msgs::msg::Imu m;
      imu_ser.deserialize_message(&ser, &m);
      double t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9;
      if (n_imu == 0) imu_t0 = t;
      imu_t1 = t;
      ++n_imu;
    } else if (topic == "/livox/lidar") {
      sensor_msgs::msg::PointCloud2 m;
      pc_ser.deserialize_message(&ser, &m);
      double t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9;
      if (n_lidar == 0) {
        lidar_t0 = t;
        lidar_pts_first = m.width * m.height;
        for (const auto & f : m.fields) { lidar_fields += f.name; lidar_fields += " "; }
      }
      lidar_t1 = t;
      ++n_lidar;
    }
  }

  std::printf("[bag_ingest_test] bag=%s\n", bag.c_str());
  std::printf("  IMU:   %zu msgs, t=[%.3f .. %.3f] span=%.2fs\n",
              n_imu, imu_t0, imu_t1, imu_t1 - imu_t0);
  std::printf("  LiDAR: %zu msgs, t=[%.3f .. %.3f] span=%.2fs, first_pts=%zu fields=[ %s]\n",
              n_lidar, lidar_t0, lidar_t1, lidar_t1 - lidar_t0, lidar_pts_first,
              lidar_fields.c_str());
  std::printf("  INGEST PRIMITIVE %s\n",
              (n_imu > 1000 && n_lidar > 100) ? "OK" : "FAILED");
  return (n_imu > 1000 && n_lidar > 100) ? 0 : 1;
}
