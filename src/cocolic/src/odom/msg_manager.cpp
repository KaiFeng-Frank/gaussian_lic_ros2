/*
 * Coco-LIC: Continuous-Time Tightly-Coupled LiDAR-Inertial-Camera Odometry using Non-Uniform B-spline
 * Copyright (C) 2023 Xiaolei Lang
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <odom/msg_manager.h>
#include <utils/parameter_struct.h>

#include <pcl/common/transforms.h>
#include <cstring>  // ROS2 port: std::memcpy for PointCloud2 field reads

namespace cocolic
{

  MsgManager::MsgManager(const YAML::Node &node, const std::string &config_path)
      : has_valid_msg_(true),
        t_offset_imu_(0),
        t_offset_camera_(0),
        cur_imu_timestamp_(-1),
        cur_pose_timestamp_(-1),
        use_image_(false),
        lidar_timestamp_end_(false),
        remove_wrong_time_imu_(false),
        if_normalized_(false),
        image_topic_("")
  {
    OdometryMode odom_mode = OdometryMode(node["odometry_mode"].as<int>());

    // ROS2 port: bag_path from yaml (was nh.param override then yaml fallback).
    bag_path_ = node["bag_path"].as<std::string>();

    /// imu topic
    std::string imu_yaml = node["imu_yaml"].as<std::string>();
    YAML::Node imu_node = YAML::LoadFile(config_path + imu_yaml);
    imu_topic_ = imu_node["imu_topic"].as<std::string>();
    // pose_topic_ = imu_node["pose_topic"].as<std::string>();
    // remove_wrong_time_imu_ = imu_node["remove_wrong_time_imu"].as<bool>();
    if_normalized_ = imu_node["if_normalized"].as<bool>();

    // double imu_frequency = node["imu_frequency"].as<double>();
    // double imu_period_s = 1. / imu_frequency;

    std::string cam_yaml = node["camera_yaml"].as<std::string>();
    YAML::Node cam_node = YAML::LoadFile(config_path + cam_yaml);
    img_time_offset_ = cam_node["img_time_offset"].as<double>();

    // add_extra_timeoffset_s_ =
    //     yaml::GetValue<double>(node, "add_extra_timeoffset_s", 0);
    // LOG(INFO) << "add_extra_timeoffset_s: " << add_extra_timeoffset_s_;
    // std::cout << "add_extra_timeoffset_s: " << add_extra_timeoffset_s_ << "\n";

    /// image topic
    if (odom_mode == OdometryMode::LICO)
      use_image_ = true;
    if (use_image_)
    {
      std::string cam_yaml = config_path + node["camera_yaml"].as<std::string>();
      YAML::Node cam_node = YAML::LoadFile(cam_yaml);
      image_topic_ = cam_node["image_topic"].as<std::string>();
      image_topic_compressed_ = std::string(image_topic_).append("/compressed");
      // ROS2 port: dropped debug image publisher (/vio/test_img).
    }
    image_max_timestamp_ = -1;

    /// lidar topic
    std::string lidar_yaml = node["lidar_yaml"].as<std::string>();
    YAML::Node lidar_node = YAML::LoadFile(config_path + lidar_yaml);
    num_lidars_ = lidar_node["num_lidars"].as<int>();
    lidar_timestamp_end_ = lidar_node["lidar_timestamp_end"].as<bool>();

    bool use_livox = false;
    bool use_vlp = false;
    for (int i = 0; i < num_lidars_; ++i)
    {
      std::string lidar_str = "lidar" + std::to_string(i);
      const auto &lidar_i = lidar_node[lidar_str];
      bool is_livox = lidar_i["is_livox"].as<bool>();
      if (is_livox)
      {
        lidar_types.push_back(LIVOX);
        use_livox = true;
      }
      else
      {
        lidar_types.push_back(VLP);
        use_vlp = true;
      }
      lidar_topics_.push_back(lidar_i["topic"].as<std::string>());
      EP_LktoI_.emplace_back();
      EP_LktoI_.back().Init(lidar_i["Extrinsics"]);
    }

    for (int k = 0; k < num_lidars_; ++k)
    {
      lidar_max_timestamps_.push_back(0);
      Eigen::Matrix4d T_Lk_to_L0 = Eigen::Matrix4d::Identity();
      if (k > 0)
      {
        T_Lk_to_L0.block<3, 3>(0, 0) =
            (EP_LktoI_[0].q.inverse() * EP_LktoI_[k].q).toRotationMatrix();
        T_Lk_to_L0.block<3, 1>(0, 3) =
            EP_LktoI_[0].q.inverse() * (EP_LktoI_[k].p - EP_LktoI_[0].p);

        // std::cout << "lidar " << k << "\n"
        //           << T_Lk_to_L0 << std::endl;
      }
      T_LktoL0_vec_.push_back(T_Lk_to_L0);
    }

    if (use_livox)
      livox_feature_extraction_ =
          std::make_shared<LivoxFeatureExtraction>(lidar_node);
    if (use_vlp)
    {
      // ROS2 port: velodyne path deferred (CBD_Building_01 is Livox-only).
      std::cerr << "[MsgManager] VLP lidar requested but velodyne feature "
                   "extraction is not ported yet (Livox-only build).\n";
    }

    LoadBag(node);
  }

  void MsgManager::LoadBag(const YAML::Node &node)
  {
    double bag_start = node["bag_start"].as<double>();
    double bag_durr = node["bag_durr"].as<double>();

    std::vector<std::string> topics;
    topics.push_back(imu_topic_); // imu
    if (use_image_)               // camera
    {
      topics.push_back(image_topic_);
      topics.push_back(image_topic_compressed_);
    }
    for (auto &v : lidar_topics_) // lidar
      topics.push_back(v);
    // topics.push_back(pose_topic_);

    // ROS2 port: rosbag::Bag/View → rosbag2_cpp::Reader. Auto-detect the storage
    // backend (sqlite3 / mcap) from the bag's metadata.yaml. A sqlite3 bag is a
    // DIRECTORY whose ".db3" extension lives on the inner file, not on bag_path_,
    // so sniffing the path extension is unreliable; leaving storage_id empty makes
    // rosbag2 read storage_identifier from metadata.yaml (works for both the
    // frontend_raw db3 dir and the offset_time_full mcap dir).
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path_;
    storage_options.storage_id = "";  // empty ⇒ detect from metadata.yaml
    reader_ = std::make_shared<rosbag2_cpp::Reader>();
    reader_->open(storage_options);

    // Topic filter (equivalent to rosbag::TopicQuery).
    rosbag2_storage::StorageFilter filter;
    filter.topics = topics;
    reader_->set_filter(filter);

    // Play window relative to bag begin; absolute begin captured lazily from the
    // first message's recv_timestamp in SpinBagOnce (avoids metadata epoch issues).
    bag_start_s_ = bag_start;
    bag_durr_s_ = bag_durr;

    std::cout << "\n🍺 LoadBag " << bag_path_ << " start at " << bag_start
              << " with duration " << bag_durr << ".\n";
  }

  void MsgManager::SpinBagOnce()
  {
    // ROS2 port: process exactly ONE handled message per call (matching the
    // ROS1 view_iterator++ semantics), skipping out-of-window messages.
    static rclcpp::Serialization<sensor_msgs::msg::Imu> imu_ser;
    static rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc_ser;
    static rclcpp::Serialization<sensor_msgs::msg::Image> img_ser;

    while (true)
    {
      if (!reader_->has_next())
      {
        has_valid_msg_ = false;
        return;
      }
      auto bag_msg = reader_->read_next();
      const int64_t t_ns = bag_msg->recv_timestamp;
      if (bag_first_ns_ < 0) bag_first_ns_ = t_ns;
      const double rel_s = (t_ns - bag_first_ns_) * 1e-9;
      if (rel_s < bag_start_s_) continue;  // before window
      if (bag_durr_s_ >= 0 && rel_s > bag_start_s_ + bag_durr_s_)
      {
        has_valid_msg_ = false;  // past window end
        return;
      }

      const std::string &msg_topic = bag_msg->topic_name;
      rclcpp::SerializedMessage ser(*bag_msg->serialized_data);

      if (msg_topic == imu_topic_)  // imu
      {
        auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
        imu_ser.deserialize_message(&ser, imu_msg.get());
        IMUMsgHandle(imu_msg);
        return;
      }
      auto it = std::find(lidar_topics_.begin(), lidar_topics_.end(), msg_topic);
      if (it != lidar_topics_.end())  // lidar
      {
        auto idx = std::distance(lidar_topics_.begin(), it);
        if (lidar_types[idx] == LIVOX)  //[solid-state lidar: Livox]
        {
          sensor_msgs::msg::PointCloud2 pc;
          pc_ser.deserialize_message(&ser, &pc);
          CustomMsgLite::ConstPtr lidar_msg = PointCloud2ToCustomMsg(pc);
          CheckLidarMsgTimestamp(t_ns * NS_TO_S, lidar_msg->timebase * NS_TO_S);
          LivoxMsgHandle(lidar_msg, idx);
        }
        else  // VLP deferred (Livox-only build)
        {
          std::cout << "[SpinBagOnce] VLP lidar not ported — skipping.\n";
        }
        return;
      }
      if (use_image_ &&
          (msg_topic == image_topic_ || msg_topic == image_topic_compressed_))  // camera
      {
        auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
        img_ser.deserialize_message(&ser, image_msg.get());
        ImageMsgHandle(image_msg);
        return;
      }
      // topic not handled (filtered out otherwise) — keep scanning
    }
  }

  // ROS2 port: build the upstream CustomMsg surface from the offset_time_full
  // PointCloud2 (20-byte stride). Field offsets resolved by name; per-point time
  // (offset_time, ns) carried through as CustomPointLite::offset_time so the
  // feature extraction's continuous-time deskew stays bit-faithful.
  CustomMsgLite::Ptr MsgManager::PointCloud2ToCustomMsg(
      const sensor_msgs::msg::PointCloud2 &pc)
  {
    auto out = std::make_shared<CustomMsgLite>();
    out->timebase = int64_t(pc.header.stamp.sec) * 1000000000LL +
                    int64_t(pc.header.stamp.nanosec);
    const size_t n = size_t(pc.width) * pc.height;
    out->point_num = uint32_t(n);
    out->points.resize(n);

    int off_x = -1, off_y = -1, off_z = -1, off_int = -1, off_tag = -1,
        off_line = -1, off_ot = -1;
    for (const auto &f : pc.fields)
    {
      if (f.name == "x") off_x = f.offset;
      else if (f.name == "y") off_y = f.offset;
      else if (f.name == "z") off_z = f.offset;
      else if (f.name == "intensity") off_int = f.offset;
      else if (f.name == "tag") off_tag = f.offset;
      else if (f.name == "line") off_line = f.offset;
      else if (f.name == "offset_time") off_ot = f.offset;
    }

    const uint8_t *base = pc.data.data();
    for (size_t i = 0; i < n; ++i)
    {
      const uint8_t *p = base + i * pc.point_step;
      CustomPointLite &cp = out->points[i];
      std::memcpy(&cp.x, p + off_x, 4);
      std::memcpy(&cp.y, p + off_y, 4);
      std::memcpy(&cp.z, p + off_z, 4);
      cp.reflectivity = (off_int >= 0) ? *(p + off_int) : 0;
      cp.tag = (off_tag >= 0) ? *(p + off_tag) : 0;
      cp.line = (off_line >= 0) ? *(p + off_line) : 0;
      if (off_ot >= 0) std::memcpy(&cp.offset_time, p + off_ot, 4);
      else cp.offset_time = 0;
    }
    return out;
  }

  void MsgManager::LogInfo() const
  {
    int m_size[3] = {0, 0, 0};
    m_size[0] = imu_buf_.size();
    m_size[1] = lidar_buf_.size();
    // if (use_image_) m_size[2] = feature_tracker_node_->NumImageMsg();
    // LOG(INFO) << "imu/lidar/image msg left: " << m_size[0] << "/" << m_size[1]
    //           << "/" << m_size[2];
  }

  void MsgManager::RemoveBeginData(int64_t start_time, // not used
                                   int64_t relative_start_time)
  { // 0
    for (auto iter = lidar_buf_.begin(); iter != lidar_buf_.end();)
    {
      if (iter->timestamp < relative_start_time)
      {
        if (iter->max_timestamp <= relative_start_time)
        { // [1]
          iter = lidar_buf_.erase(iter);
          continue;
        }
        else
        { // [2]
          // int64_t t_aft = relative_start_time + 1e-3;  //1e-3
          int64_t t_aft = relative_start_time;
          LiDARCloudData scan_bef, scan_aft;
          scan_aft.timestamp = t_aft;
          scan_aft.max_timestamp = iter->max_timestamp;
          pcl::FilterCloudByTimestamp(iter->raw_cloud, t_aft,
                                      scan_bef.raw_cloud,
                                      scan_aft.raw_cloud);
          pcl::FilterCloudByTimestamp(iter->surf_cloud, t_aft,
                                      scan_bef.surf_cloud,
                                      scan_aft.surf_cloud);
          pcl::FilterCloudByTimestamp(iter->corner_cloud, t_aft,
                                      scan_bef.corner_cloud,
                                      scan_aft.corner_cloud);

          iter->timestamp = t_aft;
          *iter->raw_cloud = *scan_aft.raw_cloud;
          *iter->surf_cloud = *scan_aft.surf_cloud;
          *iter->corner_cloud = *scan_aft.corner_cloud;
        }
      }

      iter++;
    }

    if (use_image_)
    {
      for (auto iter = image_buf_.begin(); iter != image_buf_.end();)
      {
        if (iter->timestamp < relative_start_time)
        {
          iter = image_buf_.erase(iter); //
          continue;
        }
        iter++;
      }
    }
  }

  bool MsgManager::HasEnvMsg() const
  {
    int env_msg = lidar_buf_.size();
    // if (cur_imu_timestamp_ < 0 && env_msg > 100)
    //   LOG(WARNING) << "No IMU data. CHECK imu topic" << imu_topic_;

    return env_msg > 0;
  }

  bool MsgManager::CheckMsgIsReady(double traj_max, double start_time,
                                   double knot_dt, bool in_scan_unit) const
  {
    double t_imu_wrt_start = cur_imu_timestamp_ - start_time;

    //
    if (t_imu_wrt_start < traj_max)
    {
      return false;
    }

    //
    int64_t t_front_lidar = -1;
    // Count how many unique lidar streams
    std::vector<int> unique_lidar_ids;
    for (const auto &data : lidar_buf_)
    {
      if (std::find(unique_lidar_ids.begin(), unique_lidar_ids.end(),
                    data.lidar_id) != unique_lidar_ids.end())
        continue;
      unique_lidar_ids.push_back(data.lidar_id);

      //
      t_front_lidar = std::max(t_front_lidar, data.max_timestamp);
    }

    //
    if ((int)unique_lidar_ids.size() != num_lidars_)
      return false;

    //
    int64_t t_back_lidar = lidar_max_timestamps_[0];
    for (auto t : lidar_max_timestamps_)
    {
      t_back_lidar = std::min(t_back_lidar, t);
    }

    //
    if (in_scan_unit)
    {
      //
      if (t_front_lidar > t_imu_wrt_start)
        return false;
    }
    else
    {
      //
      if (t_back_lidar < traj_max)
        return false;
    }

    return true;
  }

  bool MsgManager::AddImageToMsg(NextMsgs &msgs, const ImageData &image,
                                 int64_t traj_max)
  {
    if (image.timestamp >= traj_max)
      return false;
    msgs.if_have_image = true; // important!
    msgs.image_timestamp = image.timestamp;
    msgs.image = image.image;
    // msgs.image = image.image.clone();
    return true;
  }

  bool MsgManager::AddToMsg(NextMsgs &msgs, std::deque<LiDARCloudData>::iterator scan,
                            int64_t traj_max)
  {
    bool add_entire_scan = false;
    // if (scan->timestamp > traj_max) return add_entire_scan;

    if (scan->max_timestamp < traj_max)
    { //
      *msgs.lidar_raw_cloud += (*scan->raw_cloud);
      *msgs.lidar_surf_cloud += (*scan->surf_cloud);
      *msgs.lidar_corner_cloud += (*scan->corner_cloud);

      //
      if (msgs.scan_num == 0)
      {
        // first scan
        msgs.lidar_timestamp = scan->timestamp;
        msgs.lidar_max_timestamp = scan->max_timestamp;
      }
      else
      {
        msgs.lidar_timestamp =
            std::min(msgs.lidar_timestamp, scan->timestamp);
        msgs.lidar_max_timestamp =
            std::max(msgs.lidar_max_timestamp, scan->max_timestamp);
      }

      add_entire_scan = true;
    }
    else
    { //
      LiDARCloudData scan_bef, scan_aft;
      pcl::FilterCloudByTimestamp(scan->raw_cloud, traj_max, scan_bef.raw_cloud,
                                  scan_aft.raw_cloud);
      pcl::FilterCloudByTimestamp(scan->surf_cloud, traj_max, scan_bef.surf_cloud,
                                  scan_aft.surf_cloud);
      pcl::FilterCloudByTimestamp(scan->corner_cloud, traj_max,
                                  scan_bef.corner_cloud, scan_aft.corner_cloud);
      //
      scan_bef.timestamp = scan->timestamp;
      scan_bef.max_timestamp = traj_max - 1e-9 * S_TO_NS;
      scan_aft.timestamp = traj_max;
      scan_aft.max_timestamp = scan->max_timestamp;

      //
      scan->timestamp = traj_max;
      // *scan.max_timestamp = ； //
      *scan->raw_cloud = *scan_aft.raw_cloud;
      *scan->surf_cloud = *scan_aft.surf_cloud;
      *scan->corner_cloud = *scan_aft.corner_cloud;

      *msgs.lidar_raw_cloud += (*scan_bef.raw_cloud);
      *msgs.lidar_surf_cloud += (*scan_bef.surf_cloud);
      *msgs.lidar_corner_cloud += (*scan_bef.corner_cloud);

      //
      if (msgs.scan_num == 0)
      {
        // first scan
        msgs.lidar_timestamp = scan_bef.timestamp;
        msgs.lidar_max_timestamp = scan_bef.max_timestamp;
      }
      else
      {
        msgs.lidar_timestamp =
            std::min(msgs.lidar_timestamp, scan_bef.timestamp);
        msgs.lidar_max_timestamp =
            std::max(msgs.lidar_max_timestamp, scan_bef.max_timestamp);
      }

      add_entire_scan = false;
    }

    //
    msgs.scan_num++;

    return add_entire_scan;
  }

  ///
  bool MsgManager::GetMsgs(NextMsgs &msgs, int64_t traj_last_max, int64_t traj_max, int64_t start_time)
  {
    msgs.Clear();

    if (imu_buf_.empty() || lidar_buf_.empty())
    {
      return false;
    }
    if (cur_imu_timestamp_ - start_time < traj_max)
    {
      return false;
    }

    /// 1
    //
    std::vector<int> unique_lidar_ids;
    for (const auto &data : lidar_buf_)
    {
      if (std::find(unique_lidar_ids.begin(), unique_lidar_ids.end(),
                    data.lidar_id) != unique_lidar_ids.end())
        continue;
      unique_lidar_ids.push_back(data.lidar_id);
    }
    if (unique_lidar_ids.size() != num_lidars_)
    {
      return false;
    }
    //
    for (auto t : lidar_max_timestamps_)
    {
      if (t < traj_max)
      {
        return false;
      }
    }
    //
    if (use_image_)
    {
      if (image_max_timestamp_ < traj_max)
      {
        return false;
      }
    }

    /// 2
    for (auto it = lidar_buf_.begin(); it != lidar_buf_.end();)
    {
      if (it->timestamp >= traj_max)
      {
        ++it;
        continue;
      }
      bool add_entire_scan = AddToMsg(msgs, it, traj_max);
      if (add_entire_scan)
      {
        it = lidar_buf_.erase(it); //
      }
      else
      {
        ++it; //
      }
    }
    // LOG(INFO) << "[msgs_scan_num] " << msgs.scan_num;

    /// 3
    if (use_image_)
    {
      ///
      int img_idx = INT_MAX;
      for (int i = 0; i < image_buf_.size(); i++)
      {
        if (image_buf_[i].timestamp >= traj_last_max &&
            image_buf_[i].timestamp < traj_max)
        {
          img_idx = i;
        }
        if (image_buf_[i].timestamp >= traj_max)
        {
          break;
        }
      }

      ///
      // int img_idx = INT_MAX;
      // for (int i = 0; i < image_buf_.size(); i++)
      // {
      //   if (image_buf_[i].timestamp >= traj_last_max &&
      //       image_buf_[i].timestamp < traj_max)
      //   {
      //     img_idx = i;
      //     break;
      //   }
      // }

      if (img_idx != INT_MAX)
      {
        AddImageToMsg(msgs, image_buf_[img_idx], traj_max);
        // image_buf_.erase(image_buf_.begin() + img_idx);
      }
      else
      {
        msgs.if_have_image = false;
        // std::cout << "[GetMsgs does not get a image]\n";
        // std::getchar();
      }
    }

    return true;
  }

  void MsgManager::IMUMsgHandle(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
  {
    int64_t t_last = cur_imu_timestamp_;
    // ROS2 port: header.stamp.toSec()*S_TO_NS → ns from sec/nanosec.
    cur_imu_timestamp_ = int64_t(imu_msg->header.stamp.sec) * 1000000000LL +
                         int64_t(imu_msg->header.stamp.nanosec);

    IMUData data;
    IMUMsgToIMUData(imu_msg, data);

    /// problem
    // data.timestamp -= add_extra_timeoffset_s_;

    // for trajectory_manager
    imu_buf_.emplace_back(data);
  }

  // ROS2 port: VelodyneMsgHandle / VelodyneMsgHandleNoFeature deferred
  // (CBD_Building_01 is Livox-only; velodyne_feature_extraction not yet ported).

  void MsgManager::LivoxMsgHandle(
      const CustomMsgLite::ConstPtr &livox_msg, int lidar_id)
  {
    RTPointCloud::Ptr livox_raw_cloud(new RTPointCloud);
    //
    // livox_feature_extraction_->ParsePointCloud(livox_msg, livox_raw_cloud);
    // livox_feature_extraction_->ParsePointCloudNoFeature(livox_msg, livox_raw_cloud);
    livox_feature_extraction_->ParsePointCloudR3LIVE(livox_msg, livox_raw_cloud);

    LiDARCloudData data;
    data.lidar_id = lidar_id;
    data.timestamp = livox_msg->timebase;  // ROS2 port: header stamp (ns) = timebase
    data.raw_cloud = livox_raw_cloud;
    data.surf_cloud = livox_feature_extraction_->GetSurfaceFeature();
    data.corner_cloud = livox_feature_extraction_->GetCornerFeature();
    lidar_buf_.push_back(data);

    if (lidar_id != 0)
    {
      pcl::transformPointCloud(*data.raw_cloud, *data.raw_cloud,
                               T_LktoL0_vec_[lidar_id]);
      pcl::transformPointCloud(*data.surf_cloud, *data.surf_cloud,
                               T_LktoL0_vec_[lidar_id]);
      pcl::transformPointCloud(*data.corner_cloud, *data.corner_cloud,
                               T_LktoL0_vec_[lidar_id]);
    }
  }

  void MsgManager::ImageMsgHandle(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    // ROS2 port: dropped debug image publish.
    cv_bridge::CvImagePtr cvImgPtr;
    cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    if (cvImgPtr->image.empty())
    {
      std::cout << RED << "[ImageMsgHandle get an empty img]" << RESET << std::endl;
      return;
    }

    image_buf_.emplace_back();
    // ROS2 port: header.stamp.toNSec() → ns from sec/nanosec.
    image_buf_.back().timestamp =
        (int64_t(msg->header.stamp.sec) * 1000000000LL +
         int64_t(msg->header.stamp.nanosec)) +
        int64_t(img_time_offset_ * S_TO_NS);
    image_buf_.back().image = cvImgPtr->image;
    nerf_time_.push_back(image_buf_.back().timestamp);

    if (image_buf_.back().image.cols == 640 || image_buf_.back().image.cols == 1280)
    {
      cv::resize(image_buf_.back().image, image_buf_.back().image, cv::Size(640, 512), 0, 0, cv::INTER_LINEAR);
    }
  }

} // namespace cocolic
