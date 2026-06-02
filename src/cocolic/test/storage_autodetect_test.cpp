// Verify the LoadBag fix: rosbag2_cpp::Reader with StorageOptions{uri, ""}
// (empty storage_id) auto-detects the backend from metadata.yaml for BOTH a
// sqlite3 bag directory (.db3 inner file) and an mcap bag directory. Codex
// review flagged extension-sniffing as broken for sqlite3 dirs; this proves the
// empty-storage_id approach (matching the convenience open(uri) overload) works.
#include <cstdio>
#include <string>
#include <memory>

#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/storage_filter.hpp>

static int probe(const std::string & uri) {
  rosbag2_storage::StorageOptions so;
  so.uri = uri;
  so.storage_id = "";  // empty ⇒ detect from metadata.yaml
  rosbag2_cpp::Reader reader;
  try {
    reader.open(so);
  } catch (const std::exception & e) {
    std::printf("  OPEN FAILED uri=%s : %s\n", uri.c_str(), e.what());
    return 1;
  }
  rosbag2_storage::StorageFilter filter;
  filter.topics = {"/imu", "/livox/lidar"};
  reader.set_filter(filter);

  const auto & meta = reader.get_metadata();
  size_t n_imu = 0, n_lidar = 0;
  while (reader.has_next()) {
    auto m = reader.read_next();
    if (m->topic_name == "/imu") ++n_imu;
    else if (m->topic_name == "/livox/lidar") ++n_lidar;
    if (n_imu + n_lidar >= 2000) break;  // enough to prove it reads
  }
  std::printf("  uri=%s\n   detected storage_identifier='%s'  read imu=%zu lidar=%zu  %s\n",
              uri.c_str(), meta.storage_identifier.c_str(), n_imu, n_lidar,
              (n_imu > 0 && n_lidar > 0) ? "OK" : "FAILED");
  return (n_imu > 0 && n_lidar > 0) ? 0 : 1;
}

int main() {
  int rc = 0;
  std::printf("[storage_autodetect_test] empty storage_id auto-detect:\n");
  rc |= probe("/home/frank/data/fast_livo/CBD_Building_01_frontend_raw");                  // sqlite3 dir
  rc |= probe("/home/frank/data/fast_livo/CBD_Building_01_frontend_raw_offset_time_full");  // mcap dir
  std::printf("[storage_autodetect_test] %s\n", rc == 0 ? "ALL OK" : "FAILED");
  return rc;
}
