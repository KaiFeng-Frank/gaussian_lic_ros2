// cm-grade ingest validation: read CBD_Building_01_frontend_raw_offset_time_full
// (.mcap, point_step=20 with per-point offset_time UINT32@16) and verify the
// per-point time channel that continuous-time B-spline deskew REQUIRES — the
// property frontend_raw (point_step=16, no time) cannot provide. Replicates
// upstream livox_feature_extraction.cpp ParsePointCloud field reads + filter:
//   keep iff (line < n_scan) && ((tag & 0x30)==0x10), RTPoint.time = int64(offset_time).
// This de-risks the corrected ingest target before the full msg_manager port.
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <map>
#include <vector>

#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace {
// resolve a named field's byte offset within a point (robust vs hardcoding)
int field_offset(const sensor_msgs::msg::PointCloud2 & m, const char * name, int * datatype = nullptr) {
  for (const auto & f : m.fields) {
    if (f.name == name) { if (datatype) *datatype = f.datatype; return static_cast<int>(f.offset); }
  }
  return -1;
}
}

int main(int argc, char ** argv) {
  const bool explicit_bag = argc > 1;
  const std::string bag = explicit_bag ? argv[1]
    : (std::getenv("COCOLIC_TEST_FASTLIVO2_OFFSET_TIME_BAG")
      ? std::getenv("COCOLIC_TEST_FASTLIVO2_OFFSET_TIME_BAG")
      : "/home/frank/data/fast_livo/CBD_Building_01_frontend_raw_offset_time_full");
  if (!std::filesystem::exists(bag)) {
    std::printf("[ingest_offset_time] SKIP: bag path is unavailable: %s\n", bag.c_str());
    return explicit_bag ? 1 : 0;
  }
  const int n_scan = 6;  // Livox Avia scan lines (upstream n_scan)

  rosbag2_storage::StorageOptions so;
  so.uri = bag;
  so.storage_id = "mcap";
  rosbag2_cpp::Reader reader;
  reader.open(so);

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc_ser;

  size_t n_lidar = 0, total_pts = 0, total_kept = 0;
  bool first = true;
  int off_x=-1, off_int=-1, off_tag=-1, off_line=-1, off_ot=-1, ot_dtype=-1, pstep=0;
  // aggregate per-point-time stats over first scan + global
  double first_ot_min = 1e18, first_ot_max = -1e18;
  size_t first_nondecr = 0, first_cmp = 0;
  std::map<int,size_t> line_hist;
  double global_ot_max = -1e18;

  while (reader.has_next()) {
    auto bm = reader.read_next();
    if (bm->topic_name != "/livox/lidar") continue;
    rclcpp::SerializedMessage ser(*bm->serialized_data);
    sensor_msgs::msg::PointCloud2 m;
    pc_ser.deserialize_message(&ser, &m);

    if (first) {
      off_x   = field_offset(m, "x");
      off_int = field_offset(m, "intensity");
      off_tag = field_offset(m, "tag");
      off_line= field_offset(m, "line");
      off_ot  = field_offset(m, "offset_time", &ot_dtype);
      pstep   = static_cast<int>(m.point_step);
      std::printf("[ingest_offset_time] bag=%s\n  point_step=%d width=%u  "
                  "x@%d intensity@%d tag@%d line@%d offset_time@%d(dtype=%d, want 6=UINT32)\n",
                  bag.c_str(), pstep, m.width, off_x, off_int, off_tag, off_line, off_ot, ot_dtype);
      if (off_ot < 0) { std::printf("  FAILED: no offset_time field — wrong bag for cm\n"); return 1; }
    }

    const size_t npts = static_cast<size_t>(m.width) * m.height;
    const uint8_t * base = m.data.data();
    int64_t prev_ot = -1;
    for (size_t i = 0; i < npts; ++i) {
      const uint8_t * p = base + i * m.point_step;
      uint8_t tag, line; uint32_t ot_raw;
      std::memcpy(&tag,  p + off_tag,  1);
      std::memcpy(&line, p + off_line, 1);
      std::memcpy(&ot_raw, p + off_ot, 4);
      const int64_t ot = static_cast<int64_t>(ot_raw);   // upstream: RTPoint.time = int64(offset_time)
      ++total_pts;
      // upstream ParsePointCloud filter
      if (line < n_scan && ((tag & 0x30) == 0x10)) {
        ++total_kept;
        line_hist[line]++;
        if (ot > global_ot_max) global_ot_max = ot;
        if (first) {
          double otd = static_cast<double>(ot);
          if (otd < first_ot_min) first_ot_min = otd;
          if (otd > first_ot_max) first_ot_max = otd;
          if (prev_ot >= 0) { ++first_cmp; if (ot >= prev_ot) ++first_nondecr; }
          prev_ot = ot;
        }
      }
    }
    ++n_lidar;
    first = false;
  }

  std::printf("  scans=%zu  total_pts=%zu  kept(tag&0x30==0x10, line<%d)=%zu (%.1f%%)\n",
              n_lidar, total_pts, n_scan, total_kept, 100.0 * total_kept / (total_pts ? total_pts : 1));
  std::printf("  first-scan kept offset_time: min=%.0fns max=%.0fns span=%.3fms  monotonic(nondecr)=%zu/%zu (%.1f%%)\n",
              first_ot_min, first_ot_max, (first_ot_max - first_ot_min) * 1e-6,
              first_nondecr, first_cmp, 100.0 * first_nondecr / (first_cmp ? first_cmp : 1));
  std::printf("  global max offset_time=%.3fms (Livox 10Hz scan ~100ms expected)\n", global_ot_max * 1e-6);
  std::printf("  line histogram:");
  for (auto & kv : line_hist) std::printf(" L%d=%zu", kv.first, kv.second);
  std::printf("\n");

  // cm-grade ingest is valid iff: offset_time present, scans read, per-point time
  // spans a plausible scan window (~tens of ms) and is mostly monotonic within a scan.
  bool ok = (n_lidar > 100) && (off_ot >= 0) &&
            (first_ot_max - first_ot_min > 1e6) &&            // > 1ms span (real intra-scan time)
            (first_cmp == 0 || (double)first_nondecr / first_cmp > 0.5);
  std::printf("  CM-GRADE INGEST %s\n", ok ? "OK" : "FAILED");
  return ok ? 0 : 1;
}
