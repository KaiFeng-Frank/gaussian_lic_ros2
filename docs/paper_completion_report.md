# Paper Completion Audit

Status: INCOMPLETE
Strict matrix: PASS
Blocking items: 26

| Evidence | Kind | Reason |
| --- | --- | --- |
| `fastlivo2/Retail_Street/continuous_time_native_parity` | native_tracking_report | native tracking report lacks self-described evidence paths: bag_dir, tum_path, native_report_path |
| `fastlivo2/Retail_Street/continuous_time_native_parity` | native_tracking_report | native tracking report predates required output-pose guard or does not fail on guard rejections |
| `fastlivo2/Retail_Street/continuous_time_native_parity` | native_tracking_report | native tracking report lacks output_max_pose_step_m |
| `fastlivo2/Retail_Street/continuous_time_native_parity` | native_tracking_report | paper tracking candidate reference is not a usable trajectory: path 0 m < 0.5 m (ros1_mapper_baseline) |
| `fastlivo2/Retail_Street/continuous_time_native_parity` | native_tracking_report | paper tracking candidate current trajectory diverges before parity compare: max step 1.95e+05 m > 5 m; first_bad=(idx=8, stamp=946685438.360942006->946685438.460942030, dt=0.1s, step=15.8m, speed=158m/s); max_bad=(idx=21, stamp=946685439.660941958->946685439.760941982, dt=0.1s, step=1.95e+05m, speed=1.95e+06m/s) |
| `fastlivo2/Retail_Street/continuous_time_native_parity` | native_tracking_report | paper tracking candidate comparison fails 5 cm parity: matched=22, coverage=91.67%, rmse=1.37e+05 m |
| `m2dgr/room_01/continuous_time_native_parity` | native_tracking_report | native tracking report lacks self-described evidence paths: bag_dir, tum_path, native_report_path |
| `m2dgr/room_01/continuous_time_native_parity` | native_tracking_report | native tracking report predates required output-pose guard or does not fail on guard rejections |
| `m2dgr/room_01/continuous_time_native_parity` | native_tracking_report | native tracking report lacks output_max_pose_step_m |
| `m2dgr/room_01/continuous_time_native_parity` | native_tracking_report | paper tracking candidate current trajectory diverges before parity compare: max step 6.44e+05 m > 5 m; first_bad=(idx=8, stamp=1627643483.772107124->1627643483.872107029, dt=0.1s, step=8.56m, speed=85.6m/s); max_bad=(idx=20, stamp=1627643484.972106934->1627643485.072107077, dt=0.1s, step=6.44e+05m, speed=6.44e+06m/s) |
| `m2dgr/room_01/continuous_time_native_parity` | native_tracking_report | paper tracking candidate comparison fails 5 cm parity: matched=21, coverage=17.80%, rmse=2.25e+05 m |
| `mcd/ntu_day_01/continuous_time_native_parity` | native_tracking_report | native tracking report lacks self-described evidence paths: bag_dir, tum_path, native_report_path |
| `mcd/ntu_day_01/continuous_time_native_parity` | native_tracking_report | native tracking report predates required output-pose guard or does not fail on guard rejections |
| `mcd/ntu_day_01/continuous_time_native_parity` | native_tracking_report | native tracking report lacks output_max_pose_step_m |
| `mcd/ntu_day_01/continuous_time_native_parity` | native_tracking_report | paper tracking candidate current trajectory diverges before parity compare: max step 2.65e+05 m > 5 m; first_bad=(idx=3, stamp=1644823131.858094215->1644823131.958094358, dt=0.1s, step=5.93m, speed=59.3m/s); max_bad=(idx=17, stamp=1644823133.258094311->1644823133.358094215, dt=0.1s, step=2.65e+05m, speed=2.65e+06m/s) |
| `mcd/ntu_day_01/continuous_time_native_parity` | native_tracking_report | paper tracking candidate comparison fails 5 cm parity: matched=10, coverage=90.91%, rmse=9.96e+04 m |
| `r3live/hku_park_00/continuous_time_native_parity` | native_tracking_report | native tracking report lacks self-described evidence paths: bag_dir, tum_path, native_report_path |
| `r3live/hku_park_00/continuous_time_native_parity` | native_tracking_report | native tracking report predates required output-pose guard or does not fail on guard rejections |
| `r3live/hku_park_00/continuous_time_native_parity` | native_tracking_report | native tracking report lacks output_max_pose_step_m |
| `r3live/hku_park_00/continuous_time_native_parity` | native_tracking_report | paper tracking candidate reference is not a usable trajectory: path 0 m < 0.5 m (ros1_mapper_baseline) |
| `r3live/hku_park_00/continuous_time_native_parity` | native_tracking_report | paper tracking candidate current trajectory diverges before parity compare: max step 1.02e+06 m > 5 m; first_bad=(idx=8, stamp=1627720596.439718246->1627720596.539718390, dt=0.1s, step=10.5m, speed=105m/s); max_bad=(idx=20, stamp=1627720597.639718294->1627720597.739718437, dt=0.1s, step=1.02e+06m, speed=1.02e+07m/s) |
| `r3live/hku_park_00/continuous_time_native_parity` | native_tracking_report | paper tracking candidate comparison fails 5 cm parity: matched=21, coverage=91.30%, rmse=2.6e+05 m |
| `documentation/readme` | documentation | README still self-declares incomplete paper parity: not a claim of universal super-paper performance |
| `documentation/readme` | documentation | README still self-declares incomplete paper parity: RMSE-gated continuous-time native tracker parity |
| `documentation/readme` | documentation | README still self-declares incomplete paper parity: full RMSE-gated native Coco-LIC2 tracking BA remains |
| `documentation/readme` | documentation | README still self-declares incomplete paper parity: not a 100% paper/super-paper parity claim |
