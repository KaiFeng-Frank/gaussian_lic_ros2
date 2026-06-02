# Paper Completion Audit

Status: INCOMPLETE
Strict matrix: PASS
Blocking items: 14

| Evidence | Kind | Reason |
| --- | --- | --- |
| `fastlivo2/Retail_Street/continuous_time_native_parity` | native_tracking_report | paper tracking candidate reference is not a usable trajectory: path 0 m < 0.5 m (ros1_mapper_baseline) |
| `fastlivo2/Retail_Street/continuous_time_native_parity` | native_tracking_report | paper tracking candidate current trajectory diverges before parity compare: max step 1.95e+05 m > 5 m |
| `fastlivo2/Retail_Street/continuous_time_native_parity` | native_tracking_report | paper tracking candidate comparison fails 5 cm parity: matched=22, coverage=91.67%, rmse=1.37e+05 m |
| `m2dgr/room_01/continuous_time_native_parity` | native_tracking_report | paper tracking candidate current trajectory diverges before parity compare: max step 6.44e+05 m > 5 m |
| `m2dgr/room_01/continuous_time_native_parity` | native_tracking_report | paper tracking candidate comparison fails 5 cm parity: matched=21, coverage=17.80%, rmse=2.25e+05 m |
| `mcd/ntu_day_01/continuous_time_native_parity` | native_tracking_report | paper tracking candidate current trajectory diverges before parity compare: max step 2.65e+05 m > 5 m |
| `mcd/ntu_day_01/continuous_time_native_parity` | native_tracking_report | paper tracking candidate comparison fails 5 cm parity: matched=10, coverage=90.91%, rmse=9.96e+04 m |
| `r3live/hku_park_00/continuous_time_native_parity` | native_tracking_report | paper tracking candidate reference is not a usable trajectory: path 0 m < 0.5 m (ros1_mapper_baseline) |
| `r3live/hku_park_00/continuous_time_native_parity` | native_tracking_report | paper tracking candidate current trajectory diverges before parity compare: max step 1.02e+06 m > 5 m |
| `r3live/hku_park_00/continuous_time_native_parity` | native_tracking_report | paper tracking candidate comparison fails 5 cm parity: matched=21, coverage=91.30%, rmse=2.6e+05 m |
| `documentation/readme` | documentation | README still self-declares incomplete paper parity: not a claim of universal super-paper performance |
| `documentation/readme` | documentation | README still self-declares incomplete paper parity: RMSE-gated continuous-time native tracker parity |
| `documentation/readme` | documentation | README still self-declares incomplete paper parity: full RMSE-gated native Coco-LIC2 tracking BA remains |
| `documentation/readme` | documentation | README still self-declares incomplete paper parity: not a 100% paper/super-paper parity claim |
