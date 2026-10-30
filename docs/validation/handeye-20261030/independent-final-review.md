# Independent final acceptance: new-board handeye campaign

PASS. The finite A→B→C campaign completed48/48 actual moves and accepted captures, with16 points per group, unchanged original quality/arrival/stationary/native guards and no automatic group retry, Home, reference reset or production TF. Every group returned0 and accepted=true before the next began. The final saved real/fresh/referenced IDLE mode5 stationary proof spans0.645498s. This is historical recorded evidence; the reviewer issued no ROS/device actions or further-motion approval.

Each group's raw-motion/capture/PNG/CDR/FK/PnP/train-only PARK review passes564 independent checks. All48 images have88 ChArUco inner corners; group reprojection ranges are A0.14667–0.20165px,B0.15108–0.25626px,C0.15495–0.20510px. All48 image stamps and color/depth frame counters strictly increase, and every capture follows the new fixed board placement. Captured endpoint deviations from the audited branch are at most A3.91764e-5,B4.47254e-5,C3.54208e-5rad. Final execution/count/frame/static review passes69 checks.

Only this new fixed placement is used. The old52 raw observations are excluded. There are36 training observations at24 unique training poses and12 heldout observations at4 globally fixed validation geometries. All six predetermined candidate fits are recomputed from their original role-filtered training transforms; no X-matrix averaging, heldout fitting, heldout alignment or residual-based candidate selection occurs. Joint/membership/common-residual review passes394 checks and reproduces every candidate X and all48 common per-point residuals.

The A/C repeat external-transform difference is0.394677mm /0.001539273rad (about0.0882degrees). A/C repeats the same training targets in reverse order; this comparison confounds repetition,time and direction and cannot uniquely identify backlash or prove that the mount/board never moved.

All candidates below are evaluated against the same12 heldout observations using the first A training observation as the common raw reference; no heldout observation defines the reference. The12 observations represent only4 unique geometries.

| Training pool | Train observations | Common heldout translation median / maximum (mm) | Common heldout rotation median / maximum (rad) |
|---|---:|---:|---:|
| A only |12|0.206698 /0.348556|0.001384269 /0.002123649|
| B only |12|0.230067 /0.356076|0.001538063 /0.001970168|
| C only |12|0.186807 /0.330964|0.001473022 /0.001961244|
| A+C |24|0.197070 /0.336309|0.001441523 /0.001940440|
| All three |36|0.208875 /0.339257|0.001479076 /0.001942851|
| Balanced unique poses |24|0.216352 /0.335949|0.001501576 /0.001991481|

Balanced selection was predetermined as A-even/C-odd training IDs plus all B training IDs; all heldout observations remain excluded. It changes repeated-pose weights and is not uniformly better. The all36 fit also does not improve every metric over every individual group. The separate group-own-reference heldout maxima are A0.320245mm/0.001963882rad,B0.324010mm/0.001968012rad,C0.423459mm/0.001852158rad; those use different training anchors and should not be interchanged with the common-reference table.

These results establish recorded internal consistency and observed repeatability, not externally measured absolute grasp accuracy. They do not justify claiming0.2mm physical positioning accuracy. TCP/installation, robot kinematic and camera-intrinsic systematics, board flatness/scale, true exposure synchronization and actual tracking/stopping error remain externally unverified. Stationarity.csv stores historical health/freshness decisions rather than the complete raw full-dwell robot stream; final paired measured CDR states are independently decoded. Action results are persisted in session YAML rather than complete action-result wire traces. The recorded files and pinned binary/source identities are the provenance basis.

The finite preexecution review included the actual core Planner/GeometryGuard's4391-sample known-endpoint sequence, independent full-tool/arm geometry and increased engineering provisions. Future SDK branch selection and real dynamic clearance were not certified. Depth holes/out-of-view/anomalous pixels were retained, and the200x155mm outer-board enclosure was explicitly an assumption. Those limitations are not removed by the successful calibration run.

Deliverables are the48 original session datasets in group_A/B/C/execute/session; joint/*-product-result.yaml and membership.yaml for all predetermined fits; independent_final_math/summary.json with all six X matrices, common per-point residuals and coverage proxies; and this folder's summary/checks/input-manifest. The initial lift's software monitor/UUID failures and scoped corrections remain separately documented in independent_lift_v3_review/independent_lift_v4_review, without replacing failed evidence or attributing the original lost trigger fields with certainty.

physical_accuracy_verified=false; production_tf_published=false. Any later real-point/ruler validation, deployment or further motion is a separate decision and is not authorized by this acceptance report.
