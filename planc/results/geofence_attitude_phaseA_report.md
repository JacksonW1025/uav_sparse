STATIC VERDICT: 钳位器分量 = UNCOVERED; 反应式兜底 = AC_Fence::check 基于实际位置、约 3 Hz、边界线上 fire、FENCE_ACTION 后模式接管拉回; 动态预测 = 动态会是 clean-unsafe 硬见证(窄到中等窗口)。

# geofence_attitude Phase A Report

## A0 Source And Scope

本仓库当前 checkout 不含完整 ArduPilot 源码树, 因此静态审计使用官方 ArduPilot `Copter-4.4.1` tag 的只读浅克隆: `/tmp/ardupilot-Copter-4.4.1`, SHA `e010f97906087a3a1975e1c4fcc1f88a249599ce`。没有跑 SITL。

本格的问题是: 水平 geofence 的位置目标准入能否跨到 `SET_ATTITUDE_TARGET` 的 quaternion/body-rate 接口。静态结论是不能。`check_destination_within_fence` 只看一个给定的 `Location` 目标点; 姿态/rate 路径没有这个目标点, 也没有等价的入控制器前水平围栏裁剪。另有反应式总闸 `AC_Fence::check()`, 它基于实际位置, 所以接口无关地能看到姿态飞行造成的越界。

## A1 Clamp Component

`SET_ATTITUDE_TARGET` 是 Copter 广告支持的接口: `GCS_Mavlink.cpp:1451-1459` 包含 `MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET`。消息处理在 `GCS_Mavlink.cpp:1122-1192`: 只检查 guided mode、thrust 未忽略和 quaternion 单位长度, 然后调用 `copter.mode_guided.set_angle(attitude_quat, ang_vel, ...)`。这一段没有围栏调用。

位置目标路径不同。`ModeGuided::set_destination(Vector3f)` 在 `ArduCopter/mode_guided.cpp:329-339` 构造 `dest_loc` 并调用 `copter.fence.check_destination_within_fence(dest_loc)`。`ModeGuided::set_destination(Location)` 在 `mode_guided.cpp:421-430` 同样调用。`set_destination_posvelaccel` 在 `mode_guided.cpp:563-574` 对含 position destination 的 pos+vel+accel 目标调用该检查。对应的 fence checker 在 `libraries/AC_Fence/AC_Fence.cpp:590-628`, 对传入 `Location` 检查 ALT_MAX、ALT_MIN、circle radius 和 polygon。

姿态路径没有目标点。`ModeGuided::set_angle` 在 `mode_guided.cpp:626-653` 只保存 quaternion、body rates、thrust/climb rate 和更新时间。`angle_control_run` 在 `mode_guided.cpp:911-977` 最终直接调用 `input_rate_bf_roll_pitch_yaw` 或 `input_quaternion`。`AC_AttitudeControl::input_quaternion` 在 `libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:231-266`, `input_rate_bf_roll_pitch_yaw` 在 `AC_AttitudeControl.cpp:421-455`; 二者都没有 `AC_Fence` 或水平围栏引用。负证据扫描也只在 guided position/destination 和 guided velocity avoidance 路径发现 fence/avoidance 调用。

因此钳位器分量裁决为 **UNCOVERED**。补充边界: guided velocity-only/accel-only 没有 destination admission, 但它们在 `mode_guided.cpp:785-789` 进入 `copter.avoid.adjust_velocity(...)`; `AC_Avoid` 默认包含 stop-at-fence bit (`libraries/AC_Avoidance/AC_Avoid.h:10-15`) 并用 `FENCE_MARGIN` 调整速度 (`AC_Avoid.cpp:690-777`)。这反而说明 FENCE_MARGIN 的提前减速属于 velocity/position 控制路径, 不属于 attitude/rate 入口钳位。

## A2 Reactive Fallback

`AC_Fence::check()` 是接口无关的反应式总闸。circle fence 在 `libraries/AC_Fence/AC_Fence.cpp:498-538` 用 `AP::ahrs().get_relative_position_NE_home(home)` 得到实际位置, `_home_distance >= _circle_radius` 时记录 breach。polygon fence 在 `libraries/AC_Fence/AC_PolyFence_loader.cpp:207-268` 用 `AP::ahrs().get_location(loc)` 取当前位置后判断 inclusion/exclusion。`AC_Fence::check` 在 `AC_Fence.cpp:542-588` 统一调 alt/circle/polygon checks。

调用频率按 scheduler 是约 3 Hz: `ArduCopter/Copter.cpp:180` 注册 `SCHED_TASK(three_hz_loop, 3, ...)`, `Copter.cpp:591-605` 在该 loop 中调用 `fence_check()`。`ArduCopter/fence.cpp:8` 的注释写 1 Hz, 但实际调度表是更强证据。

`FENCE_MARGIN` 不是 breach 提前量。默认和参数说明在 `libraries/AC_Fence/AC_Fence.cpp:87-93`; circle breach 直接在 radius 线上判定 (`AC_Fence.cpp:511-523`)。margin 参与 pre-arm 合法性和 stop-at-fence avoidance, 例如 `AC_Avoid.cpp:708-777` 以 `fence->get_margin()` 计算减速/截停距离。

breach 后动作在 `ArduCopter/fence.cpp:23-82`: 新 breach 且 `FENCE_ACTION != REPORT_ONLY` 时, 默认 `RTL_AND_LAND` 切 RTL, 失败则 Land; 也可 Always Land、SmartRTL、Brake。`libraries/AC_Fence/AC_Fence.cpp:60-68` 显示 Copter 默认 action 是 `AC_FENCE_ACTION_RTL_AND_LAND`。Brake 在 `ArduCopter/mode_brake.cpp:10-69` 通过 position controller 设零 XY velocity 并更新 thrust vector; RTL 在 `ArduCopter/mode_rtl.cpp:127-185` 设回航 waypoint 并跑 `wp_nav->update_wpnav()`。所以兜底存在, 但从实际越界到模式切换, 再到水平动量被有效拉住, 有可观时间/距离窗口。

合法不 fire/晚 fire 的边界也明确: `AC_Fence::check` 在 disabled、无 enabled fences 时返回 0 (`AC_Fence.cpp:548-552`), `FENCE_ACTION=0` 只报告不接管 (`fence.cpp:30-32`), breach 后 `set_mode()` 会调用 `fence.manual_recovery_start()` (`ArduCopter/mode.cpp:315-320`), 使后续约 10 秒内 `check` 返回 0 (`AC_Fence.cpp:555-564`)。这些是配置/恢复语义, 不用于制造本格缺口; 本格动态应保持 fence 合法启用。

## A3 Topology And Prediction

拓扑是 **钳位器作用域缺口 + 反应式兜底**。它与倾角格同构于“位置/生成控制路径有承诺, 直达姿态/rate 接口不组合”, 但机制换成水平围栏, 后果换成突破边界或进入危险空间。

动态预测预注册为: **动态会是 clean-unsafe 硬见证(窄到中等窗口)**。依据是三点: attitude/rate 没有 pre-controller fence admission; reactive check 在实际 boundary breach 后按约 3 Hz 发现; FENCE_ACTION 通过 RTL/Brake/Land 的控制器逐步拉回, 不是瞬时阻断。若动态 Oracle-A 把危险空间定义为 fence boundary 立刻外侧, 预测带可能很窄但存在。若 boundary 外有缓冲障碍带, 带宽取决于可达速度、调度相位、模式切换和刹停响应。

关键不确定点: SITL 中姿态/rate 能达到的水平速度; RTL 与 Brake action 的拉回距离差异; AHRS/GPS 更新与 3 Hz fence loop 的相位; Oracle-A 几何到底以越界本身还是边界外障碍为硬后果。若动态显示总闸在硬后果前有效拉回且窗口约为 0, 应如实改判 C-ABSENT/覆盖(反应式)。若源码或动态发现姿态路径上另有围栏钳位, 应改判 COVERED; 本次静态未发现。

新增价值: 这不是重复倾角格, 而是把同栈接口不组合从姿态限制推进到 geofence 机制。它也要求 E 从风升级为几何环境: 至少要有启用的 circle/polygon fence, boundary 外危险空间或障碍带, 起点在 fence 内, `FENCE_ENABLE=1`, `FENCE_TYPE` 包含 circle 或 polygon, `FENCE_ACTION` 默认 RTL-or-Land 或另设 Brake 对照, 不靠关闭 fence 造缺口。

## Honest Boundaries

- 这格若动态成立, 是操作者一条合法输入, 即姿态/rate 飞行, 可达突破边界; 不是环境自发越界。
- 静态阶段不下 clean-unsafe 最终结论; 本报告只给覆盖位、兜底分析和动态预测。
- 不 claim 标度律、面积律或鲁棒区域大小。
