**Amendment 03（机动纠正，pre-data）**

1. 源码确认（`MPC_POS_MODE=4`）：刹停由 `StickAccelerationXY` 的"零杆 drag"实现，drag 与加速同由 `MPC_ACC_HOR` 决定、`MPC_JERK_MAX` 限 slew；无独立减速参数（`MPC_DEC_HOR_SLOW` 当前版本不存在，`MPC_ACC_HOR_MAX` 在 mode4 不用）。Amendment 02 的减速侧候选作废（系旧版文档）。
2. 旋钮回到 `MPC_ACC_HOR`，但机动纠正为"建立速度到 `MPC_VEL_MANUAL` 附近再回中"（半杆 eval234 无法暴露 drag）。次要候选 `MPC_JERK_MAX`。
3. 参数污染修复：每次运行显式设定 `MPC_ACC_HOR`、`MPC_JERK_MAX`，默认档显式设 `3.0/8.0`，不依赖 PX4 持久化存储；回读校验。
4. 本扫描兼作 oracle violation-标签真实数据验证。其余冻结量不变。
