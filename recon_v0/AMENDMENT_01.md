**Amendment 01（看数据前，成本驱动，不改任何冻结量）**
1. 解锁：`compute_robustness` 增加可选 `window=(t0,t1)`，默认保持整段行为；recon 一律传 `(11.0,13.0)`，并附回归测试验证可区分松杆暂态。窗口 `[11,13]` 早在 prereg 冻结，本次仅令代码兑现该选择，预注册不受影响。
2. 执行顺序改为 Stage 0 → Stage 2 →（条件性）Stage 1。理由：Stage 2 复用 `px4_position` 零新场景且为预设主方向；Stage 1 需新建 PX4 ALTCTL 场景（仓库现无）。Stage 1 仅当 Stage 2 为 null 时执行，执行前先盘点 `px4_hold/px4_transition/ap_althold` 是否可复用。
3. 正对照精确化：`MPC_ACC_HOR=0.5`（合法下限 2.0 之外，标 `legal=False`，仅作 oracle 灵敏度自检）；Stage 2 扫描严格限于 `[2.0,15.0]`。
4. 其余冻结量（F、阈值、四子窗、J、2σ 门、survivor 门、anti-trap、假设 H_mode/H_conjunction/H_null）一律不变。
