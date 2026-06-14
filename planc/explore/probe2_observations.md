# Probe 2 Observations

## 观察记录

- Stage-0 `bbs_probe2_stage0_low_angle_high_turb_r4800`: ANGLE_MAX=1000 cdeg, turb=2.5, r=4800.0 PWM/s, max_drop=0.021329879760742188, touch=False, sustained_att_error=False。
- Stage-0 在这个合法低 ANGLE_MAX/高湍流/重模型组合下没有诱导出命令窗口内触地或持续大姿态误差。
- 主扫固定 `ALT_HOLD` RC override、`ANGLE_MAX=3000` cdeg、mass model `planc/config/minalt_models/mass_6_00.json`、`SIM_WIND_SPD=4 m/s`、`SIM_WIND_TURB=1.5`；r 序列为 150, 300, 600, 1200, 2400, 4800 PWM/s。
- 电机限幅余量序列: 362.0, 369.0, 357.0, 370.0, 362.0, 356.0 PWM；这个量越小表示越贴近 1000/2000 PWM 限。
- 本轮主扫电机余量范围为 356.0-370.0 PWM，没有进入贴限饱和段。
- 姿态跟踪误差峰值序列: 0.32, 0.63, 0.98, 0.38, 0.63, 1.23 deg。
- 最大掉高序列: 0.01, 0.01, 0.02, 0.01, 0.01, 0.01 m。
- 单调性粗看: 电机贴限需求 非单调，需求差分=-7.00, 12.00, -13.00, 8.00, 6.00；姿态误差 非单调，需求差分=0.31, 0.35, -0.60, 0.25, 0.60；掉高 非单调，需求差分=0.00, 0.01, -0.01, -0.00, -0.00。
- 命令核对: RCIN peak 序列 450.0, 450.0, 450.0, 450.0, 450.0, 450.0 PWM；RCIN p95 slew 序列 60, 120, 240, 480, 960, 1921 PWM/s；DesRoll peak 序列 26.87, 26.87, 26.87, 26.87, 26.87, 26.87 deg。
- 打杆削减/限速记录: r=150: RCIN p95 slew 60 PWM/s (<70% of command); r=300: RCIN p95 slew 120 PWM/s (<70% of command); r=600: RCIN p95 slew 240 PWM/s (<70% of command); r=1200: RCIN p95 slew 480 PWM/s (<70% of command); r=2400: RCIN p95 slew 960 PWM/s (<70% of command); r=4800: RCIN p95 slew 1921 PWM/s (<70% of command)

## 对方法设计的启示

- `min_motor_limit_margin_pwm` 是最直接的峰值型权限需求候选；本轮未贴限且非单调幅度较小，若后续进入贴限段要把饱和信息保留下来而不是只拟合线性斜率。
- `peak_att_error_deg` 对控制误差更敏感，可作为需求泛函的第二视角；若它随 r 有弹跳，应把湍流重复或模型噪声纳入采样策略。
- `max_drop_m` 是外部后果量，适合和权限代理一起记录；它不应替代命令核对，因为掉高可能受高度控制器和清理降落阶段影响。
- 若后续数据继续出现非单调或悬崖，BBS 的二分边界搜索应改为保留多点局部模型的 BO/active learning；若单调只在某个代理上干净，二分应绑定那个代理而不是绑定二值触地。
