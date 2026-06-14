# Overdraw BBS Explore Findings

## 总观察

- DataFlash 中可读字段决定了本轮可用的余量/需求代理；字段清点保留了取不到项，后续建模应从实际字段出发。
- 能量探针把 `BAT.CurrTot`、短窗消耗率、触发时到家距离组合成连续 margin proxy；二值落点只作为叠加标记。
- 控制权限探针把命令核对、峰值电机贴限、姿态误差和掉高分开记录，避免把“命令没有施加到”误读成“需求不高”。
- 外推探针只在最便宜能量场景上比较近段拟合和远段留出；只记录误差尺度和设计信号。

## Probe 0

# Probe 0 Telemetry Field Inventory

- run_id: `bbs_probe0_inventory`
- DataFlash: `planc/logs/bbs_probe0_inventory.BIN`
- 解析时间: 2026-06-14T09:30:17.974619+00:00

## 想读的量 -> 实际字段名 / 取不到

| 想读的量 | 消息类型 | 实际字段名 / 取不到 | 样本数 |
|---|---|---|---:|
| 电量 | `BAT` | `Volt`, `VoltR`, `Curr`, `CurrTot`, `EnrgTot`, `RemPct` | 1097 |
| 姿态 | `ATT` | `Roll`, `DesRoll`, `Pitch`, `DesPitch`, `Yaw`, `DesYaw` | 43443 |
| 角速率 | `RATE` | `R`, `RDes`, `P`, `PDes`, `Y`, `YDes`, `ROut`, `POut`, `YOut`, `AOut` | 43443 |
| 电机输出 | `RCOU` | `C1`, `C2`, `C3`, `C4`, `C5`, `C6`, `C7`, `C8` | 1097 |
| 遥控输入 | `RCIN` | `C1`, `C2`, `C3`, `C4`, `C5`, `C6`, `C7`, `C8` | 1098 |
| 高度/爬升 | `CTUN` | `Alt`, `BAlt`, `DAlt`, `CRt`, `DCRt`, `ThO`, `ThH` | 1097 |
| 位置 | `POS` | `Lat`, `Lng`, `Alt`, `RelHomeAlt` | 2586 |
| 位置备用 | `GPS` | `Lat`, `Lng`, `Alt`, `Spd`, `GCrs`, `Status` | 519 |

## 实际消息类型和字段

| 消息类型 | 样本数 | 字段 |
|---|---:|---|
| `AHR2` | 2693 | `Alt`, `Lat`, `Lng`, `Pitch`, `Q1`, `Q2`, `Q3`, `Q4`, `Roll`, `TimeUS`, `Yaw` |
| `ARM` | 2 | `ArmChecks`, `ArmState`, `Forced`, `Method`, `TimeUS` |
| `ATT` | 43443 | `AEKF`, `DesPitch`, `DesRoll`, `DesYaw`, `ErrRP`, `ErrYaw`, `Pitch`, `Roll`, `TimeUS`, `Yaw` |
| `AUXF` | 1 | `TimeUS`, `function`, `pos`, `result`, `source` |
| `BARO` | 2194 | `Alt`, `CRt`, `GndTemp`, `Health`, `I`, `Offset`, `Press`, `SMS`, `Temp`, `TimeUS` |
| `BAT` | 1097 | `Curr`, `CurrTot`, `EnrgTot`, `Instance`, `RemPct`, `Res`, `Temp`, `TimeUS`, `Volt`, `VoltR` |
| `CTRL` | 1097 | `RMSPitchD`, `RMSPitchP`, `RMSRollD`, `RMSRollP`, `RMSYaw`, `TimeUS` |
| `CTUN` | 1097 | `ABst`, `Alt`, `BAlt`, `CRt`, `DAlt`, `DCRt`, `DSAlt`, `SAlt`, `TAlt`, `ThH`, `ThI`, `ThO`, `TimeUS` |
| `D32` | 1 | `Id`, `TimeUS`, `Value` |
| `DSF` | 109 | `Blk`, `Bytes`, `Dp`, `FAv`, `FMn`, `FMx`, `TimeUS` |
| `DU32` | 109 | `Id`, `TimeUS`, `Value` |
| `ERR` | 1 | `ECode`, `Subsys`, `TimeUS` |
| `ESC` | 42408 | `CTot`, `Curr`, `Err`, `Instance`, `MotTemp`, `RPM`, `RawRPM`, `Temp`, `TimeUS`, `Volt` |
| `EV` | 11 | `Id`, `TimeUS` |
| `FILE` | 519 | `Data`, `FileName`, `Length`, `Offset` |
| `FMT` | 175 | `Columns`, `Format`, `Length`, `Name`, `Type` |
| `FMTU` | 175 | `FmtType`, `MultIds`, `TimeUS`, `UnitIds` |
| `GPA` | 519 | `Delta`, `HAcc`, `I`, `SAcc`, `SMS`, `TimeUS`, `Und`, `VAcc`, `VDop`, `VV`, `YAcc` |
| `GPS` | 519 | `Alt`, `GCrs`, `GMS`, `GWk`, `HDop`, `I`, `Lat`, `Lng`, `NSats`, `Spd`, `Status`, `TimeUS`, `U`, `VZ`, `Yaw` |
| `GUIP` | 3 | `Terrain`, `TimeUS`, `Type`, `aX`, `aY`, `aZ`, `pX`, `pY`, `pZ`, `vX`, `vY`, `vZ` |
| `IMU` | 5486 | `AH`, `AHz`, `AccX`, `AccY`, `AccZ`, `EA`, `EG`, `GH`, `GHz`, `GyrX`, `GyrY`, `GyrZ`, `I`, `T`, `TimeUS` |
| `MAG` | 3294 | `Health`, `I`, `MOX`, `MOY`, `MOZ`, `MagX`, `MagY`, `MagZ`, `OfsX`, `OfsY`, `OfsZ`, `S`, `TimeUS` |
| `MAV` | 109 | `TimeUS`, `chan`, `flags`, `rxdp`, `rxp`, `ss`, `tf`, `txp` |
| `MAVC` | 9 | `Cmd`, `Fr`, `P1`, `P2`, `P3`, `P4`, `Res`, `SC`, `SS`, `TC`, `TS`, `TimeUS`, `WL`, `X`, `Y`, `Z` |
| `MODE` | 5 | `Mode`, `ModeNum`, `Rsn`, `TimeUS` |
| `MOTB` | 11 | `BatVolt`, `FailFlags`, `LiftMax`, `ThLimit`, `ThrAvMx`, `ThrOut`, `TimeUS` |
| `MSG` | 35 | `Message`, `TimeUS` |
| `MULT` | 15 | `Id`, `Mult`, `TimeUS` |
| `ORGN` | 4 | `Alt`, `Lat`, `Lng`, `TimeUS`, `Type` |
| `PARM` | 1365 | `Default`, `Name`, `TimeUS`, `Value` |
| `PIDA` | 43432 | `Act`, `D`, `Dmod`, `Err`, `FF`, `I`, `Limit`, `P`, `SRate`, `Tar`, `TimeUS` |
| `PIDE` | 34338 | `Act`, `D`, `Dmod`, `Err`, `FF`, `I`, `Limit`, `P`, `SRate`, `Tar`, `TimeUS` |
| `PIDN` | 34338 | `Act`, `D`, `Dmod`, `Err`, `FF`, `I`, `Limit`, `P`, `SRate`, `Tar`, `TimeUS` |
| `PIDP` | 43432 | `Act`, `D`, `Dmod`, `Err`, `FF`, `I`, `Limit`, `P`, `SRate`, `Tar`, `TimeUS` |
| `PIDR` | 43432 | `Act`, `D`, `Dmod`, `Err`, `FF`, `I`, `Limit`, `P`, `SRate`, `Tar`, `TimeUS` |
| `PIDY` | 43432 | `Act`, `D`, `Dmod`, `Err`, `FF`, `I`, `Limit`, `P`, `SRate`, `Tar`, `TimeUS` |
| `PM` | 10 | `ErrC`, `ErrL`, `Ex`, `I2CC`, `I2CI`, `IntE`, `LR`, `Load`, `MaxT`, `Mem`, `NL`, `NLon`, `SPIC`, `TimeUS` |
| `POS` | 2586 | `Alt`, `Lat`, `Lng`, `RelHomeAlt`, `RelOriginAlt`, `TimeUS` |
| `PSCD` | 1030 | `AD`, `DAD`, `DVD`, `PD`, `TAD`, `TPD`, `TVD`, `TimeUS`, `VD` |
| `PSCE` | 858 | `AE`, `DAE`, `DVE`, `PE`, `TAE`, `TPE`, `TVE`, `TimeUS`, `VE` |
| `PSCN` | 858 | `AN`, `DAN`, `DVN`, `PN`, `TAN`, `TPN`, `TVN`, `TimeUS`, `VN` |
| `RATE` | 43443 | `A`, `ADes`, `AOut`, `AOutSlew`, `P`, `PDes`, `POut`, `R`, `RDes`, `ROut`, `TimeUS`, `Y`, `YDes`, `YOut` |
| `RCI2` | 188 | `C15`, `C16`, `OMask`, `TimeUS` |
| `RCIN` | 1098 | `C1`, `C10`, `C11`, `C12`, `C13`, `C14`, `C2`, `C3`, `C4`, `C5`, `C6`, `C7`, `C8`, `C9`, `TimeUS` |
| `RCO2` | 1097 | `C15`, `C16`, `C17`, `C18`, `TimeUS` |
| `RCOU` | 1097 | `C1`, `C10`, `C11`, `C12`, `C13`, `C14`, `C2`, `C3`, `C4`, `C5`, `C6`, `C7`, `C8`, `C9`, `TimeUS` |
| `SIM` | 2725 | `Alt`, `Lat`, `Lng`, `Pitch`, `Q1`, `Q2`, `Q3`, `Q4`, `Roll`, `TimeUS`, `Yaw` |
| `SIM2` | 43895 | `As`, `PD`, `PE`, `PN`, `TimeUS`, `VD`, `VE`, `VN` |
| `SRTL` | 29 | `Action`, `Active`, `D`, `E`, `MaxPts`, `N`, `NumPts`, `TimeUS` |
| `TERR` | 103 | `CHeight`, `Lat`, `Lng`, `Loaded`, `Pending`, `ROfs`, `Spacing`, `Status`, `TerrH`, `TimeUS` |
| `UNIT` | 36 | `Id`, `Label`, `TimeUS` |
| `VER` | 1 | `APJ`, `BST`, `BT`, `FWS`, `FWT`, `GH`, `Maj`, `Min`, `Pat`, `TimeUS` |
| `VIBE` | 2194 | `Clip`, `IMU`, `TimeUS`, `VibeX`, `VibeY`, `VibeZ` |
| `XKF1` | 5430 | `C`, `GX`, `GY`, `GZ`, `OH`, `PD`, `PE`, `PN`, `Pitch`, `Roll`, `TimeUS`, `VD`, `VE`, `VN`, `Yaw`, `dPD` |
| `XKF2` | 5430 | `AX`, `AY`, `AZ`, `C`, `IDX`, `IDY`, `IS`, `MD`, `ME`, `MN`, `MX`, `MY`, `MZ`, `TimeUS`, `VWE`, `VWN` |
| `XKF3` | 5430 | `C`, `ErSc`, `IMX`, `IMY`, `IMZ`, `IPD`, `IPE`, `IPN`, `IVD`, `IVE`, `IVN`, `IVT`, `IYAW`, `RErr`, `TimeUS` |
| `XKF4` | 5430 | `C`, `FS`, `GPS`, `OFE`, `OFN`, `PI`, `SH`, `SM`, `SP`, `SS`, `SV`, `SVT`, `TS`, `TimeUS`, `errRP` |
| `XKF5` | 2715 | `AFI`, `C`, `FIX`, `FIY`, `HAGL`, `Herr`, `NI`, `RI`, `TimeUS`, `eAng`, `ePos`, `eVel`, `offset`, `rng` |
| `XKFS` | 5430 | `AI`, `BI`, `C`, `GI`, `MI`, `SS`, `TimeUS` |
| `XKQ` | 5430 | `C`, `Q1`, `Q2`, `Q3`, `Q4`, `TimeUS` |
| `XKT` | 44 | `AngMax`, `AngMin`, `C`, `Cnt`, `EKFMax`, `EKFMin`, `IMUMax`, `IMUMin`, `TimeUS`, `VMax`, `VMin` |
| `XKTV` | 426 | `C`, `TVD`, `TVS`, `TimeUS` |
| `XKV1` | 209 | `C`, `TimeUS`, `V00`, `V01`, `V02`, `V03`, `V04`, `V05`, `V06`, `V07`, `V08`, `V09`, `V10`, `V11` |
| `XKV2` | 209 | `C`, `TimeUS`, `V12`, `V13`, `V14`, `V15`, `V16`, `V17`, `V18`, `V19`, `V20`, `V21`, `V22`, `V23` |
| `XKY0` | 3432 | `C`, `TimeUS`, `W0`, `W1`, `W2`, `W3`, `W4`, `Y0`, `Y1`, `Y2`, `Y3`, `Y4`, `YC`, `YCS` |
| `XKY1` | 3432 | `C`, `IVE0`, `IVE1`, `IVE2`, `IVE3`, `IVE4`, `IVN0`, `IVN1`, `IVN2`, `IVN3`, `IVN4`, `TimeUS` |


## Probe 1

# Probe 1 Observations

## 观察记录

- 本轮固定 `BATT_LOW_MAH=220`, `BATT_CRT_MAH=60`, `SIM_WIND_DIR=270`, `SIM_WIND_SPD=6 m/s`; D 点数为 5。
- `BAT.CurrTot` 和 `BAT.Curr` 都可读；低电量触发发生在 D 点悬停耗电阶段，触发瞬间 `XKF1` 地速接近 0，所以 margin 代理使用触发后早期 RTL 窗口的 `CurrTot` 斜率和 `XKF1` 地速。
- 触发瞬间实际地速序列: 0.02 m/s, 0.02 m/s, 0.02 m/s, 0.02 m/s, 0.02 m/s；margin 代理使用的返航窗口地速序列: 5.85 m/s, 6.50 m/s, 6.51 m/s, 6.50 m/s, 6.50 m/s。
- margin proxy 序列: 49.7 m, 20.0 m, -0.3 m, -19.6 m, -40.5 m。
- 相邻 D 的 margin proxy 差分: -29.7 m, -20.3 m, -19.3 m, -20.8 m。
- D=120 m 的 margin proxy 贴近 0，但落点仍在 home radius 内；这个点把代理误差和二值切片的边界偏移暴露出来。
- 二值落点使用 `home_radius_m=10`；在这些点里 outside-home 的 D 为 140, 160。
- 可稳定读出最终 margin 符号的提前量: 82.5 s, 82.5 s, 3.2 s, 6.7 s。

## 对方法设计的启示

- 余量代理应优先用 `BAT.CurrTot` 的短窗斜率而不是瞬时 `BAT.Curr`，因为它直接对应累积预算并减少瞬时电流抖动；若任务流程含悬停耗电，速度/耗电率要用返航段代理，而不是触发瞬间地速。
- 早停门可以围绕“margin 符号稳定保持”来定义；本轮提前量使用返航窗口代理重放得到，在线早停需要把返航速度/耗电率改成先验或触发后短窗估计。
- 二值落点只保留了 `home_radius_m` 以内/以外的切片；保留 `margin_proxy_m` 和 `final_home_distance_m` 能给拟合器更多连续尺度。


## Probe 2

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


## Probe 3

# Probe 3 Extrapolation

## 观察记录

- 近段训练点定义为 D<=120 m，远段留出点定义为 D>120 m；没有补跑额外飞行，直接使用探针 1 的 5 个点。
- 余量标度律使用线性 `margin(D)=a*D+b`，拟合得到 `a=-0.8200`, `b=99.66`。
- 远段余量绝对误差: 4.5 m, 8.9 m。
- 用余量符号转成二值后的远段错误: 0, 0。
- 朴素 0/1 外推形式: `constant_near_segment_class`，boundary=n/a；远段二值错误: 1, 1。

## 对方法设计的启示

- 余量拟合保留了远段误差的连续尺度；即使二值预测相同，也能看到离边界的幅度误差。
- 如果近段二值全是同一类，朴素分类器只能常值外推；这时训练点里缺少边界信息，比较重点应放在远段二值错误和余量误差是否同步恶化。
- 后续若要支撑 §8 的外推论证，需要在便宜场景里明确规定近段如何覆盖边界附近，否则 0/1 基线会因为没有翻转样本而退化成常值。


## Artifact Index

- `planc/explore/data/probe1_energy_margin.csv`
- `planc/explore/data/probe2_authority_demand.csv`
- `planc/explore/data/probe3_extrapolation.csv`
- `planc/explore/plots/probe1_margin_vs_D.png`
- `planc/explore/plots/probe1_margin_timeseries_examples.png`
- `planc/explore/plots/probe2_authority_demand_vs_r.png`
- `planc/explore/plots/probe3_extrapolation.png`
- Local-only DataFlash logs: `planc/logs/bbs_probe*.BIN` (ignored; CSV/PNG/markdown artifacts above are committed)
