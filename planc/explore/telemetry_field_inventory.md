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
