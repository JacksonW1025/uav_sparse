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
