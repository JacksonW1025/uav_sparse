**Amendment 02（提名纠正，pre-data）**

1. oracle 可信性：经 Phase A 合成回归测试（数学正确）+ Stage 0 饱和对照（终端 0.32，与温和 0.027 干净区分）确认 oracle 对真实终端残余灵敏。原"正对照必须 fire 否则 oracle 不可信"一条误诊：MPC_ACC_HOR=0.5 未 fire 是因刺激无效，非 oracle 失灵。
2. 参数提名纠正：MPC_ACC_HOR 是加速建立旋钮（满杆加速上限），压低它会节流机动、降低残余，不能破坏刹停契约——经 PX4 文档/源码与 Stage0 实测共同确认，作废为合取旋钮。
3. 正确的刹车权限旋钮在减速侧：MPC_DEC_HOR_SLOW（近中位减速限）及速度→位置硬切处的 MPC_ACC_HOR_MAX；具体以当前 MPC_POS_MODE 下源码实际生效者为准。
4. 探针机动改为"先建立速度再回中"的可行型（区别于半杆的 eval234）。其余冻结量不变。
