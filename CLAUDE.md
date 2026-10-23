# CLAUDE.md

本文件为在此仓库工作的 Claude Code 提供上下文，内容整理自 2026-10-23 的一次设计审查和 EasyEDA 联调会话。

## 约定
- 用中文交流；代码、命令、料号、网络名保持原文。
- 只有用户明确要求时才 commit / push。
- 修改 EasyEDA 工程前，先确认有一份最新的 `.epro2` 导出作为备份。

## 项目概况
"FOC Driver Comet"：基于 SimpleFOC 的 DIY 无刷电机 FOC 驱动板，计划用于机械臂、机器狗等机器人关节（见 README）。

| 路径 | 内容 |
|---|---|
| `1.Hardware/` | STEP / SolidWorks（行星减速执行器、单腿装配）、Gerber_PCB4、旧 Altium 压缩包 |
| `2.Firmware/FOC-Driver/` | PlatformIO + Arduino-ESP32 + SimpleFOC 2.2.1，任务结构源自 SmartKnob |
| `4.Docs/` | BLDC 基础资料 |
| `ProPrj_FOC_Driver_comet_2026-10-23.epro2` | EasyEDA Pro 工程导出（编辑器 3.2.209），未纳入 git |

## EasyEDA 工程
工程 `FOC_Driver_comet` 位于 Personal 工作区，工程 uuid 为 `220d407a180a470891f77ee4272301bd`，包含 4 块板：

| 板 | 原理图 / 页 | PCB | 说明 |
|---|---|---|---|
| **FOC_Driver_v0.2**（当前） | Schematic_v0.2 / P1 | PCB_v0.2 | 42×42 mm 四层，JLC04161H-7628（外层 1 oz、内层 0.5 oz），最后修改 2024-06-04 |
| FOC_Driver_v0.1 | Schematic2 / P1 | PCB4 | 四层，带 OLED 和板载 AS5600 |
| Comet_Main | Schematic1 / Main_board | PCB8 | 两层，带 OLED |
| Comet_Sensor | Schematic3 / P1 | PCB3 | AS5600 小板，I²C 上拉在这块板上 |

v0.2 的文档 uuid（取自导出文件，调用 API 前请核对）：
- 原理图 `26599f8431034b5c8149f318a7951f60`
- 页 P1 `1d1d3e9457b848c19f6057387d65faa2`
- PCB `ddeff2e482a04244941cbf58af31633f`

v0.2 架构：
- 主控：ESP32-WROOM-32U-H4
- 栅极驱动：EG2133，3PWM（HIN 与 LIN# 并联，内部死区约 100 ns）
- 功率管：6×WSD3066DN33（30 V）
- 电流采样：2×INA240A2，两相 inline，10 mΩ 分流电阻
- 电源：TPS54331（VIN→5.08 V）+ AMS1117-3.3
- 接口：CP2102 + USB-C、SN65HVD232（CAN）

### 离线解析 `.epro2`
不开 EDA 也能做完整审查：
- `.epro2` 是 zip 包，内含 `project2.json`、`IMAGE/*.webp` 和一个 `.epru` 文本文件。
- `.epru` 每行一条记录，格式为 `{"type":T,"ticket":N,"id":ID}||{data}|`。`{"type":"DOCHEAD"}` 行开始一个新文档，`docType` 取值有 FOOTPRINT / SYMBOL / DEVICE / BOARD / SCH / SCH_PAGE / PCB / CONFIG。
- ATTR 记录通过 `data.parentId` 挂到元素上，包括 Designator、Device、Value、Footprint；符号文档里用 `Pin Name` / `Pin Number`。DEVICE 的 `META.attributes` 里有 MPN、`Supplier Part`（立创编号）和 Value。
- PCB 部分：
  - 单位是 mil。
  - `PAD_NET` 直接给出网表，id 格式为 `["PAD_NET",元件id,焊盘号,…]`。
  - LINE / VIA / POUR 带 `netName` 和 `layerId`：1 顶层、2 底层、15 Inner1、16 Inner2、11 板框。
  - 内电层网络在 `LAYER_FILL` 里，叠层在 `LAYER_PHYS` 里。
- 本机的 `python` 是微软商店占位符，解析用 Node（v24）。需要 Python 时用 `uv run --no-project --with <包> python -I`；`-I` 会忽略 `PYTHONIOENCODING`，要在脚本里自己设置 UTF-8 输出。

## v0.2 设计审查结论（2026-10-23）
原理图框架可以保留，**功率级和 PCB 布局需要大改**。现在能工作，是因为固件只在 12 V 下驱动 11.2 Ω 的云台电机（`voltage_limit = 5`，相电流约 0.45 A）。按铜皮和过孔估算，连续相电流只能到 1.5–2 A 左右。

### 🔴 必须修
1. **BOM 料号绑错**。JLC 贴片按料号贴，10 kΩ 的栅极电阻会让 MOSFET 长时间处在线性区并发生直通，必烧：

   | 位号 | 标注值 | 现在绑定的料号 | 改为（已在 EDA 库核实） |
   |---|---|---|---|
   | R8、R16–R20（栅极电阻） | 10Ω | C25804（10kΩ） | **C22859** 厚声 0603WAF100JT5E 10Ω，基础库 |
   | R1、R2 | 12kΩ | C25804 | **C22790** 0603WAF1202T5E，基础库 |
   | R3 | 200Ω | C25804 | **C8218** 0603WAF2000T5E，基础库 |
   | C1、C2、C16–C19（VIN 去耦） | 10uF | C30926（100nF） | **C194427** 村田 GRM188R6YA106MA73D 10µF 35V X5R，扩展库（备选：C96446 三星 25V，基础库） |

2. **功率通路太细**：
   - 每相只有 1 个 0.3 mm 过孔。
   - VIN 经 1 个过孔走 Inner1（0.5 oz）上 1 mm 宽的线到上管，每个上管的漏极也只靠 1 个过孔。
   - AGND（下管源极）只通过 R15（0603 的 0 Ω）回到 GND。
   - 相线只有 0.4–0.6 mm，分流电阻是 0603 封装。
3. USB VBUS 和 Buck 5V（VCC_5V）直接相连，会双向倒灌：5V 会经 TPS54331 上管的体二极管反灌到 VIN。
4. USB-C 的 CC1（4 脚）和 CC2（10 脚）悬空，需要各加一颗 5.1 kΩ 下拉。
5. CP2102 的 VDD（6 脚）悬空，需要加去耦电容（容值以 datasheet 典型电路为准）。
6. ESP32 的 EN 没有 RC（现在只有 12 kΩ 上拉），需要加 1 µF。

### 🟠 应改进
- **栅极驱动**：EG2133 的 VCC 用的是共用的 5V，上管 Vgs 只有约 4.5 V。数据手册 V1.0 没有 UVLO 参数，MOSFET 也没有 G-S 下拉电阻。
- **Buck 布局太分散**：PH→D1 16.5 mm，PH→L1 13.6 mm，BOOT 电容 12.7 mm，输入电容 23–26 mm；PH 网络长约 26 mm，打了 7 个过孔。
- **电流采样不是 Kelvin 接法**：IN+ 取在电机端子焊盘，IN− 取在过孔顶端，误差约 30–50%。量程只有约 ±1.6 A（ESP32 ADC 线性区为 0.15–2.45 V）。固件目前还没用电流采样。
- **母线去耦不足**：上管附近实际只有 6×100 nF。电解电容 C33/C34 在 12 mm 外的另一条支路上；料号 CK1E107…（C9900016221）按命名规则应为 25 V，标注却写 35 V，需要核对。
- **保护和接口缺失**：
  - 没有 TVS、防反接、保险丝、母线电压检测和 NTC。
  - CAN 没有 120 Ω 终端选项，接口没有 GND，也没有 ESD 保护。
  - ESP32 的 3V3 脚 14 mm 以内没有去耦电容。
  - AS5600 走 I²C，又慢又只有 12 位。
- **大量元件没有绑定料号**（C3–C15、C20–C23、R9–R15、R21/R22、R24–R27 等），贴片前需要选料。

### 旧版板子的致命问题
- **Comet_Main**：电解电容 C33/C34 极性接反；Buck 电感是 0603 封装。
- **v0.1**：钽电容 C30/C32 极性接反；Buck 电感同样是 0603 封装，这就是 README 里"电感烧了"的原因。v0.2 已改用 SWPA6045（Isat 4.3 A）。

### v0.3 改版方向
- **功率级重新布板**：
  - 40–60 V MOSFET（DFN5×6 封装）。
  - 外层 2 oz，功率走外层大面积铺铜，每次换层至少 6–10 个过孔。
  - 每个半桥旁边放 ≥10 µF/50 V 的 X7R 陶瓷电容，再加固态电解。
  - 1–3 mΩ 的 2512 分流电阻，配 Kelvin 走线。
  - MOSFET 贴壳散热。
- **栅极驱动**：给 EG2133 单独供 10–12 V；或者换 DRV8323RS（自带 UVLO、VDS 过流保护和 3 路 CSA，集成 Buck）。
- **MCU**：继续用 SimpleFOC 的话可以换 ESP32-S3，它有原生 USB，能省掉 CP2102 和自动下载电路。需要更高的电流环带宽时用 STM32G431/G474。
- **编码器**：板载 SPI 磁编码器（MT6835 / MT6701 / AS5047P），放在板子中心。
- **参考设计**：FOCn（ESP32-S3 + DRV8323）、SimpleFOC DriveShield、Ben Katz 电机驱动、ODrive v3.6。

## 固件要点（`2.Firmware/FOC-Driver`）
- 引脚与 v0.2 一致：
  - PWM：IO32 / IO33 / IO25
  - I²C：SDA=IO19，SCL=IO18
  - 电流采样：IO34 / IO35
  - CAN：TX=IO5，RX=IO4
- `BLDCDriver3PWM(32, 33, 25, 22)` 里的使能脚 22 在板上没有连接（EG2133 本身没有使能脚）。
- 当前配置：`voltage_power_supply = 12`、`voltage_limit = 5`，7 对极、11.2 Ω，电压力矩模式。SimpleFOC 版本仍是 2.2.1。

## EasyEDA API 桥接（让 Claude Code 直接操作 EDA）
- **组成**：
  - EasyEDA Pro 桌面端
  - 扩展 `run-api-gateway`，需要开启 "Allow interact with external"
  - skill `easyeda-api-skill`：已装在 `~/.claude/skills/easyeda-api-skill`，版本 release 1.1.60，唯一依赖是 `ws`
- **启动桥接（后台运行）**：`node ~/.claude/skills/easyeda-api-skill/scripts/bridge-server.mjs`。它监听 `127.0.0.1:49620`，端口被占用时顺延到 49629。启动后在 EDA 顶部的 API Gateway 菜单点 Reconnect。
- **检查连接与执行代码**：
  - 检查：`curl http://127.0.0.1:49620/health` 和 `/eda-windows`。
  - 执行：`POST /execute`，body 为 `{"code": "return await eda.dmt_Project.getCurrentProjectInfo();"}`。
  - 代码较长时写进文件，用 Node `fetch` 发送，避免 shell 转义问题。
- ⚠️ **安全**：`/execute` 没有任何鉴权，CORS 还设成了 `*`。只在需要时运行，用完就停。
- **API 约定**：
  - PCB 单位是 mil，原理图单位是 0.01 inch。
  - 层、类型等参数用枚举，不要写裸数字。
  - 改图元用 `get → toAsync → setState_* → done`，或者用 `modify`。
  - 签名以 `references/` 里的文档为准，不要猜。
- **修 BOM 的方法**：
  - 用 `eda.sch_PrimitiveComponent.modify(id, {manufacturer, manufacturerId, supplier: 'LCSC', supplierId, otherProperty: {Value}})`，保留原来的元件和封装。
  - **不要删除后重建**，那样会丢掉 PCB 上已有的布局。
  - 改完先 `sch_Document.save()`，再在 PCB 里执行 `pcb_Document.importChanges()` 并保存。
- **已知问题**：通过 API 调用 `dmt_Project.openProject()` 会超时（30 s），之后所有请求都会卡住，但心跳正常，疑似 EDA 弹了对话框在等确认。解决办法：在 EDA 里手动打开工程和目标页面，再用 API 操作。

## 当前进度（2026-10-23）
- **已完成**：
  - v0.2 离线审查。
  - skill 安装和连通性测试：EDA 版本 3.2.209，能找到这个工程。
  - 替换料号已在 EDA 库中逐个核实（见上表）。
- **进行中**：修 BOM 卡在打开工程这一步，**EDA 里还没有做任何修改**。
- **下一步**：
  1. 手动打开工程和 Schematic_v0.2 的 P1 页。
  2. 跑一次 DRC 作为基线。
  3. 按上表修改料号并保存。
  4. 导入到 PCB_v0.2，再跑一次 DRC 对比。
  5. 在原理图里补上 🔴4–6 缺的元件；PCB 端需要手动放置和布线。

## AI 工具调研结论（2026-10）
- **首选**：Claude Code + EasyEDA 官方桥接（见上）；或者每次导出 `.epro2` 做离线审查。
- **只适合信号线**：DeepPCB（能读 EasyEDA Pro 文件）、Quilter（支持 KiCad/Altium，个人免费）。功率级仍然要对照参考设计手工布局。
- **需要换平台或面向企业**：Flux、JITX、CELUS、Circuit Mind。
- **KiCad 导入**：直接导入 `.epro2` 有已知问题（KiCad issue #22239），可以先从 EasyEDA 导出 Altium 格式再转。
- **常用非 AI 工具**：TI WEBENCH、LTspice、Saturn PCB Toolkit、SimpleFOC Studio。
