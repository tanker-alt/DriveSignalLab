CARLA Cross3 交付说明 — 2026-10-23 v4
====================================

一、环境与包内容

已验证 Windows、Python 3.7.9、CARLA Python API 与服务端均为 0.9.15。
先自行启动 CARLA 可见窗口，默认连接 localhost:2000。
以下命令均在解压后的工程根目录运行，python 必须指向安装了 CARLA 的环境：

  python -m pip install -r requirements.txt
  python -B -m Toolkits.CollectionTools.selfcheck --server

包内不含 CARLA 安装、虚拟环境、实验原始数据、截图、历史工作记录或任何 .md 文件。
只附默认 Cross3 地图、必要 Python 模块和配置、此说明及文件哈希清单。
topology_validation.json 是 view.py 加载地图所需的拓扑检查文件，因此保留。
不含其他地图与地图生成器；使用本包提供的 XODR 无需 SUMO、Unreal 编辑器或生成环境。

二、地图：先加载一次，再选择一种控制流程

  python -B Toolkits/JunctionToolkit/view.py Maps/MapDataset/maps/cross3 --no-photos

默认地图 cross3_beautified，单个十字路口，junction_id=10。
每个方向三条进口车道：-1 左转、-2 直行、-3 右转；四向共 12 盏独立原生灯。
view 恢复道路外观和官方杆架。直接 generate_opendrive_world 或四相位 --load
仅加载道路和原生灯，不能替代 view 的外观恢复步骤。
已有车辆、传感器或同步实验时不要加载地图；加载器会拒绝覆盖活动实验。

两套控制流程是互斥选项，不能同时启动。新版四相位没有替换原控制器。

三、流程 A：原有逐灯循环放行 + 原采集器 + 随机车辆

控制器：control_junction_phases.py
含义：按 phase_slot 顺序一次只放行一盏灯。
在当前逐车道 Cross3 上，意味着一次只放行一条进口车道，不是南北同时放行。
默认每灯 12 秒绿、3 秒黄、2 秒全红；12 灯一轮 204 仿真秒，启动另有全红。
右转灯也参与逐灯循环，不采用常绿策略。

终端 A：
  python -B control_junction_phases.py --collect --collisions

出现 Frame collection output 后，在终端 B：
  python -B spawn_junction_traffic.py --vehicle-blueprint random --duration-seconds 304

默认每方向 3 辆、共 12 辆，按 traffic.json 选择混合车型。
控制器负责唯一 tick 和逐帧采集；生成器不推进世界。
等待终端 B 清理并退出，再在终端 A 按 Ctrl+C，保存采集并恢复设置。
不要在控制器接管后重设 TM 随机种子，否则可能重置信号网络。

逐帧输出：evaluation/outputs/<run_id>/；确切路径由控制器打印。
可离线汇总（替换路径占位符）：
  python -B -m evaluation.summarize <run目录> --spawn-manifest <本轮spawn_manifest文件>

run_experiment.py 是旧流程的可选自动验收入口，不是新版四相位入口。
旧混合车型运行曾仅记录 10/12 完成，不能据此保证任意车流全部通过。

四、流程 B：本次新增四相位 + 默认右转常绿

控制器：control_four_phases.py
顺序：南北直行 → 南北左转 → 东西直行 → 东西左转。
同一相位的两个相对进口同时放行，其他直/左车流红灯。
默认直行绿灯 20 秒、左转绿灯 15 秒；每相位后黄灯 3 秒、直左清空 2 秒。
全部时间为仿真秒；启动先进行直左清空，之后循环。

仅控制信号灯，不生成车辆：
  python -B control_four_phases.py run

自定义配时/有限周期示例：
  python -B control_four_phases.py run --straight-seconds 20 --left-seconds 15 --yellow-seconds 3 --all-red-seconds 2 --cycles 5

默认 --cycles 0 持续运行，Ctrl+C 退出。
默认 --right-turn always-green：四个右转灯在绿灯、黄灯和直左清空阶段均绿色。
因此清空阶段日志为 MAIN_RED，而非所有车道全红；参数名仍为 --all-red-seconds。
可选 --right-turn with-straight 恢复右转随同方向直行；--right-turn red 始终禁行右转。
右转常绿是实际原生信号灯绿色，不是配置车辆忽略红灯。
退出时恢复接管前灯态和配时；常绿策略只在控制器运行期间生效。

本入口默认等世界帧，不改变 TM 或世界同步设置。
只有在已配置同步世界且本进程为唯一 tick 所有者时才用 --tick。
逐帧灯态 CSV 写入 evaluation/outputs/run_logs/；切换确认帧标为 APPLYING_*，
最多等待 10 帧，确认后开始阶段计时；稳定阶段灯态不一致则报错退出。

注意：本入口尚未接入原 FrameCollector 的多灯同时放行语义，不支持 --collect。
不要把流程 A 的单活动灯采集结果解释为本四相位结果。

五、推荐观看新版车辆演示：单命令自动控制与生成

先完成第二节 view 加载。保持无其他车辆/传感器，不要另启动任何信号控制器：

  python -u -B demo_four_phase_traffic.py

自动生成 12 辆 Model 3，金色直行、蓝色左转、白色右转，遵守原生灯并禁自动变道。
固定 0.05 秒步长，按现实时间节奏运行两个周期，唯一 tick 由本脚本持有。
演示专用默认配时为各相位绿灯 15 秒、黄灯 3 秒、直左清空 3 秒。
默认右转常绿；车辆摆放准备期先保持红灯。与独立控制器的默认绿灯时长不同。

只演示一轮：
  python -u -B demo_four_phase_traffic.py --cycles 1

加速：
  python -u -B demo_four_phase_traffic.py --cycles 1 --speed 3

输出到 evaluation/outputs/four_phase_traffic/<时间戳>/：
  summary.json：逐车路线完成、红灯停止观察、原始碰撞回调、恢复状态。
  signals.csv：逐帧期望/实际灯态。
  vehicles.jsonl：车辆采样轨迹。
正常结束后清理本轮车辆和传感器，恢复灯态、TM 和世界设置。
避免强制终止进程或直接关闭终端，以便 finally 执行清理。

六、验证边界与接口

新版地图 12 条拓扑转向及 12 灯原生车道归属校验通过。
四相位、右转常绿固定轿车演示：12/12 完成，1753 帧一致，右转非绿帧 0，
原始碰撞回调 0。原始实验数据按要求不包含在此包中。
这些结果不证明持续到达车流、混合车型、行人、任意变道场景无冲突。
reference_only 仍是参考停止断面，不是实测停止线；尚无完整闯红灯或综合评分验收。

外接控制器应按 maps/cross3/manifest.json 的 signal ID / road_id / incoming_lanes
重新查找原生 actor；重载后 actor ID 会变化，不能写死 4 个灯或固定 actor ID。
新版控制器通过 arms/movements 推导四相位分组，并核对地图指纹及 affected lanes。
默认 profile 在 map_profiles/generated_cross3/profile.json，active 指针为相对路径。
只允许一个信号控制者及一个同步 tick 所有者；包未提供任意外部控制模块的通用插件接口。

七、交付校验与再次打包

DELIVERY_MANIFEST.json 记录所附文件的相对路径、字节数和 SHA-256。
包内 Python/config 全部来自本次工作区；旧 v3 压缩包未覆盖。
重打包：python -B -m Toolkits.CollectionTools.build_delivery --name <新的英文数字版本名>
仅按白名单打包，缺少必要文件即报错；所有 .md、历史 outputs 与截图均排除。
