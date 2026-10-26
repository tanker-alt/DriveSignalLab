# 安装、自检与最短运行

已验证环境：Windows x64，Python 3.7.9，CARLA 客户端/服务器均为 0.9.15。
Python 3.7 已结束上游维护，这里保留它用于复现当前 CARLA wheel；升级解释器需单独验证 wheel 与项目兼容性。
核心采集使用标准库与 CARLA；绘制 topology PNG 额外使用 NumPy 1.20.0、Matplotlib 3.4.2。
CARLA 可执行服务器和地图不包含在代码包中，必须另行安装并启动。其他系统/版本尚未现场验收。

## 安装（在解压目录执行）

```powershell
py -3.7 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
# 仅需 topology PNG 时安装：
.\.venv\Scripts\python.exe -m pip install -r requirements-visualization.txt
```

若软件源没有匹配的 carla==0.9.15 wheel，使用 CARLA 0.9.15 对应 PythonAPI 分发的匹配
Windows x64、CPython 3.7 wheel：python -m pip install <实际wheel路径>。不要安装其他版本来绕过错误。
下文 python 需指向此环境，例如先运行 .\.venv\Scripts\Activate.ps1；也可始终用完整解释器路径，
不要求修改 PowerShell 执行策略。未安装 Python 3.7 时先安装解释器；py -3.7 不是下载命令。

## 自检

```powershell
python -B -m tools.selfcheck
python -B -m unittest discover -s evaluation/tests -v
# CARLA 已启动后，只读连接检查：
python -B -m tools.selfcheck --server
```

本地自检无需服务器；测试仍需要导入 CARLA 客户端，不是完全无 CARLA 依赖。
--server 不改世界/信号/地图，报告版本及当前地图。默认 localhost:2000，可指定 --host/--port。
连接超时请检查服务器启动状态、端口和网络；版本不匹配先修正环境。

## 最短工作流

1. 启动 CARLA，手动加载需要的地图。
2. 运行 python -B -m map_tools，选择输出中的路口 ID；用 --junction-id ID --draw --focus 查看。
3. 单独终端运行控制器作为唯一 tick 所有者。
4. 另一个终端生成测试车，车队退出清理后再停止控制器。

```powershell
# 以下 1427 仅为已验证的 Town05 示例，应替换为当前地图查询出的有信号路口。
python -B control_junction_phases.py --junction-id 1427 --collect --collisions
python -B spawn_junction_traffic.py --junction-id 1427 --vehicles-per-approach 6 --duration-seconds 180
```

--vehicles-per-approach 按每个信号灯分配总数，非每条车道；当前 Town05 示例生成 24 车。
默认混合车型，不包含两轮；2 m 是初始车身净距。生成器等待控制器推进，不应单独承担 tick。
控制器 Ctrl+C 后出现 interrupted 是人工终止标记；运行失败为 failed，需检查日志。
运行期间禁止再次设置 TM 随机种子；需要种子时在控制器启动时传 --tm-seed。

原始输出位于 evaluation/outputs/<run_id>。排除摆放期后回放：

```powershell
python -B -m evaluation.summarize evaluation/outputs/<run_id> --spawn-manifest <本轮spawn_manifest路径>
```

尖括号内容需替换，回放写新 analysis 目录。独立监听 evaluation.run_collection 不 tick，
不能承诺逐帧完整。无信号路口的完整进口指标尚未支持，不承诺任意地图已验收。

## 交付包

python -B -m tools.build_delivery 生成新的 dist/<名称> 目录、ZIP 和 SHA256 文件清单。
仅选当前源码/测试/配置/工具及全部 Markdown；排除旧脚本、原始实验 CSV/JSONL、缓存和本机日志。
历史 Markdown 可能链接到被排除的原始实验文件，属于发送方留存证据，不是接收方运行依赖。
不会删除本地历史数据，不会向任何人发送文件。接收方应先执行自检，再连接自己的服务器。
