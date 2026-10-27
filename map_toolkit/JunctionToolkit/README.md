# CARLA 标准路网工具包

当前规范：urban_cross3_v1，目标 CARLA 0.9.15。标准十字和规则网格统一三进三出、左直右专用车道、独立灯控、双黄线、进口白实线、斑马线、圆弧路缘、人行道和可选路肩。

本目录是独立工具包。工具、配置、生成地图都保存在本工程；不会复制到其他主工程。新地图默认输出到 `../maps/<name>`。`generate --out` 和直接生成脚本均校验输出位于工具包工作区内。源工作区的已有验收地图保存在 `../validation`；GitHub 分支提供 `../examples`。

## 环境

生成环境使用 Python 3.12 + scenariogeneration 0.16.7；CARLA 客户端环境使用 Python 3.7 + CARLA 0.9.15，与服务器版本匹配。两者独立安装，依赖见 requirements-generator-lock.txt / requirements-carla.txt。锁文件记录本机已安装版本，其他平台安装可用性需另行核对。CARLA、Unreal、RoadRunner、Python 环境和商业模型不包含在仓库中。

工具自动寻找上级目录内的 generator-env / carla-env。找不到时使用当前 Python；跨环境用户创建 `settings.local.json`，指定 generator_python / carla_python 的本机解释器路径。该文件不提交 Git。必要时设置 workspace_root；默认工作区是工具包父目录，开发环境会识别含两个环境目录的工程根目录。

## 使用

从 JunctionToolkit 目录运行（Windows 也可将 `python toolkit.py` 替换为 `toolkit.cmd`）：

```powershell
python toolkit.py doctor
python toolkit.py generate --config configs/cross3.json --out ../maps/my_cross3
python toolkit.py generate --config configs/grid2x2.json --out ../maps/my_grid2x2
python wizard.py --type grid --rows 2 --columns 3 --name my_grid2x3 --shoulder
python toolkit.py view ../maps/my_cross3
```

生成/离线验证不切换 CARLA；view 明确加载地图，需 CARLA 已运行且没有其他实验。地图与配置名称不能覆盖已有输出。

GitHub 随附示例可用 `python toolkit.py view ../examples/cross3` 或 `../examples/grid2x2` 查看；示例保留原始地图指纹和验收数据。

## 验证与范围

```powershell
python -m unittest discover -s . -p "test*.py"
python toolkit.py validate ../examples/cross3
```

测试生成器需在生成 Python 环境运行；validate 由入口分派到 CARLA 客户端环境。在线物理检查需要加载对应地图后运行 drive_check.py / movement_check.py / standard_surface_check.py。

十字/网格为本轮规范；T 型和环岛代码保留为历史/实验功能，不属于统一规范。外部评估 experiment 默认禁用，必须明确配置 project 才会启用；普通地图生成和检查不依赖它。

详见 [道路规范](ROAD_STANDARD.md)、[验收范围与已知问题](STANDARD_VALIDATION.md)、[美化模板与批量生产方案](../BEAUTIFICATION_PIPELINE.md)。运行时 debug 标线和杆架尚不是烘焙地图资产；RoadRunner 回导和 Unreal 打包尚未实现。
