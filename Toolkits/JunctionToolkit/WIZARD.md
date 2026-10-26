> 已新增单个 T 型路口：向导类型选 `t`，或 `generate_map.cmd --type t --missing-arm north --name my_t`。详见 [T 型路口](T_JUNCTION.md)。原 cross/grid 用法保留。

# 地图生成向导

无需复制或手动编辑 JSON。在 CMD 执行：

```bat
cd /d D:\WindowsNoEditor\CarlaUE4\newProject\map_dataset\junction_toolkit
generate_map.cmd
```

按照提示填写地图名称、类型、车道数、道路长度、车道宽度、路口大小、人行道和转向分配。回车接受默认值。类型 `cross` 为单十字路口，`grid` 为规则网格（额外输入行数、列数、间距）。名称使用英文字母、数字、下划线或短横线。

确认后自动保存 `configs/generated/<名称>.json`，生成 `maps/<名称>/` 并做 CARLA 离线拓扑检查。已存在的名称会被拒绝；确认前取消不写文件。生成失败会保留配置和候选输出供排查，重试请换新名称。

生成不需要 CARLA 服务器。成功后可选择加载，默认不加载；加载需要 CARLA 已启动且没有正在运行的实验。加载不会启动信号控制器或车辆，不修改主项目的默认 Cross3 profile。

## 命令行生成

提供 --name 时直接按命令参数生成，无需交互；添加 --interactive 可继续使用向导。

```bat
generate_map.cmd --name my_cross --type cross --lanes 3 --arm-length 200
generate_map.cmd --name my_grid --type grid --rows 2 --columns 2 --spacing 344
generate_map.cmd --name my_cross4 --lanes 4 --turns "left;straight;straight;right"
```

更多选项：`generate_map.cmd --help`。`--load` 表示成功后直接加载 CARLA；不传就不会切换当前地图。命令参数的文件目录相对于工具包定位，不依赖调用时所在目录。

向导自身兼容 Python 3.7+。generate_map.cmd 使用 PATH 中的 python；本机已配置为 D:\Python37\python.exe。也可用完整路径运行：

```bat
D:\CarlaMapTools\generator-env\Scripts\python.exe -B wizard.py
```

生成器和检查器仍由 settings.json 指定两个独立 Python 环境。接收方须配置 generator_python（Python 3.12.3 + scenariogeneration 0.16.7）和 carla_python（Python 3.7.9 + carla 0.9.15），不能将两个环境混为一套。

## 当前能力与后续扩展

当前支持十字路口和同构规则网格，四方向使用相同进口车道数，出口数量与进口相同。向导显示这一限制，不提供尚未实现的选项。

默认从内侧到外侧转向分配：1 车道共享左/直/右；2 车道为左+直、右；3 车道为左、直、右；4 车道为左、直、直、右。可逐车道自定义，不能据此假定所有组合都已完成实车验收。

后续扩展目标：分别输入东、西、南、北的进口和出口车道数、转向组织；这需要同时扩展道路生成、连接和信号适配，不仅增加界面输入。T 字路口现已支持，详见 T_JUNCTION.md；环岛尚未开放。

新地图进入实验前必须准备与其指纹绑定的 profile，不能直接复用默认 Cross3 配置。多路口目前只支持生成、拓扑检查和已有直行冒烟检查，尚不支持完整多路口联合评估。

本次验证：单路口 12 条转向通过；1×2 网格 24 条转向、6 条接缝车道通过。交互取消、非法名称/参数、同名拒绝通过。此次没有自动切换用户正在查看的 CARLA 场景，也未对新候选执行实车评估。
