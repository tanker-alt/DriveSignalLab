# 团队代码更新与运行文件边界

## 本次集成

本次在已有 `feat/presslight-carla-adapter` 分支及 PR 上追加更新，包含统一 agent 接口、Cross3 场景、官方 PressLight 训练/评估代码、LMDrive overlay 更新，以及同门交付的两版 CARLA 工具源码。

同门源码按 `integrations/carla_handoff/upstream_20261021` 和 `upstream_20261023_v4` 隔离保存；来源、原包哈希与本地改动见同目录的 `SOURCE*.json`。原包的文件清单用于来源核对，Git 版本排除了历史 evaluation/outputs，因此不是原始交付包的逐文件镜像。

## 上传哪些文件

| 文件 | 是否上传 |
| --- | --- |
| Python/Shell 源码、测试、使用说明、依赖与版本说明 | 是 |
| 小型场景配置、路线、信号映射、Cross3 OpenDRIVE 地图定义 | 是，运行需要且无个人凭据 |
| 模型权重、训练 replay、数据集、实验输出、日志 | 否，放外部存储并记录版本/哈希 |
| CARLA 程序、Conda 环境、编译产物 | 否，每位成员自行安装 |
| 个人绝对路径、密钥、IDE 配置 | 不作为共享配置；使用环境变量或本地文件 |

因此不能把所有 CARLA 配置都删掉：CARLA 安装目录不上传，但定义同一个实验所需的信号、路线、地图和版本配置应共享。

## 本机路径

在启动前激活正确环境，或指定 `CROSS3_PYTHON`；官方训练还需 `PRESSLIGHT_PYTHON` 指向独立的官方 Python/TensorFlow/Keras 环境。插件 JSON 的 kwargs 支持 `${VARIABLE}` 和 `~` 展开。

```bash
export CROSS3_PYTHON=/path/to/lmdrive/env/bin/python
export PRESSLIGHT_PYTHON=/path/to/official/presslight/env/bin/python
export PRESSLIGHT_MODELS=/path/to/official/models
export LMDRIVE_AGENT_CONFIG=/path/to/local/lmdrive_config.py
export LMDRIVE_MODEL_OUTPUT=/path/to/local/model_logs
```

所有示例路径都需按自己的机器设置。原集成仍沿用工作区 `Drive/` 外部资源布局和 MDrive overlay，详见 `integrations/lmdrive/README.md` 和 `integrations/cross3/README.md`；这次上传不是包含运行环境和权重的独立安装包。历史交付说明中的作者本机路径仅作为历史记录。

## 以后如何合并

已合并的 PR 不反复追加。下次从最新 main 创建新分支，只提交本次改动，再创建新 PR；不要复制整目录覆盖远端，也不要 force push 主分支。

```bash
git switch main
git pull --ff-only origin main
git switch -c feat/next-model-adapter
# 修改、检查并按文件暂存
git add path/to/changed/files
git commit -m "Add next model adapter"
git push -u origin feat/next-model-adapter
```

同门代码如有独立 Git 仓库，后续应保留其来源与版本信息；更新隔离目录后验证调用方，避免静默覆盖本地兼容修改。
