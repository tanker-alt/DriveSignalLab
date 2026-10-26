# DriveSignalLab 协作说明

本项目基于 Xiaomi MindDrive，包含灯控原型、CARLA/LMDrive 集成、Cross3 实验和可替换 agent 接口。
代码共享与本机资源边界见 [CODE_SHARING.md](docs/CODE_SHARING.md)。
原项目说明见 README.md，使用和分发时保留 LICENSE 及原作者声明。

## 开发流程

每位成员使用自己的克隆目录、Conda 环境和 Git 分支；不要在共享目录中同时切换分支。

```bash
git clone https://github.com/tanker-alt/DriveSignalLab.git
cd DriveSignalLab
git switch -c feat/traffic-light-agent
# 修改并检查代码后，只暂存本次任务文件
git add path/to/changed_file
git commit -m "Add traffic light agent interface"
git push -u origin feat/traffic-light-agent
```

在 GitHub 创建 Pull Request，由另一位成员审核后合并。任务通过 Issues 记录负责人和验收标准。
建议仓库管理员设置 main 分支保护，要求 PR 和至少一人审核；这些设置尚未自动开启。

## 推荐分工

- 仿真与信号控制：CARLA 生命周期、同步步进、车道/停止线映射、交通状态采集、红绿灯 Agent。
- 车辆与评估：车辆 Agent 适配、MindDrive 接入、指标定义、日志分析和实验报告。
- 公共接口变更先共同确认：观测格式、相位请求、车辆控制、事件格式、仿真时间及单位。

## 环境与文件

安装参考官方 README.md 和 docs/INSTALL.md。已在现有服务器验证的核心组合是 Python 3.8、
PyTorch 2.4.1+cu118、CUDA Toolkit 11.8、GCC 11、FlashAttention 2.7.0.post2。
项目使用定制 MMCV，需要在各自环境编译，不要安装通用 mmcv/mmcv-full 覆盖。

权重、数据、Conda 环境、编译产物、运行日志和密钥不上传 GitHub。
data/routes 中的路线定义可以共享；新增大型数据目录时先补充 .gitignore。
共享配置不要硬编码某位成员的 /home 或 /mnt 路径，使用环境变量或本地配置。
当前环境验证不代表已完成权重推理或 CARLA 闭环评测。

## 上游更新

```bash
git remote add upstream https://github.com/xiaomi-mlab/MindDrive.git
git fetch upstream
```

由一名成员在独立分支处理上游合并、验证后提交 PR，不直接覆盖团队 main。
