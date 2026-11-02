# 两车道纯三叉随机路网

```powershell
python standardization/JunctionToolkit/toolkit.py random-t --config CONFIG.json --out NEW_MAP_DIRECTORY
```

配置示例：

```json
{"name":"t_1x6","rows":1,"columns":6,"seed":42,"spacing_m":300,"arm_length_m":150,"roadside_props":false}
```

此入口强制采用现有两车道三叉模板。道路宽度、人行道、路缘等参数由 `templates/t_cross2/profile.json` 统一提供，美化仍由统一 `view.py` 自动加载：多路口默认简化，单路口默认完整，可选择 `--beautify complete`。

全部路口预设为三叉，每个路口从四个方向等概率提议一个缺失方向；整网不连通则重新抽样。1×n 中间路口只能缺北/南，n×1 中间路口只能缺东/西，端点不能删除连接其余路网的方向。最终分布受连通性约束，不保证四方向各占25%。seed 控制复现。

两入口车道从左到右：缺左转→直/右；缺直行→左/右；缺右转→左/直。出口分别显式连接 +1/+2 车道。不使用三车道混合路网的双左转或双直行编码。

可用 `missing_arms` 指定完整或部分方向（未指定部分采用 missing_arm，默认 north），固定方案也必须连通。`generate --config` 中 type=t 的多路口配置默认随机缺失；单路口维持原有固定方向默认值。

为兼容旧的纯三叉配置，`random-grid` 遇到 type=t、t_probability=1 或所有路口显式指定为 t 时，转入此两车道生成器。混合路网仍为三车道。旧地图文件不会自动改变，应在新目录生成验证后再替换。
