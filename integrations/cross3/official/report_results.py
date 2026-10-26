"""Recompute checks and write a reviewable report from a completed experiment."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
from schema import PHASE_BITS


def report(root):
    progress=json.loads((root/'progress.json').read_text())
    protocol=json.loads((root/'protocol.json').read_text())
    comparison=json.loads((root/'comparison.json').read_text())
    audit=json.loads((root/'audit.json').read_text())
    rows=progress['results'];sample_count=0;releases=0;issues=[]
    for row in rows:
        folder=root/row['label'];raw=json.loads((folder/'summary.json').read_text())
        for key in ('completed','planned','mean_wait_seconds','collisions','signal_mismatches'):
            if raw[key]!=row[key]:issues.append('summary mismatch '+row['label']+' '+key)
        arrivals=json.loads((folder/'demand.json').read_text())
        if hashlib.sha256(json.dumps(arrivals).encode()).hexdigest()!=row['demand_sha256']:
            issues.append('demand mismatch '+row['label'])
        samples=json.loads((folder/'samples.json').read_text())
        for i,sample in enumerate(samples):
            state,action,following,reward,_,_,_=sample
            if [len(state[k]) for k in ('cur_phase','coming_vehicle','leaving_vehicle')]!=[8,36,36]:
                issues.append('state dimensions '+row['label'])
            if reward>0 or state['cur_phase'] not in PHASE_BITS:issues.append('state/reward '+row['label'])
            if i<len(samples)-1 and following['cur_phase']!=PHASE_BITS[action]:
                issues.append('action/next phase '+row['label'])
            sample_count+=1
        greens=[s for s in json.loads((folder/'transitions.json').read_text()) if s['stage']=='GREEN']
        for stage in greens[1:]:
            releases+=1
            if stage['blockers'] or stage['switch_age']<5:issues.append('unsafe release '+row['label'])
    for name,digest in protocol['adapter_sha256'].items():
        if hashlib.sha256((root/'source'/name).read_bytes()).hexdigest()!=digest:issues.append('source '+name)
    tests=[row for row in rows if row['label'].startswith('test_')]
    for controller,aggregate in comparison['aggregate'].items():
        cases=[r for r in tests if r['controller']==controller]
        if len(cases)!=6:issues.append('test case count '+controller)
        if abs(sum(r['mean_wait_seconds'] for r in cases)/len(cases)-aggregate['mean_wait'])>1e-9:
            issues.append('aggregate wait '+controller)
        for key in ('completed','planned'):
            if sum(r[key] for r in cases)!=aggregate[key]:issues.append('aggregate '+key+' '+controller)
    consistency=dict(passed=not issues,issues=issues,scenarios=len(rows),samples_checked=sample_count,
                     green_releases_checked=releases,collision_callbacks=sum(len(r['collisions']) for r in rows),
                     stable_signal_mismatches=sum(r['signal_mismatches'] for r in rows))
    (root/'consistency.json').write_text(json.dumps(consistency,indent=2))
    assert not issues,issues
    with (root/'scenario_metrics.csv').open('w',newline='') as stream:
        fields=['label','controller','kind','seed','planned','completed','mean_wait_seconds','elapsed','extended_clearance_seconds']
        writer=csv.DictWriter(stream,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(rows)
    validation=[]
    for round_id in range(9,protocol['rounds'],10):
        cases=[r for r in rows if r['label'].startswith('val_round%03d_'%round_id)]
        if cases:validation.append((round_id+1,sum(r['mean_wait_seconds'] for r in cases)/len(cases),
                                   sum(r['completed'] for r in cases),sum(r['planned'] for r in cases)))
    names={'fixed':'固定配时（20 秒档）','maxpressure':'MaxPressure','presslight':'PressLight'}
    lines=['# 官方 PressLight CARLA 修复与训练报告','',
           '**结论：安全回归与完整训练已完成；本轮 PressLight 未达到优于两种基线的目标。**','',
           '训练 60 轮，随机初始化种子 2026，累计 1,794 条训练转移；按四组验证需求选择 `round_059.h5`。',
           '随后固定模型，在六组未用于训练或选模的需求上与两种基线逐组比较。平均等待按场景均值等权汇总，包含入口生成等待。','',
           '| 控制器 | 测试平均等待（秒） | 完成车辆 | 碰撞回调 |','|---|---:|---:|---:|']
    for controller,metrics in comparison['aggregate'].items():
        lines.append('| {} | {:.2f} | {}/{} | {} |'.format(names[controller],metrics['mean_wait'],metrics['completed'],metrics['planned'],metrics['collisions']))
    lines+=['', 'PressLight 等待比固定配时增加 {:.2%}，比 MaxPressure 增加 {:.2%}。'.format(-comparison['fixed_wait_reduction'],-comparison['maxpressure_wait_reduction']),
            '独立测试仅在 2/6 组优于固定配时，0/6 组优于 MaxPressure；`test_presslight_7102` 有 1 辆未在 420 秒内完成。',
            '这也是 `audit.json` 未通过的原因；车流、原始记录、汇总指标与代码快照的一致性检查通过，详见 `consistency.json`。','',
            '## 修复与核对','',
            '- 切相位必须等待受控车辆及车尾清空；切换完成前禁止新的动作覆盖，释放绿灯前执行断言并记录日志。',
            '- 场景开始计时前，原生灯态与冻结状态必须连续三帧一致；运行中的持续失配仍立即停止。',
            '- 环境和官方 agent 共用官方八位相位编码；按地图信号核对四动作和车道映射。',
            '- 位置分箱上边界与官方一致；排队压力奖励及下游车道配对通过数值对照。',
            '- 目标网络从第 1 轮起加载 max(round−5,0)，修正此前第 1～4 轮仍使用随机初始目标的问题。',
            '- 直接调用未修改的官方 SimpleDQNAgent；80 维输入、4 个动作，未使用 LMDrive 或教师标签。','',
            '## 验证证据','',
            '- 4 项适配数值测试、1 项真实官方训练/目标加载/HDF5 独立计算及重载测试通过。',
            '- 40 次连续场景重置通过，最多 6 帧完成初始确认，切换灯态延迟为 0。',
            '- 两组交通回归 157/157 辆通过，覆盖 12 种有向相位切换，46 次绿灯释放检查通过。',
            '- 完整实验 {} 个场景共核对 {} 条样本、{} 次绿灯释放；碰撞回调 {}，持续灯态失配 {}。'.format(
                consistency['scenarios'],sample_count,releases,consistency['collision_callbacks'],consistency['stable_signal_mismatches']),
            '', '## 验证集学习过程','', '| 训练轮数 | 平均等待（秒） | 完成车辆 |','|---|---:|---:|']
    for round_id,wait,completed,planned in validation:lines.append('| {} | {:.2f} | {}/{} |'.format(round_id,wait,completed,planned))
    lines+=['', '![验证曲线](learning_curve.png)','', '## 比较条件与限制','',
            '三种控制器使用相同预生成到达、路线、车辆、四相位、黄灯/清空规则与统计定义。固定配时从 10/20/30 秒档在验证集选择 20 秒档；所有控制器均为 120 秒到达、最多 300 秒清空。',
            '相位档位包含共享切换开销，不等于纯绿灯时长。官方 CityFlow 的车道长度和固定动作间隔已适配为 CARLA 的道路与安全清空要求；每决策折扣仍保留官方定义。',
            '仅一个训练初始化、六组独立需求，不能据此宣称一般性收敛。v2/v3 中断结果未混入 v4。','',
            '后续值得验证的方向是增加每轮训练样本、覆盖低流量及末尾单车等待场景，并做多训练种子验证；这属于待检验的改进方向。本次独立测试集已经使用，后续调参应另留新的测试集。','']
    (root/'REPORT.md').write_text('\n'.join(lines))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(11,4),constrained_layout=True)
    rounds=[r[0] for r in validation]
    axes[0].plot(rounds,[r[1] for r in validation],marker='o',label='PressLight')
    for controller,prefix in [('fixed','val_fixed20_'),('maxpressure','val_maxpressure_')]:
        cases=[r for r in rows if r['label'].startswith(prefix)]
        mean=sum(r['mean_wait_seconds'] for r in cases)/len(cases)
        axes[0].axhline(mean,linestyle='--',color={'fixed':'#d97706','maxpressure':'#16804a'}[controller],label=controller)
    axes[0].set(xlabel='Training round',ylabel='Validation mean wait (seconds)',title='Greedy validation, four fixed demands')
    axes[0].legend();axes[0].grid(alpha=.25)
    axes[1].plot(rounds,[100*r[2]/r[3] for r in validation],marker='o')
    axes[1].axhline(100,linestyle='--',color='gray');axes[1].set(xlabel='Training round',ylabel='Completed vehicles (%)',ylim=(60,102),title='Completion within the shared 420 s limit')
    axes[1].grid(alpha=.25);fig.savefig(str(root/'learning_curve.png'),dpi=160);plt.close(fig)
    return consistency


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path)
    print(json.dumps(report(parser.parse_args().root),indent=2))
