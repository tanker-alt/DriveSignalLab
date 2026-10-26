"""Render the metric progress table as a standalone Chinese PNG (Pillow optional)."""
import argparse
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont


def render(source, output):
    groups = {key: [] for key in ('Safety', 'Comfort', 'Efficiency')}
    for line in Path(source).read_text(encoding='utf-8').splitlines():
        cells = [s.strip() for s in line.strip('|').split('|')]
        if len(cells) == 4 and cells[0] in groups:
            groups[cells[0]].append(cells[1:])
    font_path = 'C:/Windows/Fonts/msyh.ttc'
    title = ImageFont.truetype(font_path, 42)
    heading = ImageFont.truetype(font_path, 30)
    body = ImageFont.truetype(font_path, 24)
    small = ImageFont.truetype(font_path, 21)
    im = Image.new('RGB', (1800, 2200), '#edf2f7')
    d = ImageDraw.Draw(im)
    d.text((45, 30), '评估指标获取进度', font=title, fill='#15283e')
    d.text((45, 90), '更新：2026-10-25  |  智能体观测与实验评估已分离  |  待确认项不计算正式评分', font=body, fill='#46566a')
    colors = {'Safety': '#24577a', 'Comfort': '#3e6970', 'Efficiency': '#69548b'}

    def wrap(text, width, font):
        lines, current = [], ''
        for ch in text:
            if d.textbbox((0,0), current+ch, font=font)[2] > width:
                lines.append(current)
                current = ch
            else:
                current += ch
        if current:
            lines.append(current)
        return lines

    for index, (category, entries) in enumerate(groups.items()):
        x = 30 + index*590
        d.rounded_rectangle((x, 155, x+570, 1950), radius=15, fill='white')
        d.rounded_rectangle((x, 155, x+570, 223), radius=15, fill=colors[category])
        d.text((x+20,170), category, font=heading, fill='white')
        y = 245
        for name, status, detail in entries:
            for line in wrap(name, 530, body):
                d.text((x+20,y), line, font=body, fill='#1b2c41')
                y += 35
            status_color = '#21734b' if any(word in status for word in ('已实现', '已有实现', '已接入', '可复用')) else '#986519'
            for line in wrap(status, 530, small):
                d.text((x+20,y), line, font=small, fill=status_color)
                y += 29
            # Keep the figure readable; full requirements remain in the linked table.
            details = wrap(detail, 530, small)
            if len(details) > 3:
                details = details[:3]
                details[-1] = details[-1][:-2] + '…'
            for line in details:
                d.text((x+20,y), line, font=small, fill='#657285')
                y += 29
            y += 22
    d.text((45,1980), '观测：observations/agent_observations.jsonl  →  智能体候选输入（当前仿真真值）', font=body, fill='#233c55')
    d.text((45,2024), '评估：assessment/evaluation_metrics.jsonl + evaluation_report.json  →  后续分析', font=body, fill='#233c55')
    d.text((45,2073), '新增指标已补受控现场验证；实际VLA接入、TTC跨段及未实现项目仍待完成。', font=small, fill='#6f5c3b')
    d.text((45,2120), '受控验证不等于自然驾驶、多车型全面验收；零碰撞回调不代表绝对安全。', font=small, fill='#6f5c3b')
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    im.save(str(output))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', default='docs/EVALUATION_METRICS.md')
    parser.add_argument('--output', default='docs/EVALUATION_METRICS.png')
    args = parser.parse_args()
    render(args.source, args.output)
