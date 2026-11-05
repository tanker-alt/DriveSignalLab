"""Interactive map configuration and generation; compatible with Python 3.7+."""
import argparse
import datetime
import json
import math
import re
import subprocess
import sys
from pathlib import Path
from layout import ARM_NAMES, arm_turns
from road_standard import normalize
from runtime_config import map_output

ROOT = Path(__file__).resolve().parent


def map_name(value):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}', value):
        raise ValueError('鍚嶇О椤讳负 1鈥?4 浣嶈嫳鏂囧瓧姣嶃€佹暟瀛椼€佷笅鍒掔嚎鎴栫煭妯嚎锛岄浣嶄负瀛楁瘝/鏁板瓧')
    if value.upper() in {'CON', 'PRN', 'AUX', 'NUL'} | {'COM'+str(i) for i in range(1,10)} | {'LPT'+str(i) for i in range(1,10)}:
        raise ValueError('璇ュ悕绉版槸 Windows 淇濈暀鍚嶇О')
    return value


def number(kind, low, high):
    def parse(value):
        result = kind(value)
        if not math.isfinite(result) or not low <= result <= high:
            raise ValueError('璇疯緭鍏?{} 鑷?{} 涔嬮棿鐨勬暟鍊?.format(low, high))
        return result
    return parse


def ask(label, default, parse=str):
    while True:
        try:
            return parse(input('{} [{}]: '.format(label, default)).strip() or str(default))
        except ValueError as exc:
            print('杈撳叆鏃犳晥锛?, exc)


def yes(label, default=False):
    def parse(value):
        if value.lower() in ('y','yes','鏄?,'1'): return True
        if value.lower() in ('n','no','鍚?,'0'): return False
        raise ValueError('璇疯緭鍏?y 鎴?n')
    return ask(label, 'y' if default else 'n', parse)


def turns(value):
    result = [part.strip().lower() for part in value.split(',')]
    if not result or len(set(result)) != len(result) or set(result)-{'left','straight','right'}:
        raise ValueError('浣跨敤 left銆乻traight銆乺ight锛涘涓浆鍚戠敤鑻辨枃閫楀彿鍒嗛殧锛屼笉閲嶅')
    return result


def config(args, interactive):
    name = args.name or 'map_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    name = ask('鍦板浘鍚嶇О', name, map_name) if interactive else map_name(name)
    kind = args.type
    if interactive:
        def choose(value):
            if value not in ('cross','grid','t'): raise ValueError('杈撳叆 cross锛堝崄瀛楋級銆乬rid锛堣鍒欑綉鏍硷級鎴?t锛圱 鍨嬶級')
            return value
        kind = ask('鍦板浘绫诲瀷锛歝ross / grid / t', kind, choose)
    rows, columns = (args.rows if args.rows is not None else (2 if kind == 'grid' else 1)), (args.columns if args.columns is not None else (2 if kind == 'grid' else 1))
    if kind == 'grid' and interactive:
        rows = ask('缃戞牸琛屾暟', rows, number(int,1,6))
        columns = ask('缃戞牸鍒楁暟', columns, number(int,1,6))
    if kind == 'cross':
        if not interactive and (args.rows is not None or args.columns is not None):
            raise ValueError('鍗曡矾鍙ｄ笉鎺ュ彈琛屽垪璁剧疆锛涢渶瑕佺粍鍚堣浣跨敤 --type grid')
        rows = columns = 1
    if kind != 't':
        c = dict(name=name, rows=rows, columns=columns, incoming_lanes=args.lanes,
                 lane_width_m=args.lane_width, sidewalk_width_m=args.sidewalk,
                 arm_length_m=args.arm_length, spacing_m=args.spacing if args.spacing is not None else 350.,
                 seed=args.seed, shoulder_enabled=args.shoulder)
        if args.radius is not None:
            c['junction_radius_m'] = args.radius
        if args.turns:
            c['allowed_turns'] = [turns(t) for t in args.turns.split(';')]
        if interactive:
            c['shoulder_enabled'] = yes('鐢熸垚姣忎晶 0.50m 璺偐', args.shoulder)
            if kind == 'grid':
                c['spacing_m'] = ask('璺彛涓績闂磋窛/绫?, c['spacing_m'], number(float, 206, 10000))
            c['arm_length_m'] = ask('澶栧洿閬撹矾鑷傞暱/绫?, c['arm_length_m'], number(float, 80, 500))
        number(int, 1, 6)(rows);number(int, 1, 6)(columns)
        c = normalize(c)
        number(float, 2*(c['junction_radius_m']+80), 10000)(c['spacing_m'])
        print('缁熶竴瑙勮寖锛?杩?鍑猴紱鍐呬晶宸﹁浆/涓棿鐩磋/澶栦晶鍙宠浆锛涚姝㈡帀澶达紱閫愯溅閬撶嫭绔嬩俊鍙枫€?)
        return c
    fields = [('incoming_lanes','姣忔柟鍚戣繘鍙ｈ溅閬撴暟锛堝嚭鍙ｅ悓鏁帮級',args.lanes,int,1,4),
              ('arm_length_m','澶栧洿閬撹矾鑷傞暱/绫?,args.arm_length,float,80,500),
              ('lane_width_m','杞﹂亾瀹藉害/绫?,args.lane_width,float,3,4.5)]
    c = dict(name=name,rows=rows,columns=columns)
    if kind == 't' and args.lanes != 2:
        raise ValueError('T junctions require --lanes 2')
    if kind=='t':
        def missing(value):
            if value not in ARM_NAMES:raise ValueError('杈撳叆 north/south/east/west')
            return value
        c['type']='t'
        c['missing_arm']=ask('缂哄け鏂瑰悜锛堝寳=north锛屽崡=south锛屼笢=east锛岃タ=west锛?,args.missing_arm or 'north',missing) if interactive else args.missing_arm or 'north'
    elif args.missing_arm:
        raise ValueError('--missing-arm 浠呯敤浜?--type t')
    for key,label,default,typ,low,high in fields:
        c[key] = ask(label,default,number(typ,low,high)) if interactive else number(typ,low,high)(default)
    n = c['incoming_lanes']
    minimum = n*c['lane_width_m']+8
    radius = args.radius if args.radius is not None else max((17.5 if kind == 't' else 22.),minimum)
    c['junction_radius_m'] = ask('璺彛涓績鑷抽亾璺鐐硅窛绂?绫?,radius,number(float,minimum,1000)) if interactive else number(float,minimum,1000)(radius)
    spacing = args.spacing if args.spacing is not None else max(344.,2*(c['junction_radius_m']+80))
    c['spacing_m'] = ask('璺彛涓績闂磋窛/绫?,spacing,number(float,2*(c['junction_radius_m']+80),10000)) if interactive and kind=='grid' else spacing
    c['sidewalk_width_m'] = ask('浜鸿閬撳搴?绫筹紙0 鍏抽棴锛?,args.sidewalk,number(float,0,5)) if interactive else args.sidewalk
    c['roadside_props'] = yes('娣诲姞闀挎绛夎矾杈瑰皬璁炬柦锛堟病鏈夊缓绛戯級',True) if interactive else not args.no_props
    c['seed'] = args.seed
    if kind=='t':
        if args.turns:raise ValueError('T 鍨嬭矾鍙ｆ瘡鏂瑰悜杞悜涓嶅悓锛岃鐢ㄤ氦浜掓ā寮忛€愭柟鍚戣缃紝涓嶈兘鐢ㄥ叡浜?--turns')
        c['arm_turns']={ARM_NAMES[i]:lanes for i,lanes in arm_turns(c).items()}
        print('鍚勮繘鍙ｄ粠鍐呬晶鑷冲渚х殑杞悜鍒嗛厤锛?,c['arm_turns'])
        if interactive and not yes('浣跨敤涓婅堪榛樿杞悜鍒嗛厤',True):
            for direction,lanes in list(c['arm_turns'].items()):
                while True:
                    proposed=[ask('{} 杩涘彛杞﹂亾 {} 鍏佽杞悜'.format(direction,i+1),','.join(t),turns) for i,t in enumerate(lanes)]
                    previous=c['arm_turns'][direction];c['arm_turns'][direction]=proposed
                    try:arm_turns(c);break
                    except ValueError as exc:
                        c['arm_turns'][direction]=previous;print('璇ュ垎閰嶄笉鍙敤锛岃閲嶆柊杈撳叆锛?,exc)
    else:
        c['allowed_turns'] = {1:[['left','straight','right']],2:[['left','straight'],['right']],
                              3:[['left'],['straight'],['right']],4:[['left'],['straight'],['straight'],['right']]}[n]
        print('浠庡唴渚ц嚦澶栦晶鐨勮浆鍚戝垎閰嶏細',c['allowed_turns'])
        if interactive and not yes('浣跨敤涓婅堪榛樿杞悜鍒嗛厤',True):
            c['allowed_turns'] = [ask('杞﹂亾 {} 鍏佽杞悜锛堣嫳鏂囬€楀彿鍒嗛殧锛?.format(i+1),','.join(t),turns) for i,t in enumerate(c['allowed_turns'])]
        if args.turns:c['allowed_turns'] = [turns(t) for t in args.turns.split(';')]
        if len(c['allowed_turns']) != n: raise ValueError('--turns 椤讳负姣忚溅閬撲竴缁勶紝鐢ㄥ垎鍙峰垎闅?)
    number(int,1,6)(rows);number(int,1,6)(columns)
    number(float,0,5)(c['sidewalk_width_m'])
    number(float,2*(c['junction_radius_m']+80),10000)(c['spacing_m'])
    return c


def main():
    p = argparse.ArgumentParser(description='杈撳叆鍙傛暟锛岃嚜鍔ㄤ繚瀛橀厤缃€佺敓鎴愬苟妫€鏌ュ湴鍥俱€傛湭甯?--name 鏃跺惎鍔ㄤ氦浜掑悜瀵笺€?)
    p.add_argument('--name');p.add_argument('--interactive',action='store_true')
    p.add_argument('--type',choices=['cross','grid','t'],default='cross')
    p.add_argument('--missing-arm',choices=['north','south','east','west'],help='T 鍨嬭矾鍙ｇ己澶辨柟鍚戯紝榛樿 north')
    p.add_argument('--lanes',type=int,default=3);p.add_argument('--arm-length',type=float,default=150)
    p.add_argument('--rows',type=int);p.add_argument('--columns',type=int)
    p.add_argument('--lane-width',type=float,default=3.5);p.add_argument('--radius',type=float)
    p.add_argument('--spacing',type=float);p.add_argument('--sidewalk',type=float,default=2)
    p.add_argument('--seed',type=int,default=42);p.add_argument('--no-props',action='store_true')
    p.add_argument('--shoulder',action='store_true',help='鏍囧噯鍗佸瓧/缃戞牸姣忎晶澧炲姞 0.50m 璺偐')
    p.add_argument('--turns',help='鐢卞唴鍚戝锛屽 "left;straight;right"')
    p.add_argument('--load',action='store_true',help='鎷撴墤妫€鏌ラ€氳繃鍚庡姞杞?CARLA')
    a = p.parse_args();interactive = a.interactive or a.name is None
    print('鍗佸瓧涓庤鍒欑綉鏍奸噰鐢?urban_cross3_v1 瑙勮寖锛汿 鍨嬩繚鐣欐棫鐢熸垚娴佺▼锛屼笉灞炰簬鏈疆瑙勮寖銆?)
    c = config(a,interactive)
    out = map_output(ROOT.parent/'maps'/c['name']);path = ROOT/'configs/generated'/(c['name']+'.json')
    if out.exists() or path.exists(): raise ValueError('鍚嶇О宸蹭娇鐢紝璇锋崲涓€涓悕绉帮紱涓嶄細瑕嗙洊宸叉湁鍦板浘鎴栭厤缃?)
    print(json.dumps(c,ensure_ascii=False,indent=2))
    print('鍦板浘鐩綍锛?,out)
    if interactive and not yes('纭鐢熸垚',True):
        print('宸插彇娑堬紝娌℃湁鍐欏叆鏂囦欢');return
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x',encoding='utf-8') as f: json.dump(c,f,ensure_ascii=False,indent=2)
    subprocess.run([sys.executable,'-B',str(ROOT/'toolkit.py'),'generate','--config',str(path),'--out',str(out)],check=True)
    print('鐢熸垚鍙婃嫇鎵戞鏌ラ€氳繃銆傞厤缃細',path)
    print('灏氭湭鍋氬疄杞﹂獙鏀讹紱鏂板湴鍥惧疄楠岄』閲嶆柊鍑嗗 profile锛屼笉鑳藉鐢ㄩ粯璁?Cross3 閰嶇疆銆?)
    load = a.load or (interactive and yes('鐜板湪鍔犺浇鍒?CARLA锛堥渶宸插惎鍔ㄤ笖鏃犲疄楠岃繍琛岋級',False))
    if load:
        subprocess.run([sys.executable,'-B',str(ROOT/'toolkit.py'),'view',str(out)],check=True)
    else:
        print('绋嶅悗鏌ョ湅锛歱ython toolkit.py view "{}"'.format(out))


if __name__ == '__main__':
    try: main()
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        print('鏈畬鎴愶細{}\n鑻ョ敓鎴愰樁娈靛け璐ワ紝宸插啓鍏ョ殑閰嶇疆/鍊欓€夌洰褰曚細淇濈暀锛岃妫€鏌ュ悗浣跨敤鏂板悕绉伴噸璇曘€?.format(exc),file=sys.stderr)
        raise SystemExit(1)
    except (KeyboardInterrupt,EOFError):
        print('\n宸蹭腑姝㈠悜瀵硷紱鑻ュ凡寮€濮嬬敓鎴愶紝璇锋鏌ヤ繚鐣欑殑杈撳嚭銆?);raise SystemExit(130)
