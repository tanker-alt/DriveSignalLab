"""Visual-only lane/stop/group overlays; no physics or lane-policy changes."""
import math
import carla


def build(world,design):
    m=world.get_map();width=design['config']['lane_width_m'];white=carla.Color(140,140,130);yellow=carla.Color(120,90,8)
    def line(a,b,color=white,thickness=.08):
        a.z=b.z=.065;world.debug.draw_line(a,b,thickness=thickness,color=color,life_time=0,persistent_lines=True)
    # Ring lane divider follows the actual curved road reference.
    for rid in design['ring_roads']+design['ring_continuations']:
        length=design['road_geometry'][str(rid)]['length']
        for k in range(int(length*2)):
            s=.01+k*.5;end=min(length-.01,s+.5)
            if end<=s:continue
            a=m.get_waypoint_xodr(rid,-1,s);b=m.get_waypoint_xodr(rid,-1,end)
            if k%12<6:line(a.transform.location+a.transform.get_right_vector()*width/2,b.transform.location+b.transform.get_right_vector()*width/2)
    for rid in range(1,5):
        length=design['road_geometry'][str(rid)]['length']
        for s in range(1,int(length)-1,5):
            a=m.get_waypoint_xodr(rid,-1,s);b=m.get_waypoint_xodr(rid,-1,min(length-.1,s+2.5))
            for offset in (width/2,-1.5*width):line(a.transform.location+a.transform.get_right_vector()*offset,b.transform.location+b.transform.get_right_vector()*offset)
        a=m.get_waypoint_xodr(rid,-1,.1);b=m.get_waypoint_xodr(rid,-1,length-.1)
        line(a.transform.location-a.transform.get_right_vector()*width/2,b.transform.location-b.transform.get_right_vector()*width/2,yellow,.1)
    # Ground-level seven-segment numbers remain visible in RGB cameras.
    digits={'1':'bc','2':'abged','3':'abgcd','4':'fgbc','5':'afgcd','6':'afgecd','7':'abc','8':'abcdefg'}
    segments={'a':((0,0),(2,0)),'b':((2,0),(2,2)),'c':((2,2),(2,4)),'d':((0,4),(2,4)),'e':((0,2),(0,4)),'f':((0,0),(0,2)),'g':((0,2),(2,2))}
    positions={}
    for group,info in design.get('signal_groups',{}).items():
        cfg=design['signals'][info['signal_ids'][0]];wp=m.get_waypoint_xodr(cfg['road_id'],-1,cfg['reference_s'])
        p=wp.transform.location+wp.transform.get_right_vector()*(2*width+4)
        positions[group]=[p.x,p.y,.08]
        for seg in digits[group]:
            a,b=segments[seg];line(carla.Location(p.x+a[0],p.y+a[1],.08),carla.Location(p.x+b[0],p.y+b[1],.08),carla.Color(40,90,110),.18)
    return positions
