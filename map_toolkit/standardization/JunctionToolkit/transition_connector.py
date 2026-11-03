import math, xml.etree.ElementTree as ET

def build(root, source, target, rid=20000, length=160., split=80., width=3.5):
    a=root.find("./road[@id='%s']"%source); b=root.find("./road[@id='%s']"%target)
    ga=a.find('./planView/geometry'); gb=b.find('./planView/geometry')
    x=float(ga.get('x'))+float(ga.get('length'))*math.cos(float(ga.get('hdg')))
    y=float(ga.get('y'))+float(ga.get('length'))*math.sin(float(ga.get('hdg')))
    tx,ty=float(gb.get('x')),float(gb.get('y'))
    length=math.hypot(tx-x,ty-y); heading=math.atan2(ty-y,tx-x)
    split=min(split,max(1.,length-1.))
    r=ET.Element('road',{'name':'lane_transition','length':str(length),'id':str(rid),'junction':'-1'})
    link=ET.SubElement(r,'link'); ET.SubElement(link,'predecessor',{'elementType':'road','elementId':str(source),'contactPoint':'end'}); ET.SubElement(link,'successor',{'elementType':'road','elementId':str(target),'contactPoint':'start'})
    plan=ET.SubElement(r,'planView'); g=ET.SubElement(plan,'geometry',{'s':'0','x':str(x),'y':str(y),'hdg':str(heading),'length':str(length)}); ET.SubElement(g,'line')
    lanes=ET.SubElement(r,'lanes')
    for s,n in ((0,2),(split,3)):
        sec=ET.SubElement(lanes,'laneSection',{'s':str(s)}); ET.SubElement(sec,'center')
        right=ET.SubElement(sec,'right')
        for i in range(1,n+1):
            lane=ET.SubElement(right,'lane',{'id':str(-i),'type':'driving','level':'false'}); ET.SubElement(lane,'width',{'sOffset':'0','a':str(width),'b':'0','c':'0','d':'0'})
            ET.SubElement(lane,'link')
    root.append(r)
    for rr,tag,eid,cp in ((a,'successor',rid,'start'),(b,'predecessor',rid,'end')):
        ln=rr.find('link')
        if ln is None: ln=ET.SubElement(rr,'link')
        ET.SubElement(ln,tag,{'elementType':'road','elementId':str(eid),'contactPoint':cp})
    return r
