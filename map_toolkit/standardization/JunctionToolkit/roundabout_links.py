"""Validate every explicit junction connector before exporting a roundabout."""


def verify(root, connections):
    roads={int(r.get('id')):r for r in root.findall('road')}
    junctions={int(j.get('id')):j for j in root.findall('junction')}
    for path in connections:
        connector=roads[path['road']]; jid=int(connector.get('junction'))
        lane=connector.find("./lanes/laneSection/*/lane[@id='%s']" % path['lane'])
        if lane is None:raise ValueError('Missing connector lane: %s' % path)
        for tag,road_key,lane_key in [('predecessor','source','source_lane'),('successor','target','target_lane')]:
            link=connector.find('./link/'+tag)
            if link is None or link.get('elementType')!='road' or int(link.get('elementId'))!=path[road_key]:
                raise ValueError('Broken connector road link: %s %s' % (path,tag))
            ll=lane.find('./link/'+tag)
            parent=roads[path[road_key]]
            if ll is None or int(ll.get('id'))!=path[lane_key] or parent.find("./lanes/laneSection/*/lane[@id='%s']" % path[lane_key]) is None:
                raise ValueError('Broken connector lane link: %s %s' % (path,tag))
            endpoint='predecessor' if link.get('contactPoint')=='start' else 'successor'
            back=parent.find('./link/'+endpoint)
            if back is None or back.get('elementType')!='junction' or int(back.get('elementId'))!=jid:
                raise ValueError('Parent endpoint does not reference access junction: %s' % path)
        entries=junctions[jid].findall("./connection[@incomingRoad='%s'][@connectingRoad='%s']/laneLink" % (path['source'],path['road']))
        if not any(int(e.get('from'))==path['source_lane'] and int(e.get('to'))==path['lane'] for e in entries):
            raise ValueError('Missing junction laneLink: %s' % path)
    return dict(passed=True,connection_lanes=len(connections))
