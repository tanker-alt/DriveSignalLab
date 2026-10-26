"""Coordinate-based heads, connected queues and lane-overlap spatial occupancy.

Bounding polygons are clipped to sampled lane strips before longitudinal union;
primary lane membership is separate from physical occupation of adjacent lanes.
"""
import json
import math


def hull(points):
    points = sorted(set(points))
    if len(points) < 3:
        return points
    def cross(a,b,c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    lower,upper=[],[]
    for target,sequence in ((lower,points),(upper,reversed(points))):
        for p in sequence:
            while len(target)>1 and cross(target[-2],target[-1],p)<=0:
                target.pop()
            target.append(p)
    return lower[:-1]+upper[:-1]


def clip(poly, axis, boundary, sign):
    result=[]
    if not poly:
        return result
    old=poly[-1]; old_d=sign*(old[axis]-boundary)
    for p in poly:
        d=sign*(p[axis]-boundary)
        if (d>=0)!=(old_d>=0):
            t=old_d/(old_d-d)
            result.append((old[0]+t*(p[0]-old[0]),old[1]+t*(p[1]-old[1])))
        if d>=0:
            result.append(p)
        old,old_d=p,d
    return result


def union_length(intervals):
    total=0.; end=-float("inf")
    for a,b in sorted(intervals):
        total += max(0.,b-max(a,end))
        end=max(end,b)
    return total


class ApproachMetrics:
    def __init__(self, geometry):
        self.geometry=geometry
        self.config=geometry["config"]
        for key in ("queue_speed_mps","queue_gap_m","queue_head_distance_m","height_tolerance_m"):
            if not math.isfinite(self.config[key]) or self.config[key]<0:
                raise ValueError("Invalid approach metric parameter: "+key)
        self.lanes=[]
        self.keysets={}
        self.bounds={}
        for lane in geometry["lanes"]:
            self.keysets[lane["id"]]={tuple(str(p[k]) for k in ("road_id","section_id","lane_id")) for p in lane["samples"]}
            points=lane["samples"]
            if points:
                margin=max(p["width"] for p in points)/2
                self.bounds[lane["id"]]=(min(p["x"] for p in points)-margin,max(p["x"] for p in points)+margin,
                    min(p["y"] for p in points)-margin,max(p["y"] for p in points)+margin)
            segments=[]
            for a,b in zip(lane["samples"],lane["samples"][1:]):
                length=math.hypot(b["x"]-a["x"],b["y"]-a["y"])
                if length<=0 or min(a["width"],b["width"])<=0:
                    raise ValueError("Invalid lane geometry")
                segments.append((a,b,length,(b["x"]-a["x"])/length,(b["y"]-a["y"])/length))
            self.lanes.append((lane,segments))

    @staticmethod
    def project(point,segments,keys=None,extended=False):
        best=None
        for index,(a,b,length,ux,uy) in enumerate(segments):
            if keys is not None and not any(tuple(str(p[k]) for k in ("road_id","section_id","lane_id"))==keys for p in (a,b)):
                continue
            dx,dy=point[0]-a["x"],point[1]-a["y"]
            along=dx*ux+dy*uy
            t=max(0.,min(length,along))
            distance=(dx-t*ux)**2+(dy-t*uy)**2
            if best is None or distance<best[0]:
                s=along if extended and ((index==0 and along<0) or (index==len(segments)-1 and along>length)) else t
                best=(distance,a["s"]+s,ux,uy,a["z"],min(a["width"],b["width"])/2)
        return best

    def sample(self,rows):
        self.vehicle_details={str(row["vehicle_id"]):dict(geometry_match="unmapped") for row in rows}
        vehicles=[]
        for row in rows:
            vertices=row.get("bbox_vertices_world_json")
            vertices=json.loads(vertices) if isinstance(vertices,str) and vertices else vertices
            if not vertices:
                vertices=[]
            vehicles.append((row,vertices,hull([(v[0],v[1]) for v in vertices])))
        # A primary actor must be counted in at most one approach even where paths merge.
        assignments={}
        ambiguous_lanes=set()
        for row,vertices,poly in vehicles:
            keys=tuple(str(row.get(k,"")) for k in ("road_id","section_id","lane_id"))
            candidates=[]
            for lane,segments in self.lanes:
                if keys not in self.keysets[lane["id"]]:
                    continue
                p=self.project((float(row["x"]),float(row["y"])),segments,keys,True)
                if p and 0<=p[1]<=lane["samples"][-1]["s"] and p[0]<=p[5]**2 and abs(float(row["z"])-p[4])<=self.config["height_tolerance_m"]:
                    candidates.append((p[0],lane["id"],p))
            if candidates:
                candidates.sort(key=lambda x:(x[0],x[1]))
                # Equal-distance merged approaches cannot establish a unique approach.
                if len(candidates)==1 or candidates[1][0]-candidates[0][0]>1e-4:
                    assignments[str(row["vehicle_id"])]=(candidates[0][1],candidates[0][2])
                else:
                    self.vehicle_details[str(row["vehicle_id"])]["geometry_match"]="ambiguous"
                    ambiguous_lanes.update(candidate[1] for candidate in candidates if candidate[0]-candidates[0][0]<=1e-4)
        results=[]
        for lane,segments in self.lanes:
            length=lane["samples"][-1]["s"] if segments else 0
            members,intervals=[],[]
            geometry_missing=False
            for row,vertices,poly in vehicles:
                assigned=assignments.get(str(row["vehicle_id"]))
                if assigned and assigned[0]==lane["id"]:
                    detail=self.vehicle_details[str(row["vehicle_id"])]
                    detail.update(geometry_match="unique",approach_id=lane["id"],
                        origin_reference_distance_m=assigned[1][1],
                        reference_kind=lane.get("reference_kind","carla_stop_waypoint"),
                        calibration_id=lane.get("calibration_id"))
                    if vertices:
                        projected=[self.project(v,segments,extended=True)[1] for v in vertices]
                        detail.update(front_reference_distance_m=min(projected),rear_reference_distance_m=max(projected))
                        if lane.get('stop_line'):
                            line=lane['stop_line'];center=line['center'];nx,ny=line['normal_xy']
                            signed=[(center['x']-v[0])*nx+(center['y']-v[1])*ny for v in vertices]
                            detail.update(stop_line_front_distance_m=min(signed),stop_line_rear_distance_m=max(signed))
                        members.append(dict(row=row,front=min(projected),rear=max(projected),projection=assigned[1]))
                    else:
                        geometry_missing=True
                if not vertices:
                    # Missing boxes can invalidate occupancy even if origin belongs to adjacent lane.
                    near=self.project((float(row["x"]),float(row["y"])),segments)
                    if near and near[0] <= (near[5]+5)**2:
                        geometry_missing=True
                    continue
                bounds=self.bounds[lane["id"]]
                minx,maxx=min(p[0] for p in poly),max(p[0] for p in poly)
                miny,maxy=min(p[1] for p in poly),max(p[1] for p in poly)
                if maxx<bounds[0] or minx>bounds[1] or maxy<bounds[2] or miny>bounds[3]:
                    continue
                for a,b,segment_length,ux,uy in segments:
                    width=min(a["width"],b["width"])/2
                    if maxx<min(a["x"],b["x"])-width or minx>max(a["x"],b["x"])+width or maxy<min(a["y"],b["y"])-width or miny>max(a["y"],b["y"])+width:
                        continue
                    if max(v[2] for v in vertices)<min(a["z"],b["z"])-self.config["height_tolerance_m"] or min(v[2] for v in vertices)>max(a["z"],b["z"])+self.config["height_tolerance_m"]:
                        continue
                    local=[((x-a["x"])*ux+(y-a["y"])*uy,-(x-a["x"])*uy+(y-a["y"])*ux) for x,y in poly]
                    for axis,boundary,sign in ((0,0,1),(0,segment_length,-1),(1,-width,1),(1,width,-1)):
                        local=clip(local,axis,boundary,sign)
                    area=abs(sum(local[i-1][0]*p[1]-p[0]*local[i-1][1] for i,p in enumerate(local)))/2 if local else 0
                    if area>1e-7:
                        intervals.append((a["s"]+min(p[0] for p in local),a["s"]+max(p[0] for p in local)))
            members.sort(key=lambda m:(m["front"],str(m["row"]["vehicle_id"])))
            invalid_members=geometry_missing or lane["id"] in ambiguous_lanes or not segments
            queue=[]
            for member in members:
                speed=math.hypot(float(member["row"]["vx"]),float(member["row"]["vy"]))
                if speed>self.config["queue_speed_mps"]:
                    break
                if not queue and member["front"]>self.config["queue_head_distance_m"]:
                    break
                if queue and member["front"]-queue[-1]["rear"]>self.config["queue_gap_m"]:
                    break
                queue.append(member)
            result=dict(approach_id=lane["id"],signal_candidates="|".join(sorted(set(lane["signal_ids"]))),
                detection_length_m=length,geometry_status=lane["geometry_status"],
                metric_status=("no_lane_geometry" if not segments else "missing_vehicle_geometry" if geometry_missing else
                               "ambiguous_primary_lane" if lane["id"] in ambiguous_lanes else "valid"),
                primary_vehicle_count=None if lane["id"] in ambiguous_lanes or not segments else sum(1 for assigned in assignments.values() if assigned[0]==lane["id"]),
                queue_vehicle_count=len(queue) if not invalid_members else None,
                queue_tail_id=queue[-1]["row"]["vehicle_id"] if queue else None,
                queue_length_m=queue[-1]["rear"]-queue[0]["front"] if queue else 0.,
                spatial_occupancy= min(1.,union_length(intervals)/length) if length and not geometry_missing else None)
            for prefix,member in (("lane_head",members[0] if members and not invalid_members else None),
                                  ("queue_head",queue[0] if queue and not invalid_members else None)):
                result[prefix+"_id"]=member["row"]["vehicle_id"] if member else None
                result[prefix+"_distance_m"]=member["front"] if member else None
                result[prefix+"_speed_mps"]=math.hypot(float(member["row"]["vx"]),float(member["row"]["vy"])) if member else None
                result[prefix+"_longitudinal_accel_mps2"]=(-float(member["row"]["ax"])*member["projection"][2]-float(member["row"]["ay"])*member["projection"][3]) if member else None
            if invalid_members:
                result.update(queue_tail_id=None,queue_length_m=None)
            results.append(result)
        return results
