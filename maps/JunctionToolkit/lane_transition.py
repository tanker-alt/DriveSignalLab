"""Internal OpenDRIVE lane transition policy for generated networks."""
import xml.etree.ElementTree as ET

DEFAULT_TRANSITION_M = 80.0

def apply_lane_transition(road, from_lanes, to_lanes, transition_m=DEFAULT_TRANSITION_M, lane_width=3.5):
    """Add legal laneSection metadata for a widening/narrowing road.

    The caller owns road connectivity; this helper only creates the section
    boundary and width polynomial for the added edge lane.
    """
    if from_lanes == to_lanes:
        return False
    lanes = road.find('./lanes')
    section = lanes.find('laneSection')
    if section is None:
        return False
    side = section.find('left') if to_lanes > from_lanes else section.find('right')
    if side is None:
        return False
    edge_id = to_lanes if to_lanes > from_lanes else -from_lanes
    lane = side.find("lane[@id='%s']" % edge_id)
    if lane is None:
        lane = ET.SubElement(side, 'lane', {'id':str(edge_id), 'type':'driving', 'level':'false'})
        ET.SubElement(lane, 'link')
    width = lane.find('width')
    if width is None:
        width = ET.SubElement(lane, 'width')
    # Keep the base lane width until the road-splitting pass creates a new
    # laneSection. CARLA rejects zero-width lanes in a junction-connected
    # section, so transitions are materialized by the caller in separate roads.
    width.set('sOffset','0'); width.set('a',str(lane_width)); width.set('b','0'); width.set('c','0'); width.set('d','0')
    return True
