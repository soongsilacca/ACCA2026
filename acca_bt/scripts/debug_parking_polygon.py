#!/usr/bin/env python3
import xml.etree.ElementTree as ET
import math

osm_file = '/home/won/BT/osm/school4.osm'
tree = ET.parse(osm_file)
root = tree.getroot()

nodes = {}
for node in root.findall('node'):
    nid = node.get('id')
    lx = None; ly = None
    for tag in node.findall('tag'):
        if tag.get('k') == 'local_x': lx = float(tag.get('v'))
        if tag.get('k') == 'local_y': ly = float(tag.get('v'))
    if lx is not None:
        nodes[nid] = (lx, ly)

print(f"Loaded {len(nodes)} nodes.")

def is_inside(polygon, x, y):
    n = len(polygon)
    if n < 3: return False
    inside = False
    p = polygon
    for i in range(n):
        j = (i - 1) % n
        if ((p[i][1] > y) != (p[j][1] > y)) and \
           (x < (p[j][0] - p[i][0]) * (y - p[i][1]) / (p[j][1] - p[i][1]) + p[i][0]):
            inside = not inside
    return inside

# Find parking relation
for relation in root.findall('relation'):
    subtype = ""
    for tag in relation.findall('tag'):
        if tag.get('k') == 'subtype': subtype = tag.get('v')
    
    if subtype == 'parking':
        print(f"Found parking relation {relation.get('id')}")
        
        # Logic 1: Current osm_loader (BREAKS after first way)
        points_single = []
        for m in relation.findall('member'):
            if m.get('type') == 'way': 
                way_ref = m.get('ref')
                for way in root.findall('way'):
                    if way.get('id') == way_ref:
                        for nd in way.findall('nd'):
                            if nd.get('ref') in nodes:
                                points_single.append(nodes[nd.get('ref')])
                break # OLD LOGIC
        
        # Logic 2: All ways (Corrected)
        points_all = []
        for m in relation.findall('member'):
            if m.get('type') == 'way': 
                way_ref = m.get('ref')
                for way in root.findall('way'):
                    if way.get('id') == way_ref:
                        for nd in way.findall('nd'):
                            if nd.get('ref') in nodes:
                                points_all.append(nodes[nd.get('ref')])

        user_x, user_y = -6.012, 146.982
        print(f"  [Single Way] Points: {len(points_single)}")
        print(f"  [Single Way] Inside? {is_inside(points_single, user_x, user_y)}")
        
        print(f"  [All Ways]   Points: {len(points_all)}")
        print(f"  [All Ways]   Inside? {is_inside(points_all, user_x, user_y)}")
