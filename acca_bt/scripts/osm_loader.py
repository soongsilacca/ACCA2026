#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, PoseArray, Polygon, Point32
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import xml.etree.ElementTree as ET
import sys
import os
import math

class OsmLoader(Node):
    def __init__(self):
        super().__init__('osm_loader')
        
        self.latching_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        
        self.path_pub = self.create_publisher(Path, '/global_path', self.latching_qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/vector_map_marker', self.latching_qos)
        self.stop_line_pub = self.create_publisher(PoseArray, '/stop_lines', 10) # Changed to reliable/volatile to match BT node
        self.parking_zones_pub = self.create_publisher(PoseArray, '/parking/zones', self.latching_qos)
        self.boundary_pub = self.create_publisher(Polygon, '/parking/boundary', self.latching_qos)
        self.debug_centerlines_pub = self.create_publisher(MarkerArray, '/debug_all_centerlines', self.latching_qos)
        
        self.nodes = {}           
        self.way_points = {}      
        self.lanelets = []        
        self.adj = {}
        self.parking_pubs = {}
        self.parking_msgs = {}
        self.parking_array_msg = None
        self.stop_line_array_msg = None
        self.path_msg = None
        
        self.marker_array = MarkerArray() 
        self.marker_id_counter = 0

        default_osm_path = '/home/won/BT/osm/school4.osm'
        self.declare_parameter('osm_file', default_osm_path)
        osm_file = self.get_parameter('osm_file').get_parameter_value().string_value
        
        self.ref_lat = 37.0
        self.ref_lon = 127.0
        self.m_per_lat = 111111.0 
        self.m_per_lon = 111111.0 * math.cos(math.radians(self.ref_lat))
        
        self.load_osm(osm_file)
        self.generate_global_path()
        self.create_timer(1.0, self.publish_markers) 
        
        self.get_logger().info("OsmLoader initialized (Full Map Global Path).")

    # ==========================================================================
    # [Helper Methods]
    # ==========================================================================
    def get_path_length(self, points):
        if len(points) < 2: return 0.0
        d = 0.0
        for i in range(len(points)-1):
            dx = points[i+1][0] - points[i][0]
            dy = points[i+1][1] - points[i][1]
            d += math.sqrt(dx*dx + dy*dy)
        return d

    def get_point_at_ratio(self, points, ratio, total_length=None):
        if not points: return None
        if len(points) == 1: return points[0]
        if ratio <= 0.0: return points[0]
        if ratio >= 1.0: return points[-1]
        
        if total_length is None:
            total_length = self.get_path_length(points)
            
        target_dist = total_length * ratio
        current_dist = 0.0
        
        for i in range(len(points)-1):
            p1 = points[i]
            p2 = points[i+1]
            seg_len = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
            
            if current_dist + seg_len >= target_dist:
                remain = target_dist - current_dist
                seg_ratio = remain / seg_len if seg_len > 0 else 0
                nx = p1[0] + (p2[0]-p1[0]) * seg_ratio
                ny = p1[1] + (p2[1]-p1[1]) * seg_ratio
                nz = 0.0
                if len(p1) > 2 and len(p2) > 2:
                    nz = p1[2] + (p2[2]-p1[2]) * seg_ratio
                    return (nx, ny, nz)
                else:
                    return (nx, ny)
            
            current_dist += seg_len
            
        return points[-1]

    def create_line_marker(self, way_id, ns, r, g, b, ways_refs, scale=0.15):
        if way_id not in ways_refs: return None
        points = []
        for nid in ways_refs[way_id]:
            if nid in self.nodes:
                x, y, z = self.nodes[nid]
                points.append(Point(x=x, y=y, z=z))
        if len(points) < 2: return None
        
        m = Marker()
        m.header.frame_id = "map"
        m.ns = ns; m.id = self.marker_id_counter; self.marker_id_counter += 1
        m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.scale.x = scale
        m.color = ColorRGBA(r=r, g=g, b=b, a=0.9)
        m.points = points
        return m

    # ==========================================================================
    # [Call Callback]
    # ==========================================================================
    def publish_markers(self):
        now = self.get_clock().now().to_msg()
        if hasattr(self, 'marker_array'):
            for m in self.marker_array.markers: m.header.stamp = now
            self.marker_pub.publish(self.marker_array)
        if hasattr(self, 'parking_msgs'):
            for pk_id, msg in self.parking_msgs.items():
                if pk_id in self.parking_pubs:
                    msg.header.stamp = now
                    self.parking_pubs[pk_id].publish(msg)
        if hasattr(self, 'parking_array_msg') and self.parking_array_msg:
             self.parking_array_msg.header.stamp = now
             self.parking_zones_pub.publish(self.parking_array_msg)
        if hasattr(self, 'parking_boundary_msg'):
             self.boundary_pub.publish(self.parking_boundary_msg)
        if hasattr(self, 'stop_line_array_msg') and self.stop_line_array_msg:
             self.stop_line_array_msg.header.stamp = now
             self.stop_line_pub.publish(self.stop_line_array_msg)
        if self.path_msg:
            self.path_msg.header.stamp = now
            self.path_pub.publish(self.path_msg)
            
        # [DEBUG] Publish ALL centerlines faintly
        if self.lanelets:
            ma_debug = MarkerArray()
            for i, ll in enumerate(self.lanelets):
                m = Marker()
                m.header.frame_id = "map"; m.ns = "debug_centerlines"; m.id = i
                m.type = Marker.LINE_STRIP; m.action = Marker.ADD
                m.scale.x = 0.05
                m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.3) # Faint white
                for cx, cy in ll['centerline']:
                    m.points.append(Point(x=cx, y=cy, z=0.0))
                ma_debug.markers.append(m)
            self.debug_centerlines_pub.publish(ma_debug)

    # ==========================================================================
    # [Load OSM & Generate Centerline]
    # ==========================================================================
    def load_osm(self, osm_file):
        self.get_logger().info(f"Loading OSM from: {osm_file}")
        if not os.path.exists(osm_file):
             self.get_logger().error(f"File not found: {osm_file}")
             return

        try:
            tree = ET.parse(osm_file)
            root = tree.getroot()
            
            for node in root.findall('node'):
                nid = node.get('id')
                lx = None; ly = None; lz = 0.0
                for tag in node.findall('tag'):
                    if tag.get('k') == 'local_x': lx = float(tag.get('v'))
                    if tag.get('k') == 'local_y': ly = float(tag.get('v'))
                    if tag.get('k') == 'ele': lz = float(tag.get('v'))
                if lx is not None and ly is not None:
                    self.nodes[nid] = (lx, ly, lz)
            
            ways_refs = {} 
            for way in root.findall('way'):
                wid = way.get('id')
                pts = []
                refs = []
                for nd in way.findall('nd'):
                    ref = nd.get('ref')
                    refs.append(ref)
                    if ref in self.nodes:
                        pts.append(self.nodes[ref])
                self.way_points[wid] = [(p[0], p[1]) for p in pts]
                ways_refs[wid] = refs

            self.marker_array = MarkerArray()
            self.marker_id_counter = 0

            for relation in root.findall('relation'):
                is_ll = False
                for tag in relation.findall('tag'):
                    if tag.get('k') == 'type' and tag.get('v') == 'lanelet': is_ll = True
                if not is_ll: continue
                
                rel_id = relation.get('id')
                left_id = None; right_id = None
                
                for m in relation.findall('member'):
                    if m.get('role') == 'left': left_id = m.get('ref')
                    if m.get('role') == 'right': right_id = m.get('ref')

                if left_id:
                    m = self.create_line_marker(left_id, "lanelet_left", 1.0, 0.0, 0.0, ways_refs) 
                    if m: self.marker_array.markers.append(m)
                if right_id:
                    m = self.create_line_marker(right_id, "lanelet_right", 0.0, 0.0, 1.0, ways_refs) 
                    if m: self.marker_array.markers.append(m)

                if left_id and right_id:
                    left_pts_raw = self.way_points.get(left_id, [])
                    right_pts_raw = self.way_points.get(right_id, [])
                    
                    if left_pts_raw and right_pts_raw:
                        d00 = (left_pts_raw[0][0]-right_pts_raw[0][0])**2 + (left_pts_raw[0][1]-right_pts_raw[0][1])**2
                        d01 = (left_pts_raw[0][0]-right_pts_raw[-1][0])**2 + (left_pts_raw[0][1]-right_pts_raw[-1][1])**2
                        
                        right_pts_proc = list(right_pts_raw)
                        if d01 < d00: 
                            right_pts_proc.reverse()
                            
                        # Ratio-based Centerline
                        len_l = self.get_path_length(left_pts_raw)
                        len_r = self.get_path_length(right_pts_proc)
                        avg_len = (len_l + len_r) / 2.0
                        n_samples = max(2, int(avg_len / 0.2)) 
                        
                        centerline = []
                        for i in range(n_samples):
                            ratio = i / (n_samples - 1)
                            p_l = self.get_point_at_ratio(left_pts_raw, ratio, len_l)
                            p_r = self.get_point_at_ratio(right_pts_proc, ratio, len_r)
                            cx = (p_l[0] + p_r[0]) / 2.0
                            cy = (p_l[1] + p_r[1]) / 2.0
                            centerline.append((cx, cy))
                            
                        if len(centerline) > 1:
                            y1 = math.atan2(centerline[1][1]-centerline[0][1], centerline[1][0]-centerline[0][0])
                            y2 = math.atan2(centerline[-1][1]-centerline[-2][1], centerline[-1][0]-centerline[-2][0])
                            self.lanelets.append({'id': rel_id, 'centerline': centerline, 'start_yaw': y1, 'end_yaw': y2})

            self.parse_special_zones(root, ways_refs)
            self.get_logger().info(f"Loaded {len(self.lanelets)} lanelets.")

        except Exception as e:
            self.get_logger().error(f"Failed to parse OSM: {e}")
            import traceback
            traceback.print_exc()

    def generate_global_path(self):
        if not self.lanelets: return

        # 1. 인접성 구축 (Graph Building)
        self.adj = {}
        for l1 in self.lanelets:
                self.adj[l1['id']] = []
                p1 = l1['centerline'][-1] # End point of current lanelet
                
                THRESHOLD = 10000.0 # 100m (Huge range, trusting "Best Match" to filter)
                
                candidates = []

                # Iterate all potential L2s to find the BEST match
                for l2 in self.lanelets:
                     if l1['id'] == l2['id']: continue
                     
                     p2_start = l2['centerline'][0]
                     d_start = (p1[0]-p2_start[0])**2 + (p1[1]-p2_start[1])**2
                     
                     p2_end = l2['centerline'][-1]
                     d_rev = (p1[0]-p2_end[0])**2 + (p1[1]-p2_end[1])**2
                     
                     # Check 1: Normal
                     if d_start < THRESHOLD:
                         candidates.append((d_start, l2, False))

                     # Check 2: Reverse
                     if d_rev < THRESHOLD:
                         candidates.append((d_rev, l2, True))

                # Select ONLY the closest candidate (Winner takes all)
                if candidates:
                    candidates.sort(key=lambda x: x[0])
                    
                    best_dist, best_l2, need_reverse = candidates[0]
                    
                    # Log the connection for debugging
                    # self.get_logger().info(f"Connected {l1['id']} -> {best_l2['id']} (Dist: {math.sqrt(best_dist):.2f}m)")
                    
                    if need_reverse:
                        self.get_logger().info(f"Flipping reversed lanelet {best_l2['id']} to connect with {l1['id']}")
                        best_l2['centerline'].reverse()
                        best_l2['start_yaw'], best_l2['end_yaw'] = best_l2['end_yaw'], best_l2['start_yaw']
                        best_l2['start_yaw'] += math.pi
                        best_l2['end_yaw'] += math.pi
                    
                    self.adj[l1['id']].append(best_l2['id'])

        # 2. Recursive DFS for Longest Path to solve "limit" issue
        sys.setrecursionlimit(2000)
        self.max_path = []

        def dfs(curr_id, visited):
            # Check if this is the longest path so far
            if len(visited) > len(self.max_path):
                self.max_path = list(visited)

            # Get neighbors and sort for heuristic (straightest first)
            nbrs = self.adj.get(curr_id, [])
            curr_ll = next((l for l in self.lanelets if l['id'] == curr_id), None)
            
            if curr_ll and len(nbrs) > 1:
                curr_yaw = curr_ll['end_yaw']
                def get_angle_diff(nid):
                    n_ll = next((l for l in self.lanelets if l['id'] == nid), None)
                    if not n_ll: return 999.0
                    ny = n_ll['start_yaw']
                    return abs(math.atan2(math.sin(ny - curr_yaw), math.cos(ny - curr_yaw)))
                nbrs.sort(key=get_angle_diff)

            for nid in nbrs:
                if nid not in visited:
                    visited.append(nid)
                    dfs(nid, visited)
                    visited.pop()
                # If we hit the start node again (loop closure), that's good, but we don't recurse infinitely
        
        # Try finding path from all possible starts
        # Optimization: Track visited "starts" to avoid re-scanning sub-components
        processed_starts = set()
        best_path_global = []

        # Sort for determinism
        sorted_ids = [l['id'] for l in self.lanelets]

        for start_id in sorted_ids:
            if start_id in processed_starts: continue
            
            self.max_path = []
            dfs(start_id, [start_id])
            
            if len(self.max_path) > len(best_path_global):
                best_path_global = list(self.max_path)
            
            # Heuristic: mark nodes in the found path as processed
            for p in self.max_path:
                processed_starts.add(p)

        # 3. Visually Close the Loop if applicable
        if best_path_global:
            start_id = best_path_global[0]
            end_id = best_path_global[-1]
            # Check if End connects to Start
            if start_id in self.adj.get(end_id, []):
                 best_path_global.append(start_id)
                 self.get_logger().info("Loop closure detected: Appending start node to close the visual loop.")

        self.construct_path_msg(best_path_global)

    def construct_path_msg(self, loop_ids):
        path = Path()
        path.header.frame_id = "map"
        
        # Collect all points first
        raw_points = []
        for pid in loop_ids:
            ll = next((l for l in self.lanelets if l['id'] == pid), None)
            if not ll: continue
            for x, y in ll['centerline']:
                raw_points.append((x, y))
        
        if not raw_points: return []

        # Filter points that are too close (min 0.1m)
        filtered_points = [raw_points[0]]
        for i in range(1, len(raw_points)):
            lx, ly = filtered_points[-1]
            cx, cy = raw_points[i]
            dist = math.sqrt((cx-lx)**2 + (cy-ly)**2)
            if dist > 0.1:
                filtered_points.append((cx, cy))
        
        # Generate Path with Yaw
        for i in range(len(filtered_points)):
            x, y = filtered_points[i]
            p = PoseStamped()
            p.header.frame_id = "map"
            p.pose.position.x = x
            p.pose.position.y = y
            
            # Calculate Yaw
            if i < len(filtered_points) - 1:
                dx = filtered_points[i+1][0] - x
                dy = filtered_points[i+1][1] - y
                if abs(dx) < 1e-6 and abs(dy) < 1e-6:
                     # Fallback to previous yaw if extremely close (should cover by filter, but safety)
                     yaw = 0.0
                     if len(path.poses) > 0:
                         yaw = self.quat_to_yaw(path.poses[-1].pose.orientation)
                else:
                    yaw = math.atan2(dy, dx)
            else:
                # Last point: use previous yaw
                yaw = 0.0
                if len(path.poses) > 0:
                   yaw = self.quat_to_yaw(path.poses[-1].pose.orientation)

            # Yaw to Quaternion
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            p.pose.orientation.w = cy
            p.pose.orientation.z = sy
            
            path.poses.append(p)

        self.path_msg = path
        self.get_logger().info(f"Path Constructed: {len(path.poses)} points (filtered from {len(raw_points)})")
        return loop_ids
    
    def quat_to_yaw(self, q):
        # Helper to recover yaw from quaternion for the last point logic
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def parse_special_zones(self, root, ways_refs):
        parking_array = PoseArray(); parking_array.header.frame_id = "map"
        for relation in root.findall('relation'):
            subtype = ""
            for tag in relation.findall('tag'):
                if tag.get('k') == 'subtype': subtype = tag.get('v')
            if 'parking' in subtype:
                way_ref = None
                for m in relation.findall('member'):
                    if m.get('type') == 'way': way_ref = m.get('ref'); break
                if way_ref and way_ref in self.way_points:
                    pts = self.way_points[way_ref]
                    cx = sum(p[0] for p in pts) / len(pts)
                    cy = sum(p[1] for p in pts) / len(pts)
                    p_msg = PoseStamped().pose; p_msg.position.x = cx; p_msg.position.y = cy; p_msg.orientation.w = 1.0
                    parking_array.poses.append(p_msg)
                    points_3d = []
                    if way_ref in ways_refs:
                        for nid in ways_refs[way_ref]:
                            if nid in self.nodes:
                                x, y, z = self.nodes[nid]
                                points_3d.append(Point(x=x, y=y, z=z))
                    if len(points_3d)>1:
                        points_3d.append(points_3d[0])
                        m = Marker(); m.header.frame_id = "map"; m.ns = "parking_border"; m.id = self.marker_id_counter; self.marker_id_counter += 1
                        m.type = Marker.LINE_STRIP; m.action = Marker.ADD; m.scale.x = 0.3; m.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=1.0); m.points = points_3d
                        self.marker_array.markers.append(m)
                    if subtype.startswith('parking_'):
                        pk_id = subtype.split('_')[-1]
                        if pk_id not in self.parking_pubs:
                            topic = f'/parking/{pk_id}'
                            self.parking_pubs[pk_id] = self.create_publisher(PoseStamped, topic, self.latching_qos)
                        ps = PoseStamped(); ps.header.frame_id = "map"; ps.pose = p_msg
                        if not hasattr(self, 'parking_msgs'): self.parking_msgs = {}
                        self.parking_msgs[pk_id] = ps
                        tm = Marker(); tm.header.frame_id = "map"; tm.ns = "parking_id"; tm.id = self.marker_id_counter; self.marker_id_counter += 1
                        tm.type = Marker.TEXT_VIEW_FACING; tm.action = Marker.ADD; tm.pose.position.x = cx; tm.pose.position.y = cy; tm.pose.position.z = 1.0
                        tm.scale.z = 1.5; tm.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0); tm.text = f"P-{pk_id}"; self.marker_array.markers.append(tm)

            # [NEW] Extract full parking boundary (Polygon)
            if subtype == 'parking':
                poly = Polygon()
                # Iterate all member ways to build the polygon
                # Note: This simple logic assumes ways are ordered. For complex multipolygons, more logic is needed.
                for m in relation.findall('member'):
                    if m.get('type') == 'way': 
                        way_ref = m.get('ref')
                        # Find the way points
                        pts_list = []
                        if way_ref in ways_refs:
                            for nid in ways_refs[way_ref]:
                                if nid in self.nodes:
                                    pts_list.append(self.nodes[nid])
                        
                        for p in pts_list:
                             p32 = Point32()
                             p32.x = float(p[0])
                             p32.y = float(p[1])
                             p32.z = 0.0
                             poly.points.append(p32)
                
                if len(poly.points) > 2:
                    self.parking_boundary_msg = poly
                    self.get_logger().info(f"Set Parking Boundary with {len(poly.points)} points.")

        self.parking_array_msg = parking_array

        stop_array = PoseArray(); stop_array.header.frame_id = "map"
        for way in root.findall('way'):
            is_stop = False
            for tag in way.findall('tag'):
                if tag.get('v') == 'stop_line': is_stop = True
            if is_stop:
                wid = way.get('id')
                if wid in self.way_points:
                    pts = self.way_points[wid]
                    cx = sum(p[0] for p in pts) / len(pts)
                    cy = sum(p[1] for p in pts) / len(pts)
                    p = PoseStamped().pose; p.position.x = cx; p.position.y = cy; p.orientation.w = 1.0
                    stop_array.poses.append(p)
                    self.get_logger().info(f"Loaded Stop Line {wid} at ({cx:.2f}, {cy:.2f})")
                    points_3d = []
                    if wid in ways_refs:
                        for nid in ways_refs[wid]:
                            if nid in self.nodes:
                                x, y, z = self.nodes[nid]
                                points_3d.append(Point(x=x, y=y, z=z))
                    if len(points_3d) > 1:
                        m = Marker(); m.header.frame_id = "map"; m.ns = "stop_lines"; m.id = self.marker_id_counter; self.marker_id_counter += 1
                        m.type = Marker.LINE_STRIP; m.action = Marker.ADD; m.scale.x = 0.5; m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0); m.points = points_3d
                        self.marker_array.markers.append(m)
        self.stop_line_array_msg = stop_array
        return []

def main(args=None):
    rclpy.init(args=args)
    node = OsmLoader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()