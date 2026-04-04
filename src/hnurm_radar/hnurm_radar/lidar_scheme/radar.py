import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from ruamel.yaml import YAML
from detect_result.msg import Robots, Location, Locations
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sklearn.cluster import DBSCAN, KMeans
from scipy.optimize import linear_sum_assignment
from typing import Dict, List, Tuple
import yaml

from ..shared.paths import MAIN_CONFIG_PATH, CONVERTER_CONFIG_PATH
from ..Car.Car import CarList
from ..Lidar.Converter import Converter


# ====================== Tracker ======================
class TrackState:
    Tentative = 0
    Confirmed = 1
    Deleted = 2


class Track:
    def __init__(self, track_id: int, position: np.ndarray, car_id: int):
        self.track_id = track_id
        self.car_id = car_id
        self.position = position.copy()

        self.state = TrackState.Tentative
        self.hits = 1
        self.no_loss = 0
        self.age = 0

    def predict(self):
        self.age += 1

    def update(self, position: np.ndarray, car_id: int):
        self.position = position.copy()
        self.car_id = car_id
        self.hits += 1
        self.no_loss = 0

        if self.state == TrackState.Tentative and self.hits >= 3:
            self.state = TrackState.Confirmed

    def mark_missed(self):
        self.no_loss += 1
        if self.no_loss > 5:
            self.state = TrackState.Deleted


class MultiTargetTracker:
    def __init__(self, max_distance=1.0):
        self.tracks: Dict[int, Track] = {}
        self.next_id = 1
        self.max_distance = max_distance

    def predict(self):
        for track in self.tracks.values():
            track.predict()

    def update(self, detections: List[Tuple[int, np.ndarray]]):
        self.predict()

        if len(self.tracks) == 0:
            for cid, pos in detections:
                self.tracks[self.next_id] = Track(self.next_id, pos, cid)
                self.next_id += 1
            return self._get_outputs()

        track_ids = list(self.tracks.keys())
        n_tracks = len(track_ids)
        n_dets = len(detections)

        if n_dets == 0:
            for tid in track_ids:
                self.tracks[tid].mark_missed()
            self._remove_dead()
            return self._get_outputs()

        INF = 1e9
        cost_matrix = np.full((n_tracks, n_dets), INF, dtype=np.float32)

        for i, tid in enumerate(track_ids):
            track = self.tracks[tid]
            for j, (cid, pos) in enumerate(detections):
                dist = np.linalg.norm(track.position - pos)

                # 距离门控
                if dist < self.max_distance:
                    cost_matrix[i, j] = dist

        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        matched_tracks = set()
        matched_dets = set()

        for r, c in zip(row_ind, col_ind):
            if cost_matrix[r, c] < self.max_distance:
                tid = track_ids[r]
                cid, pos = detections[c]
                self.tracks[tid].update(pos, cid)
                matched_tracks.add(r)
                matched_dets.add(c)

        for i in range(n_tracks):
            if i not in matched_tracks:
                self.tracks[track_ids[i]].mark_missed()

        for j in range(n_dets):
            if j not in matched_dets:
                cid, pos = detections[j]
                self.tracks[self.next_id] = Track(self.next_id, pos, cid)
                self.next_id += 1

        self._remove_dead()
        return self._get_outputs()

    def _remove_dead(self):
        dead_ids = [tid for tid, track in self.tracks.items()
                    if track.state == TrackState.Deleted]
        for tid in dead_ids:
            del self.tracks[tid]

    def _get_outputs(self):
        results = []
        for track in self.tracks.values():
            if track.state == TrackState.Confirmed:
                results.append((track.track_id, track.car_id, track.position))
        return results


# ====================== Radar Node ======================
class Radar(Node):
    def __init__(self):
        super().__init__('Radar')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_lidar_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        main_cfg = YAML().load(open(MAIN_CONFIG_PATH, encoding='utf-8', mode='r'))
        self.global_my_color = main_cfg['global']['my_color']

        self.carList = CarList(main_cfg)
        self.converter = Converter(self.global_my_color, CONVERTER_CONFIG_PATH)

        with open(CONVERTER_CONFIG_PATH, 'r', encoding='utf-8') as f:
            calib = yaml.safe_load(f)

        self.R = np.array(calib['calib']['extrinsic']['R']['data']).reshape(3, 3)
        self.T = np.array(calib['calib']['extrinsic']['T']['data']).reshape(3)

        self.sub_detect = self.create_subscription(
            Robots, 'detect_result', self.radar_callback, qos_profile)
        self.sub_pcd = self.create_subscription(
            PointCloud2, 'target_pointcloud', self.pcd_callback, qos_lidar_profile)

        self.pub_location = self.create_publisher(Locations, 'location', qos_profile)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.on_timer)
        self.radar_to_field = np.eye(4)

        self.lidar_points = None
        self.tracker = MultiTargetTracker(max_distance=1.0)

    # ---------------- TF ----------------
    def on_timer(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'livox', rclpy.time.Time())

            t = transform.transform.translation
            q = transform.transform.rotation

            qx, qy, qz, qw = q.x, q.y, q.z, q.w
            R = np.array([
                [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
                [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
                [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
            ])

            self.radar_to_field = np.eye(4)
            self.radar_to_field[:3, :3] = R
            self.radar_to_field[:3, 3] = [t.x, t.y, t.z]

        except TransformException:
            pass

    # ---------------- PointCloud ----------------
    def pcd_callback(self, msg):
        pts = np.array(
            list(pc2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True
            ))
        )

        if pts.shape[0] == 0:
            self.lidar_points = None
            return

        self.lidar_points = pts[:, :3].astype(np.float32)

    # ⭐ 自适应 DBSCAN
    def cluster_points_dbscan(self, points):
        if points is None or len(points) == 0:
            return []

        dist = np.linalg.norm(points, axis=1)
        mean_dist = np.mean(dist)

        eps = np.clip(0.02 * mean_dist, 0.10, 0.60)

        clustering = DBSCAN(
            eps=eps,
            min_samples=8
        ).fit(points)

        labels = clustering.labels_
        clusters = []

        for label in set(labels):
            if label == -1:
                continue
            clusters.append(points[labels == label])

        return clusters

    def compute_cluster_feature(self, cluster):
        center_lidar = np.mean(cluster, axis=0)

        camera_points = (self.R @ cluster.T).T + self.T
        uvz = self.converter.camera_to_image(camera_points)

        if uvz.shape[0] == 0:
            return None

        pixels = uvz[:, :2]

        return {
            'center_lidar': center_lidar,
            'center_camera': np.mean(camera_points, axis=0),
            'center_pixel': np.mean(pixels, axis=0),
            'pixels': pixels,
            'points': cluster
        }

    # ---------------- Main ----------------
    def radar_callback(self, msg):
        if self.lidar_points is None:
            return

        detect_results = msg.detect_results

        clusters = self.cluster_points_dbscan(self.lidar_points)
        if len(clusters) == 0:
            return

        features = []
        for cluster in clusters:
            feat = self.compute_cluster_feature(cluster)
            if feat is not None:
                features.append(feat)

        if len(features) == 0:
            return

        n_clusters = len(features)
        n_boxes = len(detect_results)

        INF = 1e9
        cost_matrix = np.full((n_clusters, n_boxes), INF, dtype=np.float32)

        for i, feat in enumerate(features):
            pixels = feat['pixels']
            center_pixel = feat['center_pixel']

            for j, det in enumerate(detect_results):
                cx, cy, w, h = det.xywh_box

                x1 = cx - w / 2
                y1 = cy - h / 2
                x2 = cx + w / 2
                y2 = cy + h / 2

                inside = (
                    (pixels[:, 0] >= x1) &
                    (pixels[:, 0] <= x2) &
                    (pixels[:, 1] >= y1) &
                    (pixels[:, 1] <= y2)
                )

                ratio = np.sum(inside) / len(pixels)
                dist = np.linalg.norm(center_pixel - np.array([cx, cy]))

                if ratio > 0.3 and dist < 200:
                    diag = np.sqrt(w*w + h*h)
                    norm_dist = dist / max(diag, 1.0)
                    cost_matrix[i, j] = (1.0 - ratio) + norm_dist

        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        observations = []

        for r, c in zip(row_ind, col_ind):
            if cost_matrix[r, c] >= INF:
                continue

            feat = features[r]
            det = detect_results[c]

            label = det.label
            if label == 'NULL':
                car_id = 0
            else:
                car_id = self.carList.get_car_id(label)

                if self.global_my_color == 'Red':
                    if car_id < 100 and car_id != 7:
                        continue
                else:
                    if car_id > 100 and car_id != 107:
                        continue

            center_h = np.append(feat['center_lidar'], 1.0)
            field_xyz = (self.radar_to_field @ center_h)[:3]

            observations.append((car_id, field_xyz))

        # ⭐ track_id tracker
        tracked = self.tracker.update(observations)

        carlist_results = []
        null_locations = []

        for track_id, car_id, pos in tracked:
            if car_id == 0:
                null_locations.append(pos)
            else:
                carlist_results.append([
                    track_id,
                    car_id,
                    [0, 0, 0, 0],
                    1,
                    [0, 0, 0],
                    pos
                ])

        self.carList.update_car_info(carlist_results)
        infos = self.carList.get_all_info()

        msg_out = Locations()

        for info in infos:
            track_id, car_id, center_xy, camera_xyz, field_xyz, color, valid = info

            if color != self.global_my_color and track_id != -1:
                loc = Location()
                loc.x = float(field_xyz[0])
                loc.y = float(field_xyz[1])
                loc.z = float(field_xyz[2])
                loc.id = car_id
                loc.label = color
                msg_out.locs.append(loc)

        for pos in null_locations:
            loc = Location()
            loc.x = float(pos[0])
            loc.y = float(pos[1])
            loc.z = float(pos[2])
            loc.id = 0
            loc.label = 'NULL'
            msg_out.locs.append(loc)

        self.pub_location.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = Radar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

