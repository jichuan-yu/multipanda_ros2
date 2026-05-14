#!/usr/bin/env python3
import numpy as np
import open3d as o3d
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import os
import copy

# ===================== 配置参数 =====================
BAG_PATH = "/home/yuyang/my_pointcloud_data"
TOPIC = "/camera/camera/depth/color/points"

# 第一帧手动包围盒（支架初始区域）
INIT_BBOX = {
    'x': (-0.35, 0.36),
    'y': (-0.33, 0.42),
    'z': (0.00, 0.50)
}

# 全局 ROI 边界（以第一帧支架为中心，放大若干倍，滤除远处噪声）
# 方法：根据 INIT_BBOX 自动生成，也可手动指定
GLOBAL_ROI_EXPAND = 3.0   # 扩大倍数（边长放大3倍）
# 手动指定（可选，优先级高于自动）
GLOBAL_ROI_MANUAL = None   # 例如 {'x': (-2,2), 'y': (-2,2), 'z': (0,2)}

# 预处理参数
VOXEL_SIZE = 0.005
NB_NEIGHBORS = 20
STD_RATIO = 2.0

# ICP 参数
ICP_MAX_CORRESPONDENCE_DISTANCE = 0.05
ICP_MAX_ITERATION = 50

# 候选区域扩大因子（用于寻找ICP对应点）
SEARCH_EXPAND_FACTOR = 2.5

# 输出目录
OUTPUT_DIR = "/home/yuyang/icp_stable_roi"
os.makedirs(OUTPUT_DIR, exist_ok=True)

VISUALIZE_INTERVAL = 20
# ===================================================

# 自动生成全局 ROI（基于第一帧包围盒）
def compute_global_roi():
    if GLOBAL_ROI_MANUAL is not None:
        return GLOBAL_ROI_MANUAL
    # 计算初始包围盒中心及边长
    cx = (INIT_BBOX['x'][0] + INIT_BBOX['x'][1]) / 2
    cy = (INIT_BBOX['y'][0] + INIT_BBOX['y'][1]) / 2
    cz = (INIT_BBOX['z'][0] + INIT_BBOX['z'][1]) / 2
    len_x = INIT_BBOX['x'][1] - INIT_BBOX['x'][0]
    len_y = INIT_BBOX['y'][1] - INIT_BBOX['y'][0]
    len_z = INIT_BBOX['z'][1] - INIT_BBOX['z'][0]
    new_len_x = len_x * GLOBAL_ROI_EXPAND
    new_len_y = len_y * GLOBAL_ROI_EXPAND
    new_len_z = len_z * GLOBAL_ROI_EXPAND
    return {
        'x': (cx - new_len_x/2, cx + new_len_x/2),
        'y': (cy - new_len_y/2, cy + new_len_y/2),
        'z': (cz - new_len_z/2, cz + new_len_z/2)
    }

GLOBAL_ROI = compute_global_roi()
print(f"全局 ROI 范围: X{GLOBAL_ROI['x']}, Y{GLOBAL_ROI['y']}, Z{GLOBAL_ROI['z']}")

def load_pointclouds_generator(bag_path, topic):
    storage = StorageOptions(uri=bag_path, storage_id='sqlite3')
    conv = ConverterOptions('cdr', 'cdr')
    reader = SequentialReader()
    reader.open(storage, conv)
    topic_types = {info.name: info.type for info in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        raise RuntimeError(f"话题 {topic} 不存在")
    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()
        if topic_name == topic:
            msg = deserialize_message(data, PointCloud2)
            points = []
            for p in pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True):
                points.append([p[0], p[1], p[2]])
            if not points:
                continue
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(np.array(points, dtype=np.float64))
            yield timestamp, pcd

def global_roi_filter(pcd, roi):
    """对原始点云进行全局 ROI 裁剪，提升效率"""
    pts = np.asarray(pcd.points)
    mask = (pts[:,0] > roi['x'][0]) & (pts[:,0] < roi['x'][1]) & \
           (pts[:,1] > roi['y'][0]) & (pts[:,1] < roi['y'][1]) & \
           (pts[:,2] > roi['z'][0]) & (pts[:,2] < roi['z'][1])
    if np.sum(mask) == 0:
        return None
    filtered = o3d.geometry.PointCloud()
    filtered.points = o3d.utility.Vector3dVector(pts[mask])
    return filtered

def preprocess(pcd):
    pcd_down = pcd.voxel_down_sample(VOXEL_SIZE)
    cl, _ = pcd_down.remove_statistical_outlier(nb_neighbors=NB_NEIGHBORS, std_ratio=STD_RATIO)
    return cl

def extract_by_box(pcd, bbox):
    if pcd is None or len(pcd.points) == 0:
        return None
    pts = np.asarray(pcd.points)
    mask = (pts[:,0] > bbox['x'][0]) & (pts[:,0] < bbox['x'][1]) & \
           (pts[:,1] > bbox['y'][0]) & (pts[:,1] < bbox['y'][1]) & \
           (pts[:,2] > bbox['z'][0]) & (pts[:,2] < bbox['z'][1])
    if np.sum(mask) == 0:
        return None
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(pts[mask])
    return cloud

def expand_bbox(center, base_bbox, expand_factor):
    len_x = base_bbox['x'][1] - base_bbox['x'][0]
    len_y = base_bbox['y'][1] - base_bbox['y'][0]
    len_z = base_bbox['z'][1] - base_bbox['z'][0]
    new_len_x = len_x * expand_factor
    new_len_y = len_y * expand_factor
    new_len_z = len_z * expand_factor
    return {
        'x': (center[0] - new_len_x/2, center[0] + new_len_x/2),
        'y': (center[1] - new_len_y/2, center[1] + new_len_y/2),
        'z': (center[2] - new_len_z/2, center[2] + new_len_z/2)
    }

def icp_transform(source, target, max_dist, max_iter):
    if source is None or target is None or len(source.points)==0 or len(target.points)==0:
        return source
    source_down = source.voxel_down_sample(0.01)
    target_down = target.voxel_down_sample(0.01)
    if len(source_down.points) < 3 or len(target_down.points) < 3:
        return source
    reg = o3d.pipelines.registration.registration_icp(
        source_down, target_down, max_dist, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)
    )
    transformed = copy.deepcopy(source)
    transformed.transform(reg.transformation)
    return transformed

def main():
    print("===== ICP 稳定追踪 + 全局 ROI 加速 =====")
    print(f"全局 ROI: {GLOBAL_ROI}")
    frame_idx = 0
    prev_stand = None
    prev_center = None

    for ts, pcd_raw in load_pointclouds_generator(BAG_PATH, TOPIC):
        # 步骤1：全局 ROI 裁剪（大幅减少点数）
        pcd_roi = global_roi_filter(pcd_raw, GLOBAL_ROI)
        if pcd_roi is None or len(pcd_roi.points) == 0:
            print(f"帧 {frame_idx}: ROI 内无点，跳过")
            frame_idx += 1
            continue

        # 步骤2：预处理
        pcd_clean = preprocess(pcd_roi)
        if pcd_clean is None or len(pcd_clean.points) == 0:
            print(f"帧 {frame_idx}: 预处理后无点，跳过")
            frame_idx += 1
            continue

        # 第一帧提取
        if frame_idx == 0:
            stand = extract_by_box(pcd_clean, INIT_BBOX)
            if stand is None or len(stand.points) < 20:
                print("第一帧提取失败，请调整初始包围盒")
                break
            prev_stand = stand
            prev_center = np.mean(np.asarray(stand.points), axis=0)
            o3d.io.write_point_cloud(f"{OUTPUT_DIR}/frame_{frame_idx:04d}_stand.ply", stand)
            print(f"帧 {frame_idx}: 初始提取，点数 {len(stand.points)}")
            if VISUALIZE_INTERVAL > 0:
                pcd_clean.paint_uniform_color([0.7,0.7,0.7])
                stand.paint_uniform_color([1,0,0])
                o3d.visualization.draw_geometries([pcd_clean, stand], window_name=f"Frame {frame_idx}")
            frame_idx += 1
            continue

        # 后续帧：ICP 追踪
        search_bbox = expand_bbox(prev_center, INIT_BBOX, SEARCH_EXPAND_FACTOR)
        candidate = extract_by_box(pcd_clean, search_bbox)
        if candidate is None or len(candidate.points) < 100:
            print(f"帧 {frame_idx}: 候选区域点云不足({len(candidate.points) if candidate else 0})，使用上一帧支架")
            stand = prev_stand
        else:
            stand = icp_transform(prev_stand, candidate,
                                  ICP_MAX_CORRESPONDENCE_DISTANCE,
                                  ICP_MAX_ITERATION)
            if stand is not None and len(stand.points) > 0:
                prev_center = np.mean(np.asarray(stand.points), axis=0)

        if stand is not None and len(stand.points) > 0:
            o3d.io.write_point_cloud(f"{OUTPUT_DIR}/frame_{frame_idx:04d}_stand.ply", stand)
            print(f"帧 {frame_idx}: ICP变换，点数 {len(stand.points)}")
            prev_stand = stand
        else:
            print(f"帧 {frame_idx}: 提取失败，保留上一帧模板")

        if VISUALIZE_INTERVAL > 0 and frame_idx % VISUALIZE_INTERVAL == 0 and stand is not None:
            pcd_clean.paint_uniform_color([0.7,0.7,0.7])
            stand.paint_uniform_color([1,0,0])
            o3d.visualization.draw_geometries([pcd_clean, stand], window_name=f"Frame {frame_idx}")

        frame_idx += 1

    print("\n追踪完成。")

if __name__ == "__main__":
    main()