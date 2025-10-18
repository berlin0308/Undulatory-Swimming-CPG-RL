# 01_urdf_sph_setup.py
import os
import time
import json
import numpy as np
import genesis as gs
import xml.etree.ElementTree as ET

# ---------- user knobs ----------
URDF_PATH = os.environ.get("EEL_URDF", "/home/nas/polin/cmu-berlin/quadruped-loco-cpg-rl/env/assem_description/urdf/assem_description.urdf")  # update to your file
USE_GPU = False
SIM_DT = 4e-3
SUBSTEPS = 10
WATER_BOUNDS = (-1.0, -0.5, 0.0), (1.0, 0.5, 1.0)  # (lower_bound, upper_bound)
PARTICLE_SIZE = 0.01
RUN_STEPS = 50  # 减少到50步，大约运行几秒钟
LOG_EVERY = 10
ADD_VIS = True

# ---------- hydrodynamic force proxy knobs (tune as needed) ----------
RHO_WATER = 1000.0          # kg/m^3
CD_LATERAL = 1.5            # dimensionless drag coefficient (lateral)
AREA_LATERAL = 0.01         # m^2 effective cross-section area per sensed link
FORCE_LPF_ALPHA = 0.8       # 0..1, higher = smoother; simple IIR low-pass filter

# ---------- init ----------
gs.init(backend=gs.gpu if USE_GPU else gs.cpu)  # viewer enabled below
scene = gs.Scene(
    sim_options=gs.options.SimOptions(dt=SIM_DT, substeps=SUBSTEPS, gravity=(0, 0, -9.81)),
    sph_options=gs.options.SPHOptions(
        lower_bound=WATER_BOUNDS[0],
        upper_bound=WATER_BOUNDS[1],
        particle_size=PARTICLE_SIZE,
    ),
    vis_options=gs.options.VisOptions(visualize_sph_boundary=True),
    viewer_options=gs.options.ViewerOptions(camera_pos=(2.5, 0.0, 1.0), camera_lookat=(0.0, 0.0, 0.2)),
    show_viewer=False,
)

# ground
scene.add_entity(gs.morphs.Plane())

# a water block
liquid = scene.add_entity(
    material=gs.materials.SPH.Liquid(),  # viscosity/surface tension can be tuned later
    morph=gs.morphs.Box(pos=(0.0, 0.0, 0.55), size=(0.8, 0.3, 0.6)),
    surface=gs.surfaces.Default(color=(0.4, 0.8, 1.0), vis_mode="particle"),
)

# ---------- load eel URDF ----------
# NOTE: URDF base is "free" by default. Use fixed=True if you want the base welded to world
# 暂时使用简单的盒子替代有问题的 URDF
eel = scene.add_entity(
    material=gs.materials.Rigid(),
    morph=gs.morphs.Box(pos=(0.0, 0.0, 0.20), size=(0.1, 0.05, 0.02)),
    surface=gs.surfaces.Default(color=(0.0, 1.0, 0.0)),
)

# ---------- headless camera (for video) ----------
# Add an offscreen camera for rendering frames; GUI=False avoids OpenCV windows
cam = scene.add_camera(
    res=(640, 480),
    pos=(2.5, 0.0, 1.2),
    lookat=(0.0, 0.0, 0.2),
    fov=45,
    GUI=False,
)

scene.build()  # build once to initialize eel and finalize scene graph

# ---------- 使用简单的实体替代 URDF 链接 ----------
# 由于我们使用简单的盒子而不是 URDF，我们直接使用实体本身
head = eel  # 对于简单实体，我们使用实体本身作为"头部"
print(f"[INFO] 使用简单盒子实体作为 IMU 传感器位置")

# ---------- helper: get link pose (pos, rot) with fallbacks ----------
def _quat_to_rot(q):
    # q = [x, y, z, w] or [w, x, y, z]; try to detect by length of last element
    q = np.array(q, dtype=float).flatten()
    if q.shape[0] != 4:
        raise ValueError("Quaternion must have 4 elements")
    # Heuristic: if |q[-1]| <= 1 and |q[0]| > 1, assume [x,y,z,w] already
    # We'll assume common [x,y,z,w]
    x, y, z, w = q[0], q[1], q[2], q[3]
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    R = np.array([
        [1 - 2*(yy + zz),     2*(xy - wz),       2*(xz + wy)],
        [2*(xy + wz),         1 - 2*(xx + zz),   2*(yz - wx)],
        [2*(xz - wy),         2*(yz + wx),       1 - 2*(xx + yy)],
    ])
    return R

def _get_entity_pose(entity):
    # 获取简单实体的姿态
    # Returns (pos: np.ndarray shape (3,), rot: np.ndarray shape (3,3))
    try:
        # 尝试获取实体的世界位置和旋转
        if hasattr(entity, "world_pos") and hasattr(entity, "world_quat"):
            pos = np.array(entity.world_pos, dtype=float)
            quat = np.array(entity.world_quat, dtype=float)
            if pos.shape[0] == 3 and quat.shape[0] == 4:
                return pos, _quat_to_rot(quat)
        
        # 尝试其他可能的属性名
        for pos_attr in ("pos", "position", "world_position"):
            for quat_attr in ("quat", "orientation", "rot_quat", "world_orientation"):
                if hasattr(entity, pos_attr) and hasattr(entity, quat_attr):
                    pos = np.array(getattr(entity, pos_attr), dtype=float)
                    quat = np.array(getattr(entity, quat_attr), dtype=float)
                    if pos.shape[0] == 3 and quat.shape[0] == 4:
                        return pos, _quat_to_rot(quat)
        
        # 尝试方法调用
        for fn in ("get_pose", "world_pose", "pose", "get_world_pose"):
            if hasattr(entity, fn):
                out = getattr(entity, fn)()
                if isinstance(out, (tuple, list)) and len(out) == 2:
                    pos, quat = np.array(out[0], float), np.array(out[1], float)
                    if pos.shape[0] == 3 and quat.shape[0] == 4:
                        return pos, _quat_to_rot(quat)
        
        # 如果都失败了，返回默认值
        print("[WARN] 无法获取实体姿态，使用默认值")
        return np.array([0.0, 0.0, 0.2], dtype=float), np.eye(3)
        
    except Exception as e:
        print(f"[WARN] 获取实体姿态时出错: {e}")
        return np.array([0.0, 0.0, 0.2], dtype=float), np.eye(3)

# ---------- sensors ----------
# IMU on the head (good for diagnosing stability & disturbances)
try:
    imu = scene.add_sensor(
        gs.sensors.IMUOptions(
            entity_idx=eel.idx,
            link_idx_local=0,  # 对于简单实体，使用索引 0
            acc_noise_std=(0.02, 0.02, 0.02),
            gyro_noise_std=(0.01, 0.01, 0.01),
            delay=0.005,
            jitter=0.002,
            interpolate_for_delay=True,
        )
    )
    IMU_AVAILABLE = True
    print("[INFO] IMU 传感器添加成功")
except AttributeError:
    print(
        "[WARN] scene.add_sensor is unavailable in this Genesis version. "
        "Skipping IMU. Consider upgrading 'genesis-world' (Python >=3.10) for sensor APIs."
    )
    imu = None
    IMU_AVAILABLE = False
except Exception as e:
    print(f"[WARN] IMU 传感器添加失败: {e}")
    imu = None
    IMU_AVAILABLE = False

# NOTE on “hydrodynamic force sensing”:
# Genesis exposes IMU + rigid tactile sensors today. Direct per-link fluid drag
# isn’t a built-in sensor primitive yet; for hydrodynamic feedback you can:
#   (A) Estimate drag forces from link velocities (proxy model) — see Script 2
#   (B) Instrument contact/tactile on proxy “pads” if you use obstacles/surfaces
#   (C) Record IMU + body velocities and infer lateral/longitudinal damping
# See docs: sensors page (IMU/tactile), SPH fluids, URDF loading.  [oai_citation:1‡Genesis Documentation](https://genesis-world.readthedocs.io/en/latest/user_guide/getting_started/sensors.html)

# ---------- build & run ----------
# scene.build()

log = {"t": [], "imu": [], "force": []}

# state for force estimation
FORCE_AVAILABLE = True
prev_pos = None
f_left_f = 0.0
f_right_f = 0.0

# prepare output dir and start camera recording
os.makedirs("videos", exist_ok=True)
cam.start_recording()

for step in range(RUN_STEPS):
    scene.step()
    # render a frame each step to record video (rgb by default)
    cam.render()

    if step % LOG_EVERY == 0:
        log["t"].append(step * SIM_DT)
        if IMU_AVAILABLE:
            gt = imu.read_ground_truth()  # {'lin_acc': (3,), 'ang_vel': (3,)}
            log["imu"].append({"lin_acc": list(gt["lin_acc"]), "ang_vel": list(gt["ang_vel"])})
        else:
            log["imu"].append(None)

        # Estimate lateral hydrodynamic force split into left/right
        if FORCE_AVAILABLE:
            try:
                pos, R = _get_entity_pose(head)
                if prev_pos is None:
                    v = np.zeros(3, dtype=float)
                else:
                    v = (pos - prev_pos) / (LOG_EVERY * SIM_DT)
                prev_pos = pos.copy()

                y_body = R[:, 1]  # lateral axis
                v_lat = float(np.dot(v, y_body))
                F_lat = 0.5 * RHO_WATER * CD_LATERAL * AREA_LATERAL * abs(v_lat) * v_lat
                F_left = max(F_lat, 0.0)
                F_right = max(-F_lat, 0.0)

                # simple IIR low-pass filtering
                f_left_f = FORCE_LPF_ALPHA * f_left_f + (1.0 - FORCE_LPF_ALPHA) * F_left
                f_right_f = FORCE_LPF_ALPHA * f_right_f + (1.0 - FORCE_LPF_ALPHA) * F_right

                log["force"].append({
                    "left": float(f_left_f),
                    "right": float(f_right_f),
                    "raw_lat": float(F_lat),
                })
            except Exception as _e:
                if FORCE_AVAILABLE:  # print once
                    print(f"[WARN] Force proxy disabled: {_e}")
                FORCE_AVAILABLE = False
                log["force"].append(None)
        else:
            log["force"].append(None)

# save quick log
os.makedirs("logs", exist_ok=True)
with open("logs/imu_log.json", "w") as f:
    json.dump(log, f, indent=2)

print("Saved IMU log to logs/imu_log.json")

# stop and save the recorded video
cam.stop_recording(save_to_filename="videos/simulation.mp4", fps=int(1.0 / SIM_DT))
print("Saved video to videos/simulation.mp4")