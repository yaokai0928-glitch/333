import pybullet as p
import pybullet_data
import time
import math

# ─── 初始化 ───────────────────────────────────────────
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# ─── 环境 ─────────────────────────────────────────────
p.loadURDF("plane.urdf")
p.loadURDF("table/table.urdf", [0.5, 0, 0], useFixedBase=True)
robot = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0.625], useFixedBase=True)

# ─── 方块（放在桌面上）────────────────────────────────
box_start = [0.5, 0.1, 0.65]
box_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.03, 0.03, 0.03],
                                  rgbaColor=[0.8, 0.2, 0.2, 1])
box_col    = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.03, 0.03, 0.03])
box_id     = p.createMultiBody(baseMass=0.1,
                                baseCollisionShapeIndex=box_col,
                                baseVisualShapeIndex=box_visual,
                                basePosition=box_start)

# ─── 末端执行器 & 夹爪关节索引 ──────────────────────────
EE_LINK      = 11          # panda_hand
FINGER_L     = 9           # panda_finger_joint1
FINGER_R     = 10          # panda_finger_joint2
OPEN_WIDTH   = 0.04        # 半开度（米）
CLOSED_WIDTH = 0.005       # 闭合半开度

def set_fingers(width):
    """width: 单侧开度，0.04 = 全开，0.005 = 闭合"""
    for joint in (FINGER_L, FINGER_R):
        p.setJointMotorControl2(robot, joint, p.POSITION_CONTROL,
                                targetPosition=width, force=40)

def move_to(target_pos, target_orn, steps=200, sleep=1/120):
    """IK 插值到目标位姿，等待到位"""
    for _ in range(steps):
        joints = p.calculateInverseKinematics(robot, EE_LINK, target_pos, target_orn)
        for i in range(7):
            p.setJointMotorControl2(robot, i, p.POSITION_CONTROL,
                                    joints[i], force=300)
        p.stepSimulation()
        time.sleep(sleep)

# ─── 抓取姿态（爪子朝下）─────────────────────────────
down_orn = p.getQuaternionFromEuler([math.pi, 0, 0])

# ─── 阶段参数 ─────────────────────────────────────────
box_pos, _ = p.getBasePositionAndOrientation(box_id)
pre_grasp  = [box_pos[0], box_pos[1], box_pos[2] + 0.15]   # 方块正上方 15 cm
grasp_pos  = [box_pos[0], box_pos[1], box_pos[2] + 0.01]   # 贴近方块
lift_pos   = [box_pos[0], box_pos[1], box_pos[2] + 0.25]   # 抬高
place_pos  = [0.5, -0.2, box_pos[2] + 0.25]                # 放置目标上方
place_down = [0.5, -0.2, box_pos[2] + 0.01]                # 放置高度

# ─── 抓取流程 ─────────────────────────────────────────
print("【1】张开夹爪，移动到预抓取点")
set_fingers(OPEN_WIDTH)
move_to(pre_grasp, down_orn, steps=250)

print("【2】下降到方块")
move_to(grasp_pos, down_orn, steps=150)

print("【3】闭合夹爪")
set_fingers(CLOSED_WIDTH)
for _ in range(120):        # 等待夹爪夹稳
    p.stepSimulation()
    time.sleep(1/120)

print("【4】抬起方块")
move_to(lift_pos, down_orn, steps=200)

print("【5】移动到放置位置上方")
move_to(place_pos, down_orn, steps=250)

print("【6】下降放置")
move_to(place_down, down_orn, steps=150)

print("【7】张开夹爪释放方块")
set_fingers(OPEN_WIDTH)
for _ in range(120):
    p.stepSimulation()
    time.sleep(1/120)

print("【8】退回安全高度")
retreat = [place_down[0], place_down[1], place_down[2] + 0.2]
move_to(retreat, down_orn, steps=150)

print("抓取完成！")

# 保持窗口
while True:
    p.stepSimulation()
    time.sleep(1/120)