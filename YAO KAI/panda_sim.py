import pybullet as p
import pybullet_data
import time
import math

# 初始化
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)

# 环境
p.loadURDF("plane.urdf")
p.loadURDF("table/table.urdf",[0.5,0,0],useFixedBase=True)
robot = p.loadURDF("franka_panda/panda.urdf",[0.5,0,0.625],useFixedBase=True)

# 圆参数
center = [0.6, 0.0, 0.8]
radius = 0.15

t = 0

prev_pos = None

while True:
    # 圆轨迹
    x = center[0] + radius * math.cos(t)
    y = center[1] + radius * math.sin(t)
    z = center[2]

    target = [x,y,z]

    # IK
    joint_angles = p.calculateInverseKinematics(
        robot,
        11,
        target,
        p.getQuaternionFromEuler([math.pi,0,0])
    )

    for i in range(7):
        p.setJointMotorControl2(robot,i,p.POSITION_CONTROL,joint_angles[i],force=300)

    # 画轨迹（加分🔥）
    if prev_pos:
        p.addUserDebugLine(prev_pos, target, [1,0,0], 2)
    prev_pos = target

    t += 0.02

    p.stepSimulation()
    time.sleep(1/120)