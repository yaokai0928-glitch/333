import pybullet as p
import pybullet_data as pd
import time
import math

# --- 1. 环境初始化 ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
p.setGravity(0, 0, -9.8)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.resetDebugVisualizerCamera(1.5, 45, -30, [0.5, 0, 0.65])

# 加载地面和桌子
p.loadURDF("plane.urdf")
table_pos = [0.5, 0, 0]
p.loadURDF("table/table.urdf", table_pos, useFixedBase=True)

# 加载 Panda 机械臂
panda_pos = [0.5, 0, 0.625]
pandaId = p.loadURDF("franka_panda/panda.urdf", panda_pos, useFixedBase=True)

# --- 2. 加载方块 ---
cube_start_pos = [0.6, 0, 0.65]  # 桌面上方
cube_start_orientation = p.getQuaternionFromEuler([0,0,0])
cubeId = p.loadURDF("cube_small.urdf", cube_start_pos, cube_start_orientation)

# --- 3. 夹爪控制函数 ---
def control_gripper(open=True):
    finger_angle = 0.04 if open else 0.0
    p.setJointMotorControl2(pandaId, 9, p.POSITION_CONTROL, finger_angle, force=50)
    p.setJointMotorControl2(pandaId, 10, p.POSITION_CONTROL, finger_angle, force=50)

# --- 4. IK 控制函数 ---
def move_ee(target_pos, target_ori=[math.pi,0,0]):
    joint_poses = p.calculateInverseKinematics(pandaId, 11, target_pos,
                                               p.getQuaternionFromEuler(target_ori))
    for i in range(7):
        p.setJointMotorControl2(pandaId, i, p.POSITION_CONTROL, joint_poses[i], force=500)

# --- 5. 抓取逻辑 ---
try:
    control_gripper(open=True)  # 张开夹爪

    # 1️⃣ 移动到方块上方
    for _ in range(300):
        target_pos = [cube_start_pos[0], cube_start_pos[1], cube_start_pos[2]+0.15]
        move_ee(target_pos)
        p.stepSimulation()
        time.sleep(1./240.)

    # 2️⃣ 下移靠近方块
    for _ in range(300):
        target_pos = [cube_start_pos[0], cube_start_pos[1], cube_start_pos[2]+0.02]
        move_ee(target_pos)
        p.stepSimulation()
        time.sleep(1./240.)

    # 3️⃣ 闭合夹爪抓住方块
    control_gripper(open=False)
    for _ in range(200):
        p.stepSimulation()
        time.sleep(1./240.)

    # 4️⃣ 创建约束，把方块绑定到末端模拟抓取
    cid = p.createConstraint(
        parentBodyUniqueId=pandaId,
        parentLinkIndex=11,  # Panda 末端
        childBodyUniqueId=cubeId,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0,0,0],
        parentFramePosition=[0,0,0],
        childFramePosition=[0,0,0]
    )

    # 5️⃣ 抬起方块
    for _ in range(300):
        target_pos = [cube_start_pos[0], cube_start_pos[1], cube_start_pos[2]+0.25]
        move_ee(target_pos)
        p.stepSimulation()
        time.sleep(1./240.)

    print("抓取完成！")

    # 保持显示
    for _ in range(1000):
        p.stepSimulation()
        time.sleep(1./240.)

except Exception as e:
    print(f"Error occurred: {e}")
finally:
    p.disconnect()