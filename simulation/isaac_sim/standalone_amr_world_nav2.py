import os
import sys
from pathlib import Path

# Isaac Sim ROS 2 Bridge의 내장 Jazzy 라이브러리는 프로세스 시작 전에
# LD_LIBRARY_PATH에 있어야 한다. 누락된 경우 환경을 보정해 한 번 재실행한다.
ISAAC_SIM_ROOT = Path(sys.executable).resolve().parents[3]
ROS_DISTRO = os.environ.get("ROS_DISTRO", "jazzy")
ROS2_LIB_PATH = ISAAC_SIM_ROOT / "exts" / "isaacsim.ros2.core" / ROS_DISTRO / "lib"

if not ROS2_LIB_PATH.is_dir():
    raise RuntimeError(f"Isaac Sim ROS 2 libraries not found: {ROS2_LIB_PATH}")

library_paths = os.environ.get("LD_LIBRARY_PATH", "").split(":")
if str(ROS2_LIB_PATH) not in library_paths:
    environment = os.environ.copy()
    environment["ROS_DISTRO"] = ROS_DISTRO
    environment.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    environment["LD_LIBRARY_PATH"] = ":".join(
        [str(ROS2_LIB_PATH), *filter(None, library_paths)]
    )
    os.execve(sys.executable, [sys.executable, *sys.argv], environment)


from isaacsim import SimulationApp

# =========================================================
# 1. Isaac Sim 실행
# =========================================================

simulation_app = SimulationApp({
    "headless": False
})


# =========================================================
# 2. ROS 2 Bridge 활성화
#
# Standalone Python에서는 ROS 2 OmniGraph 노드를 사용하기 전에
# ROS 2 Bridge extension을 명시적으로 활성화한다.
# =========================================================

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()


# SimulationApp 이후에 Isaac Sim / Omniverse 모듈 import
import numpy as np

import omni.graph.core as og
import omni.usd

from pxr import Sdf

from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid

from isaacsim.robot.wheeled_robots.robots import WheeledRobot

from isaacsim.storage.native import get_assets_root_path


# =========================================================
# 3. 환경 설정
# =========================================================

WORLD_SIZE_X = 20.0
WORLD_SIZE_Y = 12.0
WALL_HEIGHT = 2.5


# =========================================================
# 4. 주요 위치
#
# 이 좌표는 향후 AMR Node가 NavigateToPose Goal을 보낼 때
# map frame 기준 좌표와 동일하게 사용한다.
# =========================================================

LOCATIONS = {
    "start": np.array(
        [0.0, 0.0, 0.0],
        dtype=np.float32,
    ),

    "supermarket": np.array(
        [-7.0, 0.0, 0.0],
        dtype=np.float32,
    ),

    "cell_a": np.array(
        [7.0, 3.5, 0.0],
        dtype=np.float32,
    ),

    "cell_b": np.array(
        [7.0, 0.0, 0.0],
        dtype=np.float32,
    ),

    "cell_c": np.array(
        [7.0, -3.5, 0.0],
        dtype=np.float32,
    ),
}


# =========================================================
# 5. World 생성
# =========================================================

world = World(
    stage_units_in_meters=1.0,
    physics_dt=1.0 / 60.0,
    rendering_dt=1.0 / 60.0,
)


# =========================================================
# 6. Box 생성 함수
# =========================================================

def add_box(
    prim_path,
    name,
    position,
    scale,
    color,
):
    obj = FixedCuboid(
        prim_path=prim_path,
        name=name,
        position=np.array(
            position,
            dtype=np.float32,
        ),
        scale=np.array(
            scale,
            dtype=np.float32,
        ),
        color=np.array(
            color,
            dtype=np.float32,
        ),
    )

    world.scene.add(obj)

    return obj


# =========================================================
# 7. 바닥
# =========================================================

add_box(
    prim_path="/World/Environment/Floor",
    name="floor",

    position=[
        0.0,
        0.0,
        -0.05,
    ],

    scale=[
        WORLD_SIZE_X,
        WORLD_SIZE_Y,
        0.1,
    ],

    color=[
        0.65,
        0.65,
        0.65,
    ],
)


# =========================================================
# 8. 벽
# =========================================================

WALL_COLOR = [
    0.78,
    0.78,
    0.78,
]


add_box(
    prim_path="/World/Environment/Wall_Left",
    name="wall_left",

    position=[
        -WORLD_SIZE_X / 2,
        0.0,
        WALL_HEIGHT / 2,
    ],

    scale=[
        0.1,
        WORLD_SIZE_Y,
        WALL_HEIGHT,
    ],

    color=WALL_COLOR,
)


add_box(
    prim_path="/World/Environment/Wall_Right",
    name="wall_right",

    position=[
        WORLD_SIZE_X / 2,
        0.0,
        WALL_HEIGHT / 2,
    ],

    scale=[
        0.1,
        WORLD_SIZE_Y,
        WALL_HEIGHT,
    ],

    color=WALL_COLOR,
)


add_box(
    prim_path="/World/Environment/Wall_Top",
    name="wall_top",

    position=[
        0.0,
        WORLD_SIZE_Y / 2,
        WALL_HEIGHT / 2,
    ],

    scale=[
        WORLD_SIZE_X,
        0.1,
        WALL_HEIGHT,
    ],

    color=WALL_COLOR,
)


add_box(
    prim_path="/World/Environment/Wall_Bottom",
    name="wall_bottom",

    position=[
        0.0,
        -WORLD_SIZE_Y / 2,
        WALL_HEIGHT / 2,
    ],

    scale=[
        WORLD_SIZE_X,
        0.1,
        WALL_HEIGHT,
    ],

    color=WALL_COLOR,
)


# =========================================================
# 9. Supermarket
# =========================================================

add_box(
    prim_path="/World/Areas/Supermarket",
    name="supermarket_area",

    position=[
        LOCATIONS["supermarket"][0],
        LOCATIONS["supermarket"][1],
        0.01,
    ],

    scale=[
        3.0,
        8.0,
        0.02,
    ],

    color=[
        0.50,
        0.20,
        0.70,
    ],
)


# =========================================================
# 10. Cell A
# =========================================================

add_box(
    prim_path="/World/Areas/Cell_A",
    name="cell_a_area",

    position=[
        LOCATIONS["cell_a"][0],
        LOCATIONS["cell_a"][1],
        0.01,
    ],

    scale=[
        3.0,
        2.5,
        0.02,
    ],

    color=[
        0.10,
        0.35,
        0.90,
    ],
)


# =========================================================
# 11. Cell B
# =========================================================

add_box(
    prim_path="/World/Areas/Cell_B",
    name="cell_b_area",

    position=[
        LOCATIONS["cell_b"][0],
        LOCATIONS["cell_b"][1],
        0.01,
    ],

    scale=[
        3.0,
        2.5,
        0.02,
    ],

    color=[
        0.95,
        0.80,
        0.10,
    ],
)


# =========================================================
# 12. Cell C
# =========================================================

add_box(
    prim_path="/World/Areas/Cell_C",
    name="cell_c_area",

    position=[
        LOCATIONS["cell_c"][0],
        LOCATIONS["cell_c"][1],
        0.01,
    ],

    scale=[
        3.0,
        2.5,
        0.02,
    ],

    color=[
        0.15,
        0.70,
        0.25,
    ],
)


# =========================================================
# 13. Nova Carter Asset
# =========================================================

assets_root_path = get_assets_root_path()

if assets_root_path is None:
    raise RuntimeError(
        "Isaac Sim Asset 경로를 찾을 수 없습니다."
    )


nova_carter_usd = (
    assets_root_path
    + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
)

print("")
print("Nova Carter USD:")
print(nova_carter_usd)
print("")


# =========================================================
# 14. Nova Carter 생성
# =========================================================

AMR_PRIM = "/World/AMR"

amr = world.scene.add(

    WheeledRobot(
        prim_path=AMR_PRIM,

        name="nova_carter",

        # Nova Carter 구동 바퀴 Joint
        wheel_dof_names=[
            "joint_wheel_left",
            "joint_wheel_right",
        ],

        create_robot=True,

        usd_path=nova_carter_usd,

        # 중앙에서 시작
        position=np.array(
            [0.0, 0.0, 0.05],
            dtype=np.float32,
        ),
    )
)


# =========================================================
# 15. World 초기화
# =========================================================

world.reset()


# =========================================================
# 16. 실제 DOF 이름 확인
# =========================================================

print("")
print("=" * 60)
print("Nova Carter DOF")
print("=" * 60)

for index, dof_name in enumerate(amr.dof_names):
    print(
        f"{index:2d} : {dof_name}"
    )

print("=" * 60)
print("")


# =========================================================
# 17. Nova Carter chassis prim 확인
#
# Nova Carter 기본 USD에서는 chassis_link가 로봇의 차체 프림이다.
# 만약 asset 구조가 달라졌다면 여기서 즉시 오류를 내어
# 잘못된 TF/Odometry가 조용히 생성되는 것을 막는다.
# =========================================================

stage = omni.usd.get_context().get_stage()

CHASSIS_PRIM = f"{AMR_PRIM}/chassis_link"

if not stage.GetPrimAtPath(CHASSIS_PRIM).IsValid():
    raise RuntimeError(
        f"Nova Carter chassis prim을 찾을 수 없습니다: {CHASSIS_PRIM}"
    )

print("")
print("=" * 60)
print(" NAV2 ROS 2 INTERFACE")
print("=" * 60)
print(f"Robot prim   : {AMR_PRIM}")
print(f"Chassis prim : {CHASSIS_PRIM}")
print("cmd_vel      : /cmd_vel")
print("Odometry     : /amr/odom")
print("TF           : map -> odom -> base_link")
print("Clock        : /clock")
print("=" * 60)
print("")


# =========================================================
# 18. /cmd_vel -> Nova Carter Wheel Action Graph
#
# Nav2 controller_server가 발행하는 geometry_msgs/Twist를
# Isaac Sim이 구독한다.
#
# /cmd_vel
#    ↓
# ROS2 Subscribe Twist
#    ↓
# Differential Controller
#    ↓
# Articulation Controller
#    ↓
# Nova Carter left/right wheel
# =========================================================

CMD_VEL_GRAPH_PATH = "/World/ROS2_AMR_CmdVel_Graph"

if stage.GetPrimAtPath(CMD_VEL_GRAPH_PATH).IsValid():
    stage.RemovePrim(CMD_VEL_GRAPH_PATH)


keys = og.Controller.Keys

og.Controller.edit(
    {
        "graph_path": CMD_VEL_GRAPH_PATH,
        "evaluator_name": "execution",
    },
    {
        keys.CREATE_NODES: [

            (
                "OnPlaybackTick",
                "omni.graph.action.OnPlaybackTick",
            ),

            (
                "Context",
                "isaacsim.ros2.bridge.ROS2Context",
            ),

            (
                "SubscribeTwist",
                "isaacsim.ros2.bridge.ROS2SubscribeTwist",
            ),

            (
                "BreakLinVel",
                "omni.graph.nodes.BreakVector3",
            ),

            (
                "BreakAngVel",
                "omni.graph.nodes.BreakVector3",
            ),

            (
                "DiffController",
                "isaacsim.robot.wheeled_robots.DifferentialController",
            ),

            (
                "ArtController",
                "isaacsim.core.nodes.IsaacArticulationController",
            ),
        ],

        keys.SET_VALUES: [

            # 현재 shell의 ROS_DOMAIN_ID 사용
            (
                "Context.inputs:useDomainIDEnvVar",
                True,
            ),

            # Nav2 표준 속도 명령
            (
                "SubscribeTwist.inputs:topicName",
                "/cmd_vel",
            ),

            # 기존 standalone 코드의 Nova Carter 파라미터 유지
            (
                "DiffController.inputs:maxAngularSpeed",
                1.5,
            ),
            (
                "DiffController.inputs:maxLinearSpeed",
                1.0,
            ),
            (
                "DiffController.inputs:maxWheelSpeed",
                10.0,
            ),
            (
                "DiffController.inputs:wheelDistance",
                0.413,
            ),
            (
                "DiffController.inputs:wheelRadius",
                0.145,
            ),

            # Nova Carter 구동 휠
            (
                "ArtController.inputs:jointNames",
                [
                    "joint_wheel_left",
                    "joint_wheel_right",
                ],
            ),

            (
                "ArtController.inputs:targetPrim",
                [Sdf.Path(AMR_PRIM)],
            ),
        ],

        keys.CONNECT: [

            # ROS subscriber는 매 simulation frame 실행
            (
                "OnPlaybackTick.outputs:tick",
                "SubscribeTwist.inputs:execIn",
            ),

            (
                "Context.outputs:context",
                "SubscribeTwist.inputs:context",
            ),

            # Twist.linear.x 추출
            (
                "SubscribeTwist.outputs:linearVelocity",
                "BreakLinVel.inputs:tuple",
            ),

            (
                "BreakLinVel.outputs:x",
                "DiffController.inputs:linearVelocity",
            ),

            # Twist.angular.z 추출
            (
                "SubscribeTwist.outputs:angularVelocity",
                "BreakAngVel.inputs:tuple",
            ),

            (
                "BreakAngVel.outputs:z",
                "DiffController.inputs:angularVelocity",
            ),

            # Differential controller 실행
            (
                "SubscribeTwist.outputs:execOut",
                "DiffController.inputs:execIn",
            ),

            (
                "OnPlaybackTick.outputs:deltaSeconds",
                "DiffController.inputs:dt",
            ),

            # 계산된 좌/우 wheel velocity를 articulation에 전달
            (
                "DiffController.outputs:velocityCommand",
                "ArtController.inputs:velocityCommand",
            ),

            (
                "OnPlaybackTick.outputs:tick",
                "ArtController.inputs:execIn",
            ),
        ],
    },
)


# =========================================================
# 19. Odometry / TF / Clock Action Graph
#
# Nova Carter 실제 pose
#      ↓
# Isaac Compute Odometry
#      ├── /amr/odom
#      └── odom -> base_link (/tf)
#
# 최소 Nav2 테스트에서는 AMCL을 사용하지 않기 때문에
# map -> odom을 identity static TF로 제공한다.
#
# 주의:
# 향후 AMCL을 붙일 때는 PublishMapToOdom 노드를 제거해야 한다.
# AMCL이 map -> odom TF를 담당하기 때문이다.
# =========================================================

ODOM_GRAPH_PATH = "/World/ROS2_AMR_OdomTF_Graph"

if stage.GetPrimAtPath(ODOM_GRAPH_PATH).IsValid():
    stage.RemovePrim(ODOM_GRAPH_PATH)


og.Controller.edit(
    {
        "graph_path": ODOM_GRAPH_PATH,
        "evaluator_name": "execution",
    },
    {
        keys.CREATE_NODES: [

            (
                "OnPlaybackTick",
                "omni.graph.action.OnPlaybackTick",
            ),

            (
                "ReadSimTime",
                "isaacsim.core.nodes.IsaacReadSimulationTime",
            ),

            (
                "Context",
                "isaacsim.ros2.bridge.ROS2Context",
            ),

            (
                "ComputeOdom",
                "isaacsim.core.nodes.IsaacComputeOdometry",
            ),

            (
                "PublishOdom",
                "isaacsim.ros2.bridge.ROS2PublishOdometry",
            ),

            (
                "PublishOdomToBase",
                "isaacsim.ros2.bridge.ROS2PublishRawTransformTree",
            ),

            (
                "PublishMapToOdom",
                "isaacsim.ros2.bridge.ROS2PublishRawTransformTree",
            ),

            (
                "PublishClock",
                "isaacsim.ros2.bridge.ROS2PublishClock",
            ),
        ],

        keys.SET_VALUES: [

            (
                "Context.inputs:useDomainIDEnvVar",
                True,
            ),

            # -----------------------------
            # Odometry 계산 대상
            # -----------------------------

            (
                "ComputeOdom.inputs:chassisPrim",
                [Sdf.Path(CHASSIS_PRIM)],
            ),

            # -----------------------------
            # /amr/odom
            # -----------------------------

            (
                "PublishOdom.inputs:topicName",
                "/amr/odom",
            ),

            (
                "PublishOdom.inputs:odomFrameId",
                "odom",
            ),

            (
                "PublishOdom.inputs:chassisFrameId",
                "base_link",
            ),

            # -----------------------------
            # odom -> base_link
            # -----------------------------

            (
                "PublishOdomToBase.inputs:topicName",
                "/tf",
            ),

            (
                "PublishOdomToBase.inputs:parentFrameId",
                "odom",
            ),

            (
                "PublishOdomToBase.inputs:childFrameId",
                "base_link",
            ),

            # -----------------------------
            # map -> odom
            #
            # 최소 테스트용 identity static TF
            # -----------------------------

            (
                "PublishMapToOdom.inputs:topicName",
                "/tf_static",
            ),

            (
                "PublishMapToOdom.inputs:parentFrameId",
                "map",
            ),

            (
                "PublishMapToOdom.inputs:childFrameId",
                "odom",
            ),

            (
                "PublishMapToOdom.inputs:translation",
                [0.0, 0.0, 0.0],
            ),

            (
                "PublishMapToOdom.inputs:rotation",
                [0.0, 0.0, 0.0, 1.0],
            ),

            (
                "PublishMapToOdom.inputs:staticPublisher",
                True,
            ),

            # -----------------------------
            # /clock
            # -----------------------------

            (
                "PublishClock.inputs:topicName",
                "/clock",
            ),
        ],

        keys.CONNECT: [

            # -----------------------------
            # Compute Odometry
            # -----------------------------

            (
                "OnPlaybackTick.outputs:tick",
                "ComputeOdom.inputs:execIn",
            ),

            # -----------------------------
            # /amr/odom Publish
            # -----------------------------

            (
                "ComputeOdom.outputs:execOut",
                "PublishOdom.inputs:execIn",
            ),

            (
                "ComputeOdom.outputs:angularVelocity",
                "PublishOdom.inputs:angularVelocity",
            ),

            (
                "ComputeOdom.outputs:linearVelocity",
                "PublishOdom.inputs:linearVelocity",
            ),

            (
                "ComputeOdom.outputs:orientation",
                "PublishOdom.inputs:orientation",
            ),

            (
                "ComputeOdom.outputs:position",
                "PublishOdom.inputs:position",
            ),

            (
                "ReadSimTime.outputs:simulationTime",
                "PublishOdom.inputs:timeStamp",
            ),

            (
                "Context.outputs:context",
                "PublishOdom.inputs:context",
            ),

            # -----------------------------
            # odom -> base_link TF
            # -----------------------------

            (
                "OnPlaybackTick.outputs:tick",
                "PublishOdomToBase.inputs:execIn",
            ),

            (
                "ComputeOdom.outputs:orientation",
                "PublishOdomToBase.inputs:rotation",
            ),

            (
                "ComputeOdom.outputs:position",
                "PublishOdomToBase.inputs:translation",
            ),

            (
                "ReadSimTime.outputs:simulationTime",
                "PublishOdomToBase.inputs:timeStamp",
            ),

            (
                "Context.outputs:context",
                "PublishOdomToBase.inputs:context",
            ),

            # -----------------------------
            # map -> odom static TF
            # -----------------------------

            (
                "OnPlaybackTick.outputs:tick",
                "PublishMapToOdom.inputs:execIn",
            ),

            (
                "ReadSimTime.outputs:simulationTime",
                "PublishMapToOdom.inputs:timeStamp",
            ),

            (
                "Context.outputs:context",
                "PublishMapToOdom.inputs:context",
            ),

            # -----------------------------
            # /clock
            # -----------------------------

            (
                "OnPlaybackTick.outputs:tick",
                "PublishClock.inputs:execIn",
            ),

            (
                "ReadSimTime.outputs:simulationTime",
                "PublishClock.inputs:timeStamp",
            ),

            (
                "Context.outputs:context",
                "PublishClock.inputs:context",
            ),
        ],
    },
)


print("")
print("=" * 60)
print(" ROS 2 ACTION GRAPHS READY")
print("=" * 60)
print("Expected ROS 2 interfaces:")
print("  /cmd_vel     geometry_msgs/msg/Twist   [SUB]")
print("  /amr/odom    nav_msgs/msg/Odometry     [PUB]")
print("  /tf          odom -> base_link         [PUB]")
print("  /tf_static   map  -> odom              [PUB]")
print("  /clock       rosgraph_msgs/msg/Clock   [PUB]")
print("")
print("NOTE:")
print("  /tf_static map->odom is ONLY for the minimal Nav2 test.")
print("  Remove it when AMCL/localization is added.")
print("=" * 60)
print("")


# =========================================================
# 20. Simulation Loop
#
# 기존 TCP Goal / WheelBasePoseController 로직은 제거됨.
# 이제 로봇 이동 명령은 오직 /cmd_vel에서 받는다.
# =========================================================

print_counter = 0

while simulation_app.is_running():

    world.step(
        render=True
    )

    print_counter += 1

    if print_counter >= 60:

        print_counter = 0

        current_position, _ = (
            amr.get_world_pose()
        )

        print(
            f"[AMR] "
            f"x={current_position[0]:6.2f} "
            f"y={current_position[1]:6.2f} "
            f"| waiting /cmd_vel"
        )


# =========================================================
# 21. 종료
# =========================================================

simulation_app.close()
