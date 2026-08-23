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
# SimulationApp 이후 import
# =========================================================

import math
import numpy as np
import omni.graph.core as og
import omni.kit.app
import omni.usd

from pxr import Sdf

from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid

from isaacsim.core.utils.extensions import enable_extension

from isaacsim.robot.wheeled_robots.robots import WheeledRobot

from isaacsim.robot.wheeled_robots.controllers.differential_controller import (
    DifferentialController,
)

from isaacsim.robot.wheeled_robots.controllers.wheel_base_pose_controller import (
    WheelBasePoseController,
)

from isaacsim.storage.native import get_assets_root_path


# =========================================================
# 2. ROS2 Bridge Extension 활성화
# =========================================================

ROS2_EXTENSIONS = (
    "isaacsim.ros2.core",
    "isaacsim.ros2.nodes",
    "isaacsim.ros2.bridge",
)

for extension_id in ROS2_EXTENSIONS:
    enable_extension(extension_id)

# Extension과 OmniGraph node type 등록이 완료될 때까지 Kit를 갱신한다.
extension_manager = omni.kit.app.get_app().get_extension_manager()
for _ in range(30):
    simulation_app.update()
    if all(
        extension_manager.is_extension_enabled(extension_id)
        for extension_id in ROS2_EXTENSIONS
    ):
        break

disabled_extensions = [
    extension_id
    for extension_id in ROS2_EXTENSIONS
    if not extension_manager.is_extension_enabled(extension_id)
]
if disabled_extensions:
    simulation_app.close()
    raise RuntimeError(
        "Failed to load required ROS 2 extensions: "
        + ", ".join(disabled_extensions)
    )


print("")
print("=" * 60)
print(" ROS2 BRIDGE ENABLED")
print("=" * 60)
print("Extensions: " + ", ".join(ROS2_EXTENSIONS))
print("=" * 60)
print("")


# =========================================================
# 3. 환경 설정
# =========================================================

WORLD_SIZE_X = 20.0
WORLD_SIZE_Y = 12.0
WALL_HEIGHT = 2.5


# =========================================================
# 4. 주요 위치
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

amr = world.scene.add(

    WheeledRobot(
        prim_path="/World/AMR",

        name="nova_carter",

        wheel_dof_names=[
            "joint_wheel_left",
            "joint_wheel_right",
        ],

        create_robot=True,

        usd_path=nova_carter_usd,

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
# 16. ROS2 OmniGraph
#
# 명령:
#
# /amr/goal
# geometry_msgs/msg/Point
#       ↓
# ROS2 Subscriber
#       ↓
# Nova Carter
#
#
# 위치:
#
# Nova Carter
#       ↓
# IsaacComputeOdometry
#       ↓
# ROS2PublishOdometry
#       ↓
# /amr/odom
#
#
# TCP 5005 / 5006 모두 제거
# =========================================================

ROS2_GRAPH_PATH = (
    "/World/ROS2_AMR_Graph"
)

AMR_CHASSIS_PRIM = (
    "/World/AMR/chassis_link"
)


stage = (
    omni.usd
    .get_context()
    .get_stage()
)


existing_graph = (
    stage.GetPrimAtPath(
        ROS2_GRAPH_PATH
    )
)


if (
    existing_graph
    and existing_graph.IsValid()
):

    stage.RemovePrim(
        ROS2_GRAPH_PATH
    )


keys = og.Controller.Keys


# =========================================================
# Graph 생성
# =========================================================

og.Controller.edit(

    {
        "graph_path": ROS2_GRAPH_PATH,
        "evaluator_name": "execution",
    },

    {

        # =================================================
        # Node 생성
        # =================================================

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
                "SubscribeGoal",
                "isaacsim.ros2.bridge.ROS2Subscriber",
            ),
        ],


        # =================================================
        # 값 설정
        # =================================================

        keys.SET_VALUES: [

            # ---------------------------------------------
            # Odometry 대상
            # ---------------------------------------------

            (
                "ComputeOdom.inputs:chassisPrim",

                [
                    Sdf.Path(
                        AMR_CHASSIS_PRIM
                    )
                ],
            ),


            # ---------------------------------------------
            # /amr/odom
            # ---------------------------------------------

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


            # ---------------------------------------------
            # /amr/goal
            #
            # geometry_msgs/msg/Point
            # ---------------------------------------------

            (
                "SubscribeGoal.inputs:topicName",
                "/amr/goal",
            ),

            (
                "SubscribeGoal.inputs:messagePackage",
                "geometry_msgs",
            ),

            (
                "SubscribeGoal.inputs:messageSubfolder",
                "msg",
            ),

            (
                "SubscribeGoal.inputs:messageName",
                "Point",
            ),


            # ---------------------------------------------
            # ROS_DOMAIN_ID 사용
            # ---------------------------------------------

            (
                "Context.inputs:useDomainIDEnvVar",
                True,
            ),
        ],


        # =================================================
        # 연결
        # =================================================

        keys.CONNECT: [

            # ---------------------------------------------
            # Tick -> Odometry
            # ---------------------------------------------

            (
                "OnPlaybackTick.outputs:tick",
                "ComputeOdom.inputs:execIn",
            ),

            (
                "ComputeOdom.outputs:execOut",
                "PublishOdom.inputs:execIn",
            ),

            (
                "ComputeOdom.outputs:position",
                "PublishOdom.inputs:position",
            ),

            (
                "ComputeOdom.outputs:orientation",
                "PublishOdom.inputs:orientation",
            ),

            (
                "ComputeOdom.outputs:linearVelocity",
                "PublishOdom.inputs:linearVelocity",
            ),

            (
                "ComputeOdom.outputs:angularVelocity",
                "PublishOdom.inputs:angularVelocity",
            ),

            (
                "ReadSimTime.outputs:simulationTime",
                "PublishOdom.inputs:timeStamp",
            ),

            (
                "Context.outputs:context",
                "PublishOdom.inputs:context",
            ),


            # ---------------------------------------------
            # Tick -> Goal Subscriber
            # ---------------------------------------------

            (
                "OnPlaybackTick.outputs:tick",
                "SubscribeGoal.inputs:execIn",
            ),

            (
                "Context.outputs:context",
                "SubscribeGoal.inputs:context",
            ),
        ],
    },
)


# =========================================================
# Generic Subscriber는 message type 지정 후
# x/y/z 출력 Attribute를 동적으로 생성함
# =========================================================

simulation_app.update()
simulation_app.update()


GOAL_NODE_PATH = (
    ROS2_GRAPH_PATH
    + "/SubscribeGoal"
)


goal_x_attr = (
    og.Controller.attribute(

        GOAL_NODE_PATH
        + ".outputs:x"
    )
)


goal_y_attr = (
    og.Controller.attribute(

        GOAL_NODE_PATH
        + ".outputs:y"
    )
)


goal_z_attr = (
    og.Controller.attribute(

        GOAL_NODE_PATH
        + ".outputs:z"
    )
)


if (

    goal_x_attr is None

    or not goal_x_attr.is_valid()

    or goal_y_attr is None

    or not goal_y_attr.is_valid()

    or goal_z_attr is None

    or not goal_z_attr.is_valid()
):

    raise RuntimeError(

        "ROS2 Subscriber Point 출력 "
        "x/y/z를 찾지 못했습니다."
    )


print("")
print("=" * 60)
print(" ROS2 AMR GRAPH CREATED")
print("=" * 60)

print(
    f"Graph       : "
    f"{ROS2_GRAPH_PATH}"
)

print(
    f"Chassis     : "
    f"{AMR_CHASSIS_PRIM}"
)

print(
    "Goal Topic  : /amr/goal"
)

print(
    "Goal Type   : geometry_msgs/msg/Point"
)

print(
    "Odom Topic  : /amr/odom"
)

print(
    "TCP 5005    : REMOVED"
)

print(
    "TCP 5006    : REMOVED"
)

print("=" * 60)
print("")


# =========================================================
# 17. DOF 확인
# =========================================================

print("")
print("=" * 60)
print("Nova Carter DOF")
print("=" * 60)


for index, dof_name in enumerate(
    amr.dof_names
):

    print(
        f"{index:2d} : {dof_name}"
    )


print("=" * 60)
print("")


# =========================================================
# 18. Differential Controller
# =========================================================

differential_controller = (
    DifferentialController(

        name=(
            "nova_carter_diff_controller"
        ),

        wheel_radius=0.145,

        wheel_base=0.413,

        max_linear_speed=1.0,

        max_angular_speed=1.5,

        max_wheel_speed=10.0,
    )
)


# =========================================================
# 19. Pose Controller
# =========================================================

pose_controller = (
    WheelBasePoseController(

        name=(
            "nova_carter_pose_controller"
        ),

        open_loop_wheel_controller=(
            differential_controller
        ),

        is_holonomic=False,
    )
)


# =========================================================
# 20. 시작 정보
# =========================================================

print("")
print("=" * 60)
print(" AMR STANDALONE WORLD")
print("=" * 60)

print(
    "AMR STATUS   : IDLE"
)

print(
    "TARGET       : Waiting for /amr/goal"
)

print("")

print(
    "Communication"
)

print(
    "Mission Command : "
    "ROS2 Bridge <- /amr/goal"
)

print(
    "Pose Feedback   : "
    "ROS2 Bridge -> /amr/odom"
)

print("=" * 60)
print("")


# =========================================================
# 21. 상태 변수
# =========================================================

TARGET_POSITION = None

TARGET_NAME = None

goal_reached = False

print_counter = 0


# /amr/goal Point.z의 command_id
last_goal_command_id = 0


# =========================================================
# 22. Simulation Loop
# =========================================================

while simulation_app.is_running():


    # =====================================================
    # 1. /amr/goal 읽기
    # =====================================================

    try:

        goal_x = float(

            og.Controller.get(
                goal_x_attr
            )
        )


        goal_y = float(

            og.Controller.get(
                goal_y_attr
            )
        )


        goal_command_id = int(

            round(

                float(

                    og.Controller.get(
                        goal_z_attr
                    )
                )
            )
        )


    except Exception as error:

        print(

            f"[ROS2 GOAL ERROR] "
            f"{error}"
        )


        goal_command_id = (
            last_goal_command_id
        )


    # =====================================================
    # 2. 새로운 Goal인지 확인
    # =====================================================

    if (

        goal_command_id > 0

        and goal_command_id
        != last_goal_command_id
    ):

        last_goal_command_id = (
            goal_command_id
        )


        TARGET_POSITION = np.array(

            [
                goal_x,
                goal_y,
                0.0,
            ],

            dtype=np.float32,
        )


        TARGET_NAME = (

            f"({goal_x:.2f}, "
            f"{goal_y:.2f})"
        )


        goal_reached = False


        pose_controller.reset()


        print("")
        print("=" * 60)
        print("NEW ROS2 TARGET RECEIVED")
        print("=" * 60)

        print(
            f"Command ID : "
            f"{goal_command_id}"
        )

        print(
            f"X          : "
            f"{goal_x:.2f}"
        )

        print(
            f"Y          : "
            f"{goal_y:.2f}"
        )

        print("=" * 60)
        print("")


    # =====================================================
    # 3. 현재 위치
    # =====================================================

    (
        current_position,
        current_orientation,
    ) = amr.get_world_pose()


    # =====================================================
    # 4. Goal 존재
    # =====================================================

    if TARGET_POSITION is not None:


        dx = (

            TARGET_POSITION[0]
            - current_position[0]
        )


        dy = (

            TARGET_POSITION[1]
            - current_position[1]
        )


        distance = math.sqrt(

            dx * dx
            + dy * dy
        )


        # =================================================
        # 이동
        # =================================================

        if not goal_reached:


            action = (
                pose_controller.forward(

                    start_position=(
                        current_position
                    ),

                    start_orientation=(
                        current_orientation
                    ),

                    goal_position=(
                        TARGET_POSITION
                    ),

                    lateral_velocity=0.6,

                    yaw_velocity=0.8,

                    heading_tol=0.05,

                    position_tol=0.20,
                )
            )


            amr.apply_wheel_actions(
                action
            )


            # =============================================
            # Isaac 내부에서 로봇 정지
            #
            # Mission 성공 판정은
            # AMR Mission Node가 /amr/odom으로 처리
            # =============================================

            if distance < 0.20:


                goal_reached = True


                amr.apply_wheel_actions(

                    differential_controller.forward(

                        command=np.array(
                            [0.0, 0.0]
                        )
                    )
                )


                print("")
                print("=" * 60)
                print("TARGET REACHED")
                print("=" * 60)

                print(
                    f"x="
                    f"{current_position[0]:.2f}"
                )

                print(
                    f"y="
                    f"{current_position[1]:.2f}"
                )

                print("=" * 60)
                print("")


        # =================================================
        # 위치 출력
        # =================================================

        print_counter += 1


        if print_counter >= 30:


            print_counter = 0


            print(

                f"[AMR] "

                f"x="
                f"{current_position[0]:6.2f} "

                f"y="
                f"{current_position[1]:6.2f} "

                f"| target="
                f"{TARGET_NAME} "

                f"| distance="
                f"{distance:5.2f}m"
            )


    # =====================================================
    # 5. Simulation Step
    #
    # 여기서
    # - /amr/goal 구독
    # - /amr/odom 발행
    # =====================================================

    world.step(
        render=True
    )


# =========================================================
# 23. 종료
# =========================================================

simulation_app.close()
