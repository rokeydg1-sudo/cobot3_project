from isaacsim import SimulationApp
import socket


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

enable_extension(
    "isaacsim.ros2.bridge"
)

# Extension 로딩 반영
simulation_app.update()


print("")
print("=" * 60)
print(" ROS2 BRIDGE ENABLED")
print("=" * 60)
print("Extension : isaacsim.ros2.bridge")
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
# 16. ROS2 Odometry OmniGraph 생성
#
# Isaac Sim
#   ↓
# IsaacComputeOdometry
#   ↓
# ROS2PublishOdometry
#   ↓
# /amr/odom
#
# TCP 5006 / amr_pose_bridge_node 필요 없음
# =========================================================

ROS2_GRAPH_PATH = "/World/ROS2_AMR_Odom_Graph"

# 우선 AMR articulation root 자체를 chassisPrim으로 사용
AMR_CHASSIS_PRIM = "/World/AMR/chassis_link"


stage = omni.usd.get_context().get_stage()


# 기존 Graph가 있으면 제거
if stage.GetPrimAtPath(
    ROS2_GRAPH_PATH
):

    stage.RemovePrim(
        ROS2_GRAPH_PATH
    )


keys = og.Controller.Keys


og.Controller.edit(

    {
        "graph_path": ROS2_GRAPH_PATH,
        "evaluator_name": "execution",
    },

    {

        # =================================================
        # OmniGraph Node 생성
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
        ],


        # =================================================
        # Node 설정값
        # =================================================

        keys.SET_VALUES: [

            # ---------------------------------------------
            # 실제 Nova Carter 위치 계산 대상
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
            # ROS2 Topic
            # ---------------------------------------------

            (
                "PublishOdom.inputs:topicName",
                "/amr/odom",
            ),


            # ---------------------------------------------
            # Frame
            # ---------------------------------------------

            (
                "PublishOdom.inputs:odomFrameId",
                "odom",
            ),

            (
                "PublishOdom.inputs:chassisFrameId",
                "base_link",
            ),


            # ---------------------------------------------
            # ROS_DOMAIN_ID
            #
            # True이면 실행 터미널의
            # ROS_DOMAIN_ID 환경변수를 그대로 사용
            # ---------------------------------------------

            (
                "Context.inputs:useDomainIDEnvVar",
                True,
            ),
        ],


        # =================================================
        # Node 연결
        # =================================================

        keys.CONNECT: [

            # ---------------------------------------------
            # Simulation Tick
            # ↓
            # Odometry 계산
            # ---------------------------------------------

            (
                "OnPlaybackTick.outputs:tick",
                "ComputeOdom.inputs:execIn",
            ),


            # ---------------------------------------------
            # 계산 완료
            # ↓
            # ROS2 Odometry Publish
            # ---------------------------------------------

            (
                "ComputeOdom.outputs:execOut",
                "PublishOdom.inputs:execIn",
            ),


            # ---------------------------------------------
            # Position
            # ---------------------------------------------

            (
                "ComputeOdom.outputs:position",
                "PublishOdom.inputs:position",
            ),


            # ---------------------------------------------
            # Orientation
            # ---------------------------------------------

            (
                "ComputeOdom.outputs:orientation",
                "PublishOdom.inputs:orientation",
            ),


            # ---------------------------------------------
            # Linear Velocity
            # ---------------------------------------------

            (
                "ComputeOdom.outputs:linearVelocity",
                "PublishOdom.inputs:linearVelocity",
            ),


            # ---------------------------------------------
            # Angular Velocity
            # ---------------------------------------------

            (
                "ComputeOdom.outputs:angularVelocity",
                "PublishOdom.inputs:angularVelocity",
            ),


            # ---------------------------------------------
            # Simulation Time
            # ---------------------------------------------

            (
                "ReadSimTime.outputs:simulationTime",
                "PublishOdom.inputs:timeStamp",
            ),


            # ---------------------------------------------
            # ROS2 Context
            # ---------------------------------------------

            (
                "Context.outputs:context",
                "PublishOdom.inputs:context",
            ),
        ],
    },
)


print("")
print("=" * 60)
print(" ROS2 ODOM GRAPH CREATED")
print("=" * 60)
print(f"Graph       : {ROS2_GRAPH_PATH}")
print(f"Chassis     : {AMR_CHASSIS_PRIM}")
print("ROS2 Topic  : /amr/odom")
print("TCP 5006    : REMOVED")
print("Pose Bridge : NOT REQUIRED")
print("=" * 60)
print("")


# =========================================================
# 17. AMR Mission 명령 TCP Server
#
# 이 부분은 아직 그대로 유지
#
# ROS2 amr_mission_node
#       ↓
# TCP 5005
#       ↓
# Isaac Sim
# =========================================================

HOST = "127.0.0.1"
PORT = 5005


server_socket = socket.socket(
    socket.AF_INET,
    socket.SOCK_STREAM,
)


server_socket.setsockopt(
    socket.SOL_SOCKET,
    socket.SO_REUSEADDR,
    1,
)


server_socket.bind(
    (HOST, PORT)
)


server_socket.listen(5)


# Simulation Loop 방해하지 않도록 non-blocking
server_socket.setblocking(False)


client_sockets = []


print("")
print("=" * 60)
print(" AMR COMMAND SERVER")
print("=" * 60)
print(f"Listening : {HOST}:{PORT}")
print("")
print("Target Coordinates")
print(
    f"Supermarket : "
    f"{LOCATIONS['supermarket'][:2]}"
)
print(
    f"Cell A      : "
    f"{LOCATIONS['cell_a'][:2]}"
)
print(
    f"Cell B      : "
    f"{LOCATIONS['cell_b'][:2]}"
)
print(
    f"Cell C      : "
    f"{LOCATIONS['cell_c'][:2]}"
)
print("=" * 60)


# =========================================================
# 18. 실제 DOF 이름 확인
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
# 19. Differential Controller
# =========================================================

differential_controller = DifferentialController(

    name="nova_carter_diff_controller",

    wheel_radius=0.145,

    wheel_base=0.413,

    max_linear_speed=1.0,

    max_angular_speed=1.5,

    max_wheel_speed=10.0,
)


# =========================================================
# 20. 목표 좌표 Controller
# =========================================================

pose_controller = WheelBasePoseController(

    name="nova_carter_pose_controller",

    open_loop_wheel_controller=(
        differential_controller
    ),

    is_holonomic=False,
)


# =========================================================
# 21. 시작 정보 출력
# =========================================================

print("")
print("=" * 60)
print(" AMR STANDALONE WORLD")
print("=" * 60)

print("AMR STATUS   : IDLE")
print("TARGET       : Waiting for external command")

print("")

print("Available coordinates:")

print(
    f"Supermarket  : "
    f"{LOCATIONS['supermarket'][0]:.1f} "
    f"{LOCATIONS['supermarket'][1]:.1f}"
)

print(
    f"Cell A       : "
    f"{LOCATIONS['cell_a'][0]:.1f} "
    f"{LOCATIONS['cell_a'][1]:.1f}"
)

print(
    f"Cell B       : "
    f"{LOCATIONS['cell_b'][0]:.1f} "
    f"{LOCATIONS['cell_b'][1]:.1f}"
)

print(
    f"Cell C       : "
    f"{LOCATIONS['cell_c'][0]:.1f} "
    f"{LOCATIONS['cell_c'][1]:.1f}"
)

print("=" * 60)

print("")
print("Communication")
print(
    f"Mission Command : TCP "
    f"{HOST}:{PORT}"
)
print(
    "Pose Feedback   : "
    "ROS2 Bridge -> /amr/odom"
)
print("=" * 60)
print("")


# =========================================================
# 22. 상태 변수
# =========================================================

TARGET_POSITION = None

TARGET_NAME = None


goal_reached = False

print_counter = 0


# 현재 Goal을 보낸 TCP Client
active_client = None


# =========================================================
# 23. Simulation Loop
# =========================================================

while simulation_app.is_running():


    # =====================================================
    # 1. 새로운 Mission TCP Client 연결 확인
    # =====================================================

    try:

        client_socket, client_address = (
            server_socket.accept()
        )


        client_socket.setblocking(False)


        client_sockets.append(
            client_socket
        )


        print(
            f"[TCP] Client connected: "
            f"{client_address}"
        )


    except BlockingIOError:

        pass


    # =====================================================
    # 2. Client가 보낸 Goal 좌표 확인
    # =====================================================

    for client_socket in client_sockets[:]:


        try:

            data = client_socket.recv(
                1024
            )


            if not data:

                client_socket.close()


                if client_socket in client_sockets:

                    client_sockets.remove(
                        client_socket
                    )


                continue


            command = (
                data
                .decode("utf-8")
                .strip()
            )


            print(
                f"[TCP] Received: {command}"
            )


            # ---------------------------------------------
            # 입력:
            #
            # -7.0 0.0
            #  7.0 3.5
            # ---------------------------------------------

            values = command.split()


            if len(values) != 2:

                client_socket.sendall(
                    b"ERROR invalid command\n"
                )

                continue


            goal_x = float(
                values[0]
            )

            goal_y = float(
                values[1]
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


            active_client = client_socket


            goal_reached = False


            pose_controller.reset()


            print("")
            print("=" * 60)
            print("NEW TARGET RECEIVED")
            print("=" * 60)
            print(f"X : {goal_x:.2f}")
            print(f"Y : {goal_y:.2f}")
            print("=" * 60)
            print("")


        except BlockingIOError:

            pass


        except ValueError:

            print(
                "[TCP] Invalid coordinates"
            )


            try:

                client_socket.sendall(
                    b"ERROR invalid coordinates\n"
                )

            except Exception:

                pass


        except Exception as error:

            print(
                f"[TCP ERROR] {error}"
            )


            try:

                client_socket.close()

            except Exception:

                pass


            if client_socket in client_sockets:

                client_sockets.remove(
                    client_socket
                )


            if active_client is client_socket:

                active_client = None


    # =====================================================
    # 3. 현재 Nova Carter 실제 위치 / 방향
    #
    # 이 값은 AMR 이동 제어용으로 계속 사용
    #
    # ROS2 Publish는 OmniGraph가 직접 처리
    # =====================================================

    current_position, current_orientation = (
        amr.get_world_pose()
    )


    # =====================================================
    # 4. Goal 없으면 대기
    # =====================================================

    if TARGET_POSITION is None:

        world.step(
            render=True
        )

        continue


    # =====================================================
    # 5. 목표까지 거리 계산
    # =====================================================

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


    # =====================================================
    # 6. 목표로 이동
    # =====================================================

    if not goal_reached:


        action = pose_controller.forward(

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


        amr.apply_wheel_actions(
            action
        )


        # =================================================
        # 7. 도착 판정
        # =================================================

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
                f"x={current_position[0]:.2f}"
            )

            print(
                f"y={current_position[1]:.2f}"
            )

            print("=" * 60)
            print("")


            # ---------------------------------------------
            # Mission Node에 REACHED 응답
            # ---------------------------------------------

            if active_client is not None:


                try:

                    active_client.sendall(
                        b"REACHED\n"
                    )


                    print(
                        "[TCP] Sent: REACHED"
                    )


                    active_client.close()


                    if (
                        active_client
                        in client_sockets
                    ):

                        client_sockets.remove(
                            active_client
                        )


                except Exception as error:

                    print(
                        f"[TCP] "
                        f"REACHED send error: "
                        f"{error}"
                    )


                active_client = None


    # =====================================================
    # 8. 위치 출력
    # =====================================================

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
    # 9. Simulation Step
    #
    # OnPlaybackTick가 여기서 실행되면서
    # /amr/odom이 자동 Publish됨
    # =====================================================

    world.step(
        render=True
    )


# =========================================================
# 24. 종료
# =========================================================

for client_socket in client_sockets:

    try:

        client_socket.close()

    except Exception:

        pass


server_socket.close()


simulation_app.close()