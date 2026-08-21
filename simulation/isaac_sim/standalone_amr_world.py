from isaacsim import SimulationApp
import socket
import time


# =========================================================
# 1. Isaac Sim 실행
# =========================================================

simulation_app = SimulationApp({
    "headless": False
})


# =========================================================
# SimulationApp 이후 Isaac Sim 모듈 import
# =========================================================

import numpy as np
import math

from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid

from isaacsim.robot.wheeled_robots.robots import WheeledRobot

from isaacsim.robot.wheeled_robots.controllers.differential_controller import (
    DifferentialController,
)

from isaacsim.robot.wheeled_robots.controllers.wheel_base_pose_controller import (
    WheelBasePoseController,
)

from isaacsim.storage.native import get_assets_root_path


# =========================================================
# 2. 환경 설정
# =========================================================

WORLD_SIZE_X = 20.0
WORLD_SIZE_Y = 12.0
WALL_HEIGHT = 2.5


# =========================================================
# 3. 주요 위치
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
# 4. World 생성
# =========================================================

world = World(
    stage_units_in_meters=1.0,
    physics_dt=1.0 / 60.0,
    rendering_dt=1.0 / 60.0,
)


# =========================================================
# 5. Box 생성 함수
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
# 6. 바닥
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
# 7. 벽
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
# 8. Supermarket
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
# 9. Cell A
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
# 10. Cell B
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
# 11. Cell C
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
# 12. Nova Carter Asset
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
# 13. Nova Carter 생성
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
# 14. World 초기화
# =========================================================

world.reset()


# =========================================================
# 15. AMR Mission 명령 TCP Server
#
# ROS2 amr_mission_node -> Isaac Sim
#
# Port 5005
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


# Simulation Loop을 막지 않도록 non-blocking
server_socket.setblocking(False)


client_sockets = []


print("")
print("=" * 60)
print(" AMR COMMAND SERVER")
print("=" * 60)
print(f"Listening : {HOST}:{PORT}")
print("")
print("Target Coordinates")
print(f"Supermarket : {LOCATIONS['supermarket'][:2]}")
print(f"Cell A      : {LOCATIONS['cell_a'][:2]}")
print(f"Cell B      : {LOCATIONS['cell_b'][:2]}")
print(f"Cell C      : {LOCATIONS['cell_c'][:2]}")
print("=" * 60)


# =========================================================
# 16. Pose Bridge TCP Client 설정
#
# Isaac Sim -> ROS2 amr_pose_bridge_node
#
# Port 5006
# =========================================================

POSE_HOST = "127.0.0.1"
POSE_PORT = 5006


pose_socket = None


# 연결 재시도 시간 관리
last_pose_connect_attempt = 0.0


# 위치 전송 시간 관리
last_pose_send_time = 0.0


# 20 Hz
POSE_SEND_INTERVAL = 0.05


# =========================================================
# 17. Pose Bridge 연결 함수
# =========================================================

def connect_pose_bridge():

    global pose_socket
    global last_pose_connect_attempt


    # 이미 연결되어 있으면 아무것도 하지 않음
    if pose_socket is not None:
        return


    current_time = time.time()


    # 연결 실패 시 매 프레임마다 재시도하지 않고
    # 1초마다 한 번씩 재시도
    if (
        current_time
        - last_pose_connect_attempt
        < 1.0
    ):
        return


    last_pose_connect_attempt = current_time


    sock = None


    try:

        sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM
        )


        # 연결 시도 때문에 Simulation이 오래 멈추지 않도록
        # 짧은 Timeout 사용
        sock.settimeout(0.2)


        sock.connect(
            (
                POSE_HOST,
                POSE_PORT
            )
        )


        # 연결 이후에는 일반 Blocking Socket 사용
        sock.settimeout(None)


        pose_socket = sock


        print("")
        print(
            f"[POSE] Connected to Pose Bridge "
            f"{POSE_HOST}:{POSE_PORT}"
        )
        print("")


    except Exception:

        if sock is not None:

            try:
                sock.close()
            except Exception:
                pass


        pose_socket = None


# =========================================================
# 18. 실제 AMR Pose 전송 함수
#
# 전송 형식:
#
# x y z qw qx qy qz
# =========================================================

def send_amr_pose(
    position,
    orientation,
):

    global pose_socket
    global last_pose_send_time


    # Bridge가 연결되지 않았으면 연결 시도
    connect_pose_bridge()


    if pose_socket is None:
        return


    current_time = time.time()


    # 20Hz보다 빠르게 보내지 않음
    if (
        current_time
        - last_pose_send_time
        < POSE_SEND_INTERVAL
    ):
        return


    last_pose_send_time = current_time


    try:

        # Isaac Sim Quaternion:
        #
        # [w, x, y, z]

        message = (
            f"{float(position[0])} "
            f"{float(position[1])} "
            f"{float(position[2])} "
            f"{float(orientation[0])} "
            f"{float(orientation[1])} "
            f"{float(orientation[2])} "
            f"{float(orientation[3])}\n"
        )


        pose_socket.sendall(
            message.encode("utf-8")
        )


    except Exception as error:

        print(
            f"[POSE] Connection lost: {error}"
        )


        try:

            pose_socket.close()

        except Exception:

            pass


        pose_socket = None


# =========================================================
# 19. 실제 DOF 이름 확인
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
# 20. Differential Controller
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
# 21. 목표 좌표 Controller
# =========================================================

pose_controller = WheelBasePoseController(

    name="nova_carter_pose_controller",

    open_loop_wheel_controller=(
        differential_controller
    ),

    is_holonomic=False,
)


# =========================================================
# 22. 시작 정보 출력
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
print("TCP")
print(f"Mission Command : {HOST}:{PORT}")
print(f"Pose Bridge     : {POSE_HOST}:{POSE_PORT}")
print("=" * 60)
print("")


# =========================================================
# 23. 상태 변수
# =========================================================

TARGET_POSITION = None

TARGET_NAME = None


goal_reached = False

print_counter = 0


# 현재 Goal을 보낸 TCP Client
#
# 목표 도착 시 REACHED 응답을 보내기 위해 보관
active_client = None


# =========================================================
# 24. Simulation Loop
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


            # -------------------------------------------------
            # 입력 형식
            #
            # -7.0 0.0
            #
            # 또는
            #
            # 7.0 3.5
            # -------------------------------------------------

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


            # 이 Goal을 보낸 Client 기억
            active_client = client_socket


            # 새로운 Goal이 들어왔으므로
            # 다시 이동 상태로 변경
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
    # =====================================================

    current_position, current_orientation = (
        amr.get_world_pose()
    )


    # =====================================================
    # 4. 실제 AMR 위치를 Pose Bridge로 전송
    #
    # Isaac Sim
    #   ↓ TCP 5006
    # amr_pose_bridge_node
    #   ↓
    # /amr/odom
    # =====================================================

    send_amr_pose(
        current_position,
        current_orientation
    )


    # =====================================================
    # 5. Goal이 없으면 현재 위치에서 대기
    # =====================================================

    if TARGET_POSITION is None:

        world.step(
            render=True
        )

        continue


    # =====================================================
    # 6. 목표까지 거리 계산
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
    # 7. 목표로 이동
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
        # 8. 도착 판정
        # =================================================

        if distance < 0.20:


            goal_reached = True


            # ---------------------------------------------
            # AMR 정지
            # ---------------------------------------------

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
    # 9. 위치 상태 출력
    # =====================================================

    print_counter += 1


    if print_counter >= 30:


        print_counter = 0


        print(
            f"[AMR] "
            f"x={current_position[0]:6.2f} "
            f"y={current_position[1]:6.2f} "
            f"| target={TARGET_NAME} "
            f"| distance={distance:5.2f}m"
        )


    # =====================================================
    # 10. Simulation Step
    # =====================================================

    world.step(
        render=True
    )


# =========================================================
# 25. 종료
# =========================================================

for client_socket in client_sockets:


    try:

        client_socket.close()

    except Exception:

        pass


server_socket.close()


# Pose Bridge TCP 연결 종료
if pose_socket is not None:

    try:

        pose_socket.close()

    except Exception:

        pass


simulation_app.close()