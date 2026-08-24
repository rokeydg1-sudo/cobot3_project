"""Scenario 0에서 사용할 데이터 구조와 재현용 고정값."""

from dataclasses import dataclass

from interfaces.msg import Location as LocationMessage


# =========================================================
# Location
#
# FMS가 소유하는 물리 위치 정보
# cuOpt는 논리 위치만 반환하고,
# FMS가 논리 위치 ID를 실제 좌표로 변환할 때 사용
# =========================================================

@dataclass(frozen=True)
class Location:
    name: str
    x: float
    y: float
    yaw: float = 0.0


# =========================================================
# Task
#
# Assembly Cell이 생성하고
# FMS가 운송 작업으로 관리하는 데이터
# =========================================================

@dataclass
class Task:
    task_id: str
    kit_id: str
    delivery_cell: str

    urgency: int
    requested_at: int
    deadline: int

    # Assembly 작업 예상 소요시간
    processing_time: float

    quantity: int = 1
    status: str = "WAITING"


    def __post_init__(self) -> None:

        # Scenario 0에서는
        # AMR capacity = 1
        # Kit 1개 단위 운송
        if self.quantity != 1:
            raise ValueError(
                "Scenario 0 requires quantity 1."
            )


        # 긴급도 범위
        if not 1 <= self.urgency <= 5:
            raise ValueError(
                "urgency must be between 1 and 5."
            )


        # 요청 시간 / Deadline 검증
        if (
            self.requested_at < 0
            or self.deadline <= self.requested_at
        ):
            raise ValueError(
                "Invalid requested_at or deadline."
            )


        # 작업시간 검증
        if self.processing_time < 0:
            raise ValueError(
                "processing_time must be non-negative."
            )


# =========================================================
# AMR State
#
# 현재 Scenario 0은 AMR 1대지만
# 추후 Multi-AMR 확장을 고려한 구조
# =========================================================

@dataclass(frozen=True)
class AMRState:
    amr_id: str
    state: str

    x: float
    y: float
    yaw: float

    load_state: str

    current_task_id: str = ""


# =========================================================
# cuOpt Optimization Request
#
# 현재 Scenario 0:
# Task 여러 개 + AMR 1대 상태
#
# 추후 Multi-AMR에서는
# amr_state -> amr_states 형태로 확장 가능
# =========================================================

@dataclass(frozen=True)
class OptimizationRequest:
    tasks: tuple[Task, ...]
    amr_state: AMRState


# =========================================================
# cuOpt Ordered Task
#
# 중요:
#
# cuOpt는 물리좌표를 반환하지 않는다.
#
# cuOpt 역할:
# "어떤 Task를 어떤 순서로 수행할 것인가"
#
# 물리 좌표 변환은 FMS에서 수행
# =========================================================

@dataclass(frozen=True)
class OrderedTask:
    sequence: int
    task_id: str
    delivery_cell: str


# =========================================================
# cuOpt Optimization Result
# =========================================================

@dataclass(frozen=True)
class OptimizationResult:
    success: bool
    message: str

    ordered_tasks: tuple[OrderedTask, ...]

    total_distance: float


# =========================================================
# FMS Location Database
#
# 현재 Isaac Sim 실제 좌표 기준
#
# 추후 Map / Location Manager로
# 분리할 수 있지만 현재는 FMS가 소유
# =========================================================

AMR_START = Location(
    name=LocationMessage.AMR_START,
    x=0.0,
    y=0.0,
)


PARTS_SUPERMARKET = Location(
    name=LocationMessage.PARTS_SUPERMARKET,
    x=-7.0,
    y=0.0,
)


ASSEMBLY_CELL_A = Location(
    name=LocationMessage.ASSEMBLY_CELL_A,
    x=7.0,
    y=3.5,
)


ASSEMBLY_CELL_B = Location(
    name=LocationMessage.ASSEMBLY_CELL_B,
    x=7.0,
    y=0.0,
)


ASSEMBLY_CELL_C = Location(
    name=LocationMessage.ASSEMBLY_CELL_C,
    x=7.0,
    y=-3.5,
)


LOCATIONS = (
    AMR_START,
    PARTS_SUPERMARKET,
    ASSEMBLY_CELL_A,
    ASSEMBLY_CELL_B,
    ASSEMBLY_CELL_C,
)


LOCATION_BY_ID = {
    "amr_start": AMR_START,
    "supermarket": PARTS_SUPERMARKET,
    "cell_a": ASSEMBLY_CELL_A,
    "cell_b": ASSEMBLY_CELL_B,
    "cell_c": ASSEMBLY_CELL_C,
}


# =========================================================
# Scenario 0 재현용 고정 Task
#
# 중요:
#
# 아래 TASKS는
# FMS / cuOpt 단위 테스트용 데이터다.
#
# 실제 E2E 통합에서는
# Assembly Cell이 생성한 Task를
# FMS가 받아서 Queue에 넣는 구조로 사용
# =========================================================

TASKS = (

    Task(
        task_id="task_01",
        kit_id="KIT_STAR",
        delivery_cell="cell_a",
        urgency=1,
        requested_at=0,
        deadline=580,
        processing_time=3.0,
    ),

    Task(
        task_id="task_02",
        kit_id="KIT_SQUARE",
        delivery_cell="cell_b",
        urgency=4,
        requested_at=5,
        deadline=380,
        processing_time=5.0,
    ),

    Task(
        task_id="task_03",
        kit_id="KIT_TRIANGLE",
        delivery_cell="cell_c",
        urgency=1,
        requested_at=10,
        deadline=600,
        processing_time=8.0,
    ),

    Task(
        task_id="task_04",
        kit_id="KIT_CIRCLE",
        delivery_cell="cell_a",
        urgency=3,
        requested_at=15,
        deadline=450,
        processing_time=10.0,
    ),

    Task(
        task_id="task_05",
        kit_id="KIT_STAR",
        delivery_cell="cell_c",
        urgency=5,
        requested_at=20,
        deadline=300,
        processing_time=3.0,
    ),

    Task(
        task_id="task_06",
        kit_id="KIT_SQUARE",
        delivery_cell="cell_b",
        urgency=1,
        requested_at=25,
        deadline=620,
        processing_time=5.0,
    ),

    Task(
        task_id="task_07",
        kit_id="KIT_TRIANGLE",
        delivery_cell="cell_a",
        urgency=4,
        requested_at=30,
        deadline=400,
        processing_time=8.0,
    ),

    Task(
        task_id="task_08",
        kit_id="KIT_CIRCLE",
        delivery_cell="cell_c",
        urgency=2,
        requested_at=35,
        deadline=520,
        processing_time=10.0,
    ),

    Task(
        task_id="task_09",
        kit_id="KIT_STAR",
        delivery_cell="cell_b",
        urgency=5,
        requested_at=40,
        deadline=320,
        processing_time=3.0,
    ),

    Task(
        task_id="task_10",
        kit_id="KIT_SQUARE",
        delivery_cell="cell_a",
        urgency=2,
        requested_at=45,
        deadline=540,
        processing_time=5.0,
    ),
)


# =========================================================
# Scenario 0 초기 AMR 상태
#
# 테스트용 기본값
#
# 실제 Pull 방식에서는
# RequestTask Service를 통해 AMR이
# 현재 위치 / 상태를 FMS에 전달
# =========================================================

INITIAL_AMR_STATE = AMRState(
    amr_id="AMR_01",
    state="IDLE",

    x=AMR_START.x,
    y=AMR_START.y,
    yaw=AMR_START.yaw,

    load_state="EMPTY",

    current_task_id="",
)
