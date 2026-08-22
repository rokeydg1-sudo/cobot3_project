"""Scenario 0에서 사용할 데이터 구조와 재현용 고정값."""

from dataclasses import dataclass


@dataclass(frozen=True)
class Location:
    location_id: str
    name: str
    x: float
    y: float
    yaw: float = 0.0


@dataclass
class Task:
    """Assembly Cell이 요청하고 FMS가 큐에서 관리하는 부품 운송 작업."""

    task_id: str
    kit_id: str
    delivery_cell: str
    urgency: int
    requested_at: int
    deadline: int
    service_time: int
    quantity: int = 1
    status: str = "WAITING"

    def __post_init__(self) -> None:
        if self.quantity != 1:
            raise ValueError("Scenario 0 requires quantity 1.")
        if not 1 <= self.urgency <= 5:
            raise ValueError("urgency must be between 1 and 5.")
        if self.requested_at < 0 or self.deadline <= self.requested_at:
            raise ValueError("Invalid requested_at or deadline.")
        if self.service_time < 0:
            raise ValueError("service_time must be non-negative.")


@dataclass(frozen=True)
class AMRState:
    amr_id: str
    state: str
    x: float
    y: float
    yaw: float
    load_state: str
    current_task_id: str = ""


@dataclass(frozen=True)
class OptimizationRequest:
    tasks: tuple[Task, ...]
    amr_state: AMRState


@dataclass(frozen=True)
class OrderedTask:
    sequence: int
    task_id: str
    delivery_cell: str
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class OptimizationResult:
    success: bool
    message: str
    ordered_tasks: tuple[OrderedTask, ...]
    total_distance: float


# Isaac Sim stage 좌표가 확정되면 아래 고정값만 수정한다.
AMR_START = Location("amr_start", "AMR Start", 0.0, 0.0)
PARTS_SUPERMARKET = Location("supermarket", "Parts Supermarket", -6.0, 0.0)
ASSEMBLY_CELL_A = Location("cell_a", "Assembly Cell A", 0.0, 4.0)
ASSEMBLY_CELL_B = Location("cell_b", "Assembly Cell B", 5.0, 4.0)
ASSEMBLY_CELL_C = Location("cell_c", "Assembly Cell C", 10.0, 4.0)

LOCATIONS = (
    AMR_START,
    PARTS_SUPERMARKET,
    ASSEMBLY_CELL_A,
    ASSEMBLY_CELL_B,
    ASSEMBLY_CELL_C,
)
LOCATION_BY_ID = {location.location_id: location for location in LOCATIONS}


# Scenario 0 고정 Task 10개. 리스트와 주석 순서가 최초 입력 순서다.
TASKS = (
    # 입력 순서 01: Cell A 일반 요청
    Task("task_01", "kit_a", "cell_a", 1, 0, 580, 15),
    # 입력 순서 02: Cell B 높은 긴급도 요청
    Task("task_02", "kit_b", "cell_b", 4, 5, 380, 15),
    # 입력 순서 03: Cell C 일반 요청
    Task("task_03", "kit_c", "cell_c", 1, 10, 600, 15),
    # 입력 순서 04: Cell A 중간 긴급도 요청
    Task("task_04", "kit_a", "cell_a", 3, 15, 450, 15),
    # 입력 순서 05: Cell C 최우선 긴급 요청
    Task("task_05", "kit_c", "cell_c", 5, 20, 300, 15),
    # 입력 순서 06: Cell B 일반 요청
    Task("task_06", "kit_b", "cell_b", 1, 25, 620, 15),
    # 입력 순서 07: Cell A 높은 긴급도 요청
    Task("task_07", "kit_a", "cell_a", 4, 30, 400, 15),
    # 입력 순서 08: Cell C 낮은 긴급도 요청
    Task("task_08", "kit_c", "cell_c", 2, 35, 520, 15),
    # 입력 순서 09: Cell B 최우선 긴급 요청
    Task("task_09", "kit_b", "cell_b", 5, 40, 320, 15),
    # 입력 순서 10: Cell A 낮은 긴급도 요청
    Task("task_10", "kit_a", "cell_a", 2, 45, 540, 15),
)

INITIAL_AMR_STATE = AMRState(
    amr_id="amr_01",
    state="IDLE",
    x=AMR_START.x,
    y=AMR_START.y,
    yaw=AMR_START.yaw,
    load_state="EMPTY",
)
