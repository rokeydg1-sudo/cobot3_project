import queue
import random

from .task import Task


class AssemblyCell:

    # ====================================================
    # Cell 상태
    # ====================================================

    WAITING_FOR_PART = "WAITING_FOR_PART"
    PROCESSING = "PROCESSING"


    def __init__(self, cell_id, queue_size=3):

        self.cell_id = cell_id
        self.queue_size = queue_size

        # ====================================================
        # Cell별 FIFO Task Queue
        # ====================================================

        self.task_queue = queue.Queue(maxsize=queue_size)


        # ====================================================
        # 현재 Cell 상태
        # ====================================================

        self.state = self.WAITING_FOR_PART

        self.current_task = None
        self.start_time = None


        # ====================================================
        # 초기 Task 3개 생성
        # ====================================================

        self.fill_queue()


    # ====================================================
    # 랜덤 Task 생성
    # ====================================================

    def create_random_task(self):

        shape = random.choice([
            "STAR",
            "SQUARE",
            "TRIANGLE",
            "CIRCLE",
        ])

        return Task(shape)


    # ====================================================
    # Queue를 항상 queue_size만큼 유지
    # ====================================================

    def fill_queue(self):

        while not self.task_queue.full():

            task = self.create_random_task()

            self.task_queue.put_nowait(task)

            print(
                f"Cell {self.cell_id} - "
                f"새 Task 생성: "
                f"Task {task.task_id} "
                f"({task.shape}, {task.processing_time:.0f}초)"
            )


    # ====================================================
    # 현재 맨 앞 Task 확인
    # ====================================================

    def get_next_task(self):

        if self.task_queue.empty():
            return None

        # Queue에서 제거하지 않고 맨 앞 Task만 확인
        return self.task_queue.queue[0]


    # ====================================================
    # AMR 부품 도착
    # ====================================================

    def part_arrived(self, simulation_time):

        # 이미 작업 중이면 무시
        if self.state == self.PROCESSING:

            print(
                f"[Time: {simulation_time:.2f}] "
                f"Cell {self.cell_id} - "
                f"이미 작업 중입니다."
            )

            return False


        # Queue가 비어 있으면 작업 불가
        if self.task_queue.empty():

            print(
                f"[Time: {simulation_time:.2f}] "
                f"Cell {self.cell_id} - "
                f"Task가 없습니다."
            )

            return False


        # ====================================================
        # 맨 앞 Task를 실제 작업 대상으로 꺼냄
        # ====================================================

        self.current_task = self.task_queue.get_nowait()

        self.state = self.PROCESSING

        self.start_time = simulation_time


        print(
            f"[Time: {simulation_time:.2f}] "
            f"Cell {self.cell_id} - "
            f"AMR 부품 도착"
        )

        print(
            f"[Time: {simulation_time:.2f}] "
            f"Cell {self.cell_id} - "
            f"Task {self.current_task.task_id} "
            f"({self.current_task.shape}) 작업 시작 "
            f"[{self.current_task.processing_time:.0f}초]"
        )

        return True


    # ====================================================
    # 작업 완료
    # ====================================================

    def complete_task(self, simulation_time):

        if self.current_task is None:
            return None


        print(
            f"[Time: {simulation_time:.2f}] "
            f"Cell {self.cell_id} - "
            f"Task {self.current_task.task_id} "
            f"({self.current_task.shape}) 작업 완료"
        )


        completed_task = self.current_task


        # Queue 작업 완료 처리
        self.task_queue.task_done()


        # 현재 작업 초기화
        self.current_task = None

        self.start_time = None

        self.state = self.WAITING_FOR_PART


        # ====================================================
        # Task 하나가 사라졌으므로
        # 랜덤 Task를 추가해서 다시 3개 유지
        # ====================================================

        self.fill_queue()


        return completed_task


    # ====================================================
    # Simulation Step마다 호출
    # ====================================================

    def update(self, simulation_time):

        # WAITING_FOR_PART 상태에서는 아무것도 하지 않음
        # AMR 도착 신호를 기다림

        if self.state == self.WAITING_FOR_PART:
            return None


        # ====================================================
        # PROCESSING 상태
        # ====================================================

        if self.state == self.PROCESSING:

            elapsed_time = (
                simulation_time - self.start_time
            )

            if (
                elapsed_time
                >= self.current_task.processing_time
            ):

                return self.complete_task(
                    simulation_time
                )


        return None


    # ====================================================
    # 현재 Queue 확인용
    # ====================================================

    def print_queue(self):

        tasks = list(self.task_queue.queue)

        queue_text = " -> ".join(
            f"{task.shape}({task.processing_time:.0f}s)"
            for task in tasks
        )

        print(
            f"Cell {self.cell_id} Queue: "
            f"[{queue_text}]"
        )