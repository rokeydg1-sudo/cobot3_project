import queue


# ============================================================
# Task
# ============================================================

class Task:

    def __init__(self, task_id, kit_id, processing_time):

        self.task_id = task_id
        self.kit_id = kit_id
        self.processing_time = processing_time


# ============================================================
# Assembly Cell
# ============================================================

class AssemblyCell:

    def __init__(self, cell_id, queue_size=3):

        # Assembly Cell 번호
        self.cell_id = cell_id

        # Cell별 FIFO Queue
        # 최대 3개의 Task를 저장
        self.task_queue = queue.Queue(maxsize=queue_size)

        # 현재 작업 중인 Task
        self.current_task = None

        # 작업 중인지 여부
        self.is_working = False

        # 현재 작업 시작 시간
        self.start_time = None


    def add_task(self, task):

        # Queue에 Task 추가
        self.task_queue.put(task)


    def start_task(self, simulation_time):

        # Queue가 비어있으면 작업하지 않음
        if self.task_queue.empty():
            return

        # FIFO로 가장 먼저 들어온 Task 가져오기
        self.current_task = self.task_queue.get()

        self.is_working = True

        self.start_time = simulation_time

        print(
            f"[Time: {simulation_time:.1f}] "
            f"Cell {self.cell_id} - "
            f"Task {self.current_task.task_id} "
            f"({self.current_task.kit_id}) START"
        )


    def complete_task(self, simulation_time):

        print(
            f"[Time: {simulation_time:.1f}] "
            f"Cell {self.cell_id} - "
            f"Task {self.current_task.task_id} "
            f"({self.current_task.kit_id}) COMPLETE"
        )

        # 완료된 Task 저장
        completed_task = self.current_task

        # 현재 작업 초기화
        self.current_task = None
        self.is_working = False
        self.start_time = None

        # Queue 작업 완료 처리
        self.task_queue.task_done()

        # 완료된 Task를 Queue 뒤에 다시 추가
        self.task_queue.put(completed_task)

        # 완료된 Task 반환
        return completed_task


    def update(self, simulation_time):

        # 현재 작업이 없다면 새 작업 시작
        if not self.is_working:

            self.start_task(simulation_time)

            return None


        # 작업 중이라면 경과 시간 계산
        elapsed_time = simulation_time - self.start_time


        # 작업시간이 끝났는지 확인
        if elapsed_time >= self.current_task.processing_time:

            return self.complete_task(simulation_time)


        return None
