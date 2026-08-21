import queue

from .task import Task


class AssemblyCell:

    def __init__(self, cell_id, queue_size=3):

        # Assembly Cell 번호
        self.cell_id = cell_id

        # Cell별 FIFO Task Queue
        # 최대 3개의 Task를 저장
        self.task_queue = queue.Queue(maxsize=queue_size)

        # 현재 작업 중인 Task
        self.current_task = None

        # 현재 작업 중인지 여부
        self.is_working = False

        # 현재 작업 시작 시간
        self.start_time = None


    def add_task(self, task):
        """Task를 Queue에 추가"""

        self.task_queue.put(task)


    def start_task(self, simulation_time):
        """Queue의 가장 앞 Task를 가져와 작업 시작"""

        if self.task_queue.empty():
            return

        # FIFO
        self.current_task = self.task_queue.get()

        self.is_working = True

        self.start_time = simulation_time

        print(
            f"[Time: {simulation_time:.2f}] "
            f"Cell {self.cell_id} - "
            f"Task {self.current_task.task_id} "
            f"({self.current_task.kit_id}) 작업 시작"
        )


    def complete_task(self, simulation_time):
        """현재 Task 작업 완료"""

        print(
            f"[Time: {simulation_time:.2f}] "
            f"Cell {self.cell_id} - "
            f"Task {self.current_task.task_id} "
            f"({self.current_task.kit_id}) 작업 완료"
        )

        # 완료된 Task 저장
        completed_task = self.current_task

        # 현재 작업 상태 초기화
        self.current_task = None
        self.is_working = False
        self.start_time = None

        # Queue에서 가져온 Task의 작업 완료를 알림
        self.task_queue.task_done()

        # 완료된 Task를 다시 Queue 뒤에 추가
        self.task_queue.put(completed_task)

        # 완료된 Task를 반환
        return completed_task


    def update(self, simulation_time):
        """Simulation Step마다 호출"""

        # 현재 작업이 없다면 새로운 작업 시작
        if not self.is_working:

            self.start_task(simulation_time)

            return None


        # 현재 작업이 진행 중이면
        elapsed_time = simulation_time - self.start_time


        # 작업시간이 끝났는지 확인
        if elapsed_time >= self.current_task.processing_time:

            return self.complete_task(simulation_time)


        return None

if __name__ == "__main__":
    main()