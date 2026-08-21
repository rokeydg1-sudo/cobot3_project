class Task:

    def __init__(self, task_id, kit_id, processing_time):

        # 작업을 구분하기 위한 ID
        self.task_id = task_id

        # 작업에 필요한 Kit
        self.kit_id = kit_id

        # 작업에 필요한 시간
        self.processing_time = processing_time