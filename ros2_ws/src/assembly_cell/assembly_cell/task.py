class Task:

    # ====================================================
    # 도형별 작업 시간
    # ====================================================

    PROCESSING_TIMES = {
        "STAR": 3.0,
        "SQUARE": 5.0,
        "TRIANGLE": 8.0,
        "CIRCLE": 10.0,
    }


    # ====================================================
    # Task ID 자동 생성용
    # ====================================================

    _next_task_id = 1


    def __init__(self, shape):

        # 입력된 도형 이름을 대문자로 통일
        self.shape = shape.upper()

        # 지원하지 않는 도형인지 확인
        if self.shape not in self.PROCESSING_TIMES:
            raise ValueError(
                f"지원하지 않는 Task 도형입니다: {self.shape}"
            )


        # ====================================================
        # Task ID 자동 부여
        # ====================================================

        self.task_id = Task._next_task_id

        Task._next_task_id += 1


        # ====================================================
        # Kit ID
        #
        # 예:
        # STAR -> KIT_STAR
        # CIRCLE -> KIT_CIRCLE
        # ====================================================

        self.kit_id = f"KIT_{self.shape}"


        # ====================================================
        # 도형에 따른 작업 시간 자동 설정
        # ====================================================

        self.processing_time = self.PROCESSING_TIMES[
            self.shape
        ]


    def __repr__(self):

        return (
            f"Task("
            f"id={self.task_id}, "
            f"shape={self.shape}, "
            f"processing_time={self.processing_time}"
            f")"
        )