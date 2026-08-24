3.4 assembly_node.py

추가 보완 정책

Nav2 연동 이후 Task 유실 문제를 줄이기 위해
Assembly → FMS 통신 정책도 한 번 더 수정했다.

기존에는 Assembly가 Task를 보냈을 때
FMS가 실행되지 않거나 응답할 수 없는 상태라면
Task 전달이 끊길 가능성이 있었다.

현재는:

Assembly Task 생성
      ↓
FMS 연결 확인
      ↓
FMS 없음
      ↓
[FMS WAIT]
      ↓
Task 유지
      ↓
일정 시간 후 재전송

형태로 동작한다.

핵심은 재전송 시 새로운 Task를 생성하는 것이 아니라,
동일한 task_id의 기존 Task를 다시 요청한다는 것이다.

따라서 Assembly와 FMS의 정책은 서로 맞물린다.

Assembly
= 동일 Task 재전송 가능

FMS
= 동일 task_id는 중복 등록하지 않음

이 조합으로 인해 FMS 실행 순서가 늦거나
일시적으로 통신이 끊겨도 Task가 사라지지 않도록 보완했다.
