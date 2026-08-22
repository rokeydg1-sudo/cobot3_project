# Scenario 0 FMS-cuOpt 경로 생성

기존 `cuopt_fms_smoke` 코드를 수정하지 않고 추가한 Scenario 0 전용 코드다.

## 시나리오

- `Task Queue`: 최대 10개
- `AMR`: 1대, capacity `1`
- `Parts Supermarket`: 1개
- `Assembly Cell`: A, B, C 3개
- 각 Task는 `Parts Supermarket (Pickup) -> Assembly Cell (Delivery)`로 수행
- 큐에 담긴 Task 전체를 cuOpt에 한 번에 전달
- 거리, 긴급도, 완료기한 및 서비스 시간을 고려해 방문 순서 최적화
- 모든 작업 완료 후 AMR 시작 위치로 복귀

## 코드 구성

- `scenario0_models.py`: `Task`, `Location`, 고정 좌표 정의
- `scenario0_fms.py`: `Scenario0FMSNode`, 고정 Task 10개 자동 적재, Assembly Cell Task subscription, AMR Solve Trigger service와 일괄 최적화 callback
- `scenario0_cuopt_solver.py`: 거리·시간 행렬과 cuOpt 제약 구성, 결과 변환

## Task 기준

`Task`는 다음 정보를 가진다.

- `task_id`: 작업 식별자
- `delivery_cell`: 배송 대상 Cell
- `kit_id`: 요청 부품 키트
- `urgency`: `1`(일반)부터 `5`(최우선)
- `requested_at`: 요청 발생 시각
- `deadline`: 작업 완료기한
- `service_time`: Cell에서 배송 완료에 필요한 시간

cuOpt는 이동 거리 합계를 최소화한다. 긴급도는 등급별 목표 완료시간으로 변환하고, `deadline`과 비교해 더 빠른 값을 Delivery time window로 적용한다. `service_time`은 전체 작업 완료시간 계산에 포함한다.

## 실행

`cudf`와 `cuopt`가 설치된 Python 환경에서 실행한다.

```bash
python3 scenario0_fms.py
```

JSON 파일은 사용하지 않는다. Scenario 0를 동일하게 재현할 수 있도록 Task 10개를 코드에 고정하고, Node 초기화 시 `add_task()`를 통해 큐에 자동 적재한다. 실제 Isaac Sim stage 좌표가 확정되면 `scenario0_models.py`의 좌표만 교체한다.

## Assembly Cell Task 수신

FMS는 `/assembly/task_request` Topic에서 `std_msgs/msg/String`을 수신한다. JSON 대신 다음 순서의 쉼표 구분 문자열을 사용한다.

```text
task_id,cell_id,kit_id,urgency,requested_at,deadline,service_time
```

예시:

```text
task_01,cell_a,kit_a,5,0,120,15
```

Node는 유효한 Task를 수신 순서대로 최대 10개까지 저장한다. 중복 `task_id`, 잘못된 Cell, 잘못된 필드 또는 큐 초과 요청은 거부한다. 현재 재현 모드에서는 시작과 동시에 고정 Task 10개가 큐를 채운다.

Task가 들어오는 것만으로 cuOpt Solve를 실행하지 않는다. AMR이 `/amr/request_route`의 `std_srvs/srv/Trigger` service를 요청하면 현재 큐 전체를 cuOpt에 한 번 전달한다.

```bash
ros2 service call /amr/request_route std_srvs/srv/Trigger "{}"
```

큐가 비었거나 최적화가 이미 시작된 경우 요청을 거부한다.
