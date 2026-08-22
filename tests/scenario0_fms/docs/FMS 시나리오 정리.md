# FMS 시나리오 정리

## 1. 목적

본 문서는 Isaac Sim 기반 AMR 부품 공급 시스템의 `Scenario 0`를 정의한다.

최종 목표는 Multi-AMR 기반 FMS이지만, `Scenario 0`에서는 AMR 1대로 FMS와 NVIDIA cuOpt를 이용한 부품 키트 운송 경로 생성을 우선 검증한다.

## 2. 시연 구성

- `Assembly Cell`: 3개
- `AMR`: 1대
- `Parts Supermarket`: 1개
- AMR을 제외한 `Assembly Cell` 3곳과 `Parts Supermarket` 1곳의 위치는 고정한다.
- AMR의 위치는 주행 상태에 따라 변경된다.

## 3. 부품 운송 작업

`Assembly Cell`은 조립에 필요한 부품 키트를 요청한다. AMR은 `Parts Supermarket`에서 해당 부품 키트를 가져와 요청한 `Assembly Cell`에 전달한다.

따라서 각 요청은 다음 `Pickup-Delivery` 작업으로 표현한다.

```text
Parts Supermarket (Pickup) -> Assembly Cell (Delivery)
```

AMR의 적재 용량은 초기 검증 조건에 따라 `1`로 본다. 따라서 한 번에 부품 키트 하나를 운송하며, 각 `Assembly Cell` 배송 전에 `Parts Supermarket`에서 해당 키트를 먼저 수령해야 한다.

## 4. FMS와 cuOpt의 역할

### FMS

- `Assembly Cell`의 부품 키트 요청을 접수한다.
- 요청을 `Task`로 생성하여 최대 길이 `10`의 작업큐에 저장하고 상태를 관리한다.
- `Parts Supermarket`과 요청한 `Assembly Cell`을 방문할 작업 위치로 구성한다.
- 고정 작업 위치, AMR의 현재 위치, 운송 조건을 cuOpt 입력으로 변환한다.
- AMR이 새 경로를 요청하면 현재 작업큐 전체를 한 번에 cuOpt에 전달한다.
- cuOpt가 반환한 최적 작업 순서와 `route_stops`를 보관한다.
- 다음 `route_stop`을 하나씩 Nav2의 Navigation Goal로 하달한다.

### cuOpt

- FMS에서 전달받은 작업 위치와 운송 제약을 이용해 최적화를 수행한다.
- 각 요청에서 `Pickup`이 `Delivery`보다 먼저 수행되도록 한다.
- AMR의 적재 용량을 준수한다.
- 거리, 긴급도, 완료기한과 작업시간을 고려한다.
- 요청된 작업 위치의 최적 방문 순서를 `route_stops`로 FMS에 반환한다.

cuOpt는 AMR을 직접 주행시키지 않으며, 최종 목적지까지의 중간경로를 생성하지 않는다.

### Nav2

- FMS가 하달한 다음 `route_stop`을 Navigation Goal로 받는다.
- 현재 AMR 위치에서 해당 목적지까지의 실제 `path`와 중간 `waypoints`를 생성한다.
- 지도, costmap과 장애물을 반영하여 주행 경로를 계획한다.
- Controller를 통해 `/cmd_vel`을 생성한다.

## 5. Scenario 0 처리 흐름

```text
Assembly Cell 1/2/3
        |
        | Task Request
        v
 FMS Task Queue (max 10)
        |
        | AMR Route Request triggers Solve
        | Tasks + AMR State + Constraints
        v
      cuOpt
        |
        | Task Sequence + Route Stops
        v
       FMS
        |
        | Next Route Stop as Navigation Goal
        v
      Nav2
        |
        | Path / Intermediate Waypoints / cmd_vel
        v
 Isaac Sim AMR
```

세 `Assembly Cell`이 모두 요청하고 AMR이 키트를 하나씩만 운송할 수 있다면, cuOpt가 반환할 결과는 개념적으로 다음 형태가 된다.

```text
Current AMR Position
-> Parts Supermarket -> Assembly Cell 2
-> Parts Supermarket -> Assembly Cell 1
-> Parts Supermarket -> Assembly Cell 3
```

위 순서의 `Assembly Cell 2 -> 1 -> 3`은 예시이며, 실제 순서는 위치와 이동 비용을 이용해 cuOpt가 계산한다.

## 6. 용어와 경로 계획의 책임

- `Task Sequence`: cuOpt가 결정한 부품 운송 작업의 수행 우선순위
- `Route Stop`: Pickup 또는 Delivery가 수행되는 작업 위치. cuOpt가 방문 순서를 결정한다.
- `Navigation Goal`: FMS가 다음 `route_stop`의 좌표로 생성하여 Nav2에 하달하는 목표
- `Path`: Nav2가 현재 위치부터 Navigation Goal까지 생성한 실제 이동 경로
- `Waypoint`: Nav2 경로를 구성하는 중간 지점

따라서 cuOpt 결과를 `waypoints`라고 부르지 않고 `route_stops`라고 부른다. cuOpt는 어디를 어떤 순서로 방문할지 결정하고, Nav2는 각 위치까지 어떻게 이동할지 결정한다.

## 7. 초기 경로 최적화 기준

- 작업 위치 간 이동 비용은 우선 Euclidean distance로 계산한다.
- 각 부품 요청의 `Pickup-Delivery` 선행 조건을 유지한다.
- AMR capacity `1` 제약을 유지한다.
- `Task`의 `urgency`, `deadline`, `service_time`을 최적화 조건에 포함한다.
- FMS가 최적화 대상 작업과 작업 위치를 결정하고, cuOpt는 그 입력에 대한 최적 방문 순서를 결정한다.

## 8. 통신 및 내부 API

- Assembly Cell에서 FMS로 전달되는 `Task` 요청은 ROS 2 Topic을 사용한다.
- AMR에서 FMS로 전달되는 경로 요청은 ROS 2 Service를 사용하며 cuOpt Solve의 트리거가 된다.
- FMS와 cuOpt는 같은 FMS PC에서 Python 내부 API로 연결한다.
- FMS는 cuOpt 결과 전체를 보관하고, 실제 주행 시 다음 `route_stop`만 Nav2에 전달한다.
- FMS와 Nav2는 `NavigateToPose` ROS 2 Action으로 연결한다.
- Nav2와 Isaac Sim AMR은 `/cmd_vel`, `/odom`, `/tf`, `/scan`을 사용한다.

현재 ROS 2 통신 데이터 타입은 임시 상태이며, 향후 custom interface로 확정한다. `defined.py`는 ROS interface 생성 전 Scenario 0 실행에 필요한 `Task`, `AMRState`, 최적화 요청·결과와 고정 데이터를 정의한다.

## 9. 현재 구현 상태

구현된 부분은 다음과 같다.

1. `Scenario0FMSNode`와 최대 길이 `10`의 Task Queue
2. Scenario 0 재현용 고정 Task 10개 적재
3. AMR의 ROS 2 Service 요청을 cuOpt Solve 트리거로 사용
4. FMS의 `OptimizationRequest` 생성
5. cuOpt Pickup-Delivery, capacity, 거리, time window와 service time 구성
6. cuOpt 결과를 `OptimizationResult`의 `task_sequence`, `route_stops`로 반환
7. FMS의 최신 최적화 결과 보관

현재 Assembly Cell 요청은 `std_msgs/msg/String`, AMR 경로 요청은 `std_srvs/srv/Trigger`를 사용하는 임시 구조다. Python 문법과 고정 데이터는 검증했지만 현재 기본 환경에 `cudf`가 없어 실제 cuOpt Solve는 아직 실행하지 못했다.

Nav2 Goal Dispatcher, 실제 주행, `/cmd_vel`, Isaac Sim 센서와 AMR 상태 연동은 후속 단계에서 진행한다.



## 10. 현재 확정된 조건

- `Assembly Cell` 3개, `Parts Supermarket` 1개, AMR 1대로 시연한다.
- FMS Task Queue의 최대 길이는 `10`이다.
- Scenario 0 재현을 위해 Task 10개를 코드에 고정하고 입력 순서를 주석으로 표시한다.
- 고정 Task는 실제 요청과 동일하게 FMS 작업큐에 적재한다.
- Task 수신만으로 cuOpt Solve를 실행하지 않는다.
- AMR의 경로 요청이 cuOpt Solve를 트리거한다.
- cuOpt에는 요청 시점의 작업큐 전체를 한 번에 전달한다.
- AMR capacity는 `1`이며 각 배송 전에 Parts Supermarket Pickup이 필요하다.

## 11. 추가 확정이 필요한 항목

다음 항목은 구현 전에 확정해야 한다.

- `Assembly Cell` 3곳과 `Parts Supermarket` 1곳의 실제 좌표
- AMR의 초기 위치 또는 depot 좌표
- 모든 배송 완료 후 depot 복귀 필요 여부
- 실제 ROS 2 custom message와 service의 필드
- 부품 키트 적재·하재 시간의 실제 값
- 긴급도를 완료기한 또는 목적함수 가중치로 변환하는 정책
- 최적화 이후 Task 상태 변경 및 큐 제거 시점
- AMR 요청에 포함할 현재 pose와 load state

## 12. 다음 작업

1. Assembly Cell Task 요청과 AMR Route 요청을 위한 ROS 2 custom interface 정의
2. `Trigger`를 AMR ID, 현재 pose와 load state를 전달할 Route Request service로 교체
3. cuOpt 실행 환경에서 `OptimizationRequest -> OptimizationResult` 실제 동작 검증
4. FMS가 `route_stops`를 순서대로 Nav2에 전달하는 Goal Dispatcher 구현
5. Nav2 결과에 따른 Task 상태와 Task Queue 갱신

## 13. 향후 확장

`Scenario 0`를 검증한 후 AMR 수를 늘려 다음 기능으로 확장한다.

- Multi-AMR 작업 할당
- AMR별 최적 방문 순서 생성
- AMR 상태와 위치를 반영한 재최적화
- 추가 운송 제약과 예외 상황 처리
