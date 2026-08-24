# AMR Nav2 통신 변경 내역

## 대상 파일

- `ros2_ws/src/navigation/amr_withnav2.py`

## 변경 목적

기존 `NavigateToPose` 클라이언트는 유지하면서, FMS 및 Isaac Sim과의 ROS 2
통신을 현재 워크스페이스의 AMR 통합 구조와 일치시켰다.

## 주요 변경 사항

- 기존 `ExecuteMission` Action Server와 FMS Push 방식을 제거했다.
- `RequestTask` Service를 사용한 AMR Pull 방식으로 변경했다.
- 문자열 기반 `/amr/status`를 `AMRStatus` 메시지로 변경했다.
- 코드 내부의 `LOCATIONS` 좌표 정의를 제거했다.
- FMS가 반환한 Pickup/Delivery 물리 좌표를 사용하도록 변경했다.
- Isaac Sim의 `/amr/odom`을 구독해 현재 위치와 초기 통신 상태를 확인한다.
- Pickup 이동, 적재, Delivery 이동, 완료 및 IDLE 복귀 상태 전이를 추가했다.
- Nav2에는 `/navigate_to_pose` Action으로 목적지를 전달한다.
- Nav2가 경로 계획과 주행 제어를 수행하고 `/cmd_vel`을 발행하는 구조로 정리했다.
- Isaac Sim 시간과 맞추기 위해 `use_sim_time`의 기본값을 `true`로 설정했다.

## 현재 통신 흐름

```text
Isaac Sim /amr/odom
  → AMR 상태 확인
  → /fms/request_task
  → Pickup/Delivery 좌표 수신
  → Nav2 /navigate_to_pose
  → Nav2 /cmd_vel
  → Isaac Sim 이동
  → /amr/status
  → Mission 완료 후 IDLE
```

## 주의사항

- 검증된 `amr_withnav2.py`의 `NavigateToPose` 통신을 최종 실행 노드인
  `amr_control/amr_node.py`에 통합했다.
- `amr_node.py`가 FMS 작업 요청, 상태 발행, Isaac odom 수신 및 Nav2 주행을
  모두 담당한다. 검증용 `amr_withnav2.py`와 동시에 실행하지 않는다.
- 기존 `/compute_path_to_pose` 및 `/amr/path_command` 방식은 제거했다.
- AMR 상태는 상태 전환 시 이벤트로 전달하며, 당시 Isaac Sim odom 좌표를
  함께 담는다. 주행 중 1초 단위 `POSITION_UPDATE`는 제거했다.
- `AMRStatus.timestamp`에는 시스템 시계 기준으로 AMR이 상태 데이터를 패킹한
  시각을 기록한다.
  FMS는 AMR별 최신 timestamp보다 오래된 상태를 무시하고 기존 상태 객체의
  필드를 직접 갱신한다.
- Nav2 목적지는 작업 스레드와 블로킹 대기 대신 Action Future 콜백으로 순차
  처리하며, 로딩 대기도 `sleep` 대신 ROS timer를 사용한다.
- ROS 콜백 처리는 `MultiThreadedExecutor`의 두 스레드로 유지한다.
- Delivery 도착 시 상태와 이벤트가 모두 `ARRIVED_DELIVERY`가 되도록 상태
  전이를 정리했다.
- FMS는 `AMRRuntimeState`를 `amr_id`별로 생성해 여러 AMR의 최신 상태와
  위치를 독립적으로 관리한다.
- FMS 작업 할당 전 `IDLE`, 빈 작업 ID, `EMPTY` 적재 상태 및 저장된 AMR
  상태를 검증한다.
- cuOpt의 `requested_at`은 Assembly 메시지가 FMS에 도착한 시각을 FMS 시작
  기준 상대시간으로 기록한다.
- Scenario 0 정책에 따라 할당된 작업은 응답 시 대기 큐에서 제거하며, Nav2
  시간 초과 시 goal을 자동 취소하지 않는다.
- AMR은 `IDLE`이고 실행 중이거나 응답 대기 중인 작업이 없을 때 1초 주기의
  Timer로 FMS에 다음 작업을 요청한다.
- `RequestTask`의 고정 Pickup/Delivery 필드를 `Location[] destinations`로
  변경했다. FMS는 논리 위치와 물리 좌표를 묶어 실행 순서대로 반환하고,
  AMR은 목록을 순회하며 각 위치를 `NavigateToPose`로 순차 처리한다.
- `Location`은 `name`, `x`, `y`, `yaw`로만 구성한다. `name` 값은
  `AMR_START`, `PARTS_SUPERMARKET`, `ASSEMBLY_CELL_A/B/C`를 사용하며 ROS
  메시지 안에는 별도의 위치 상수를 중복 정의하지 않는다.
- `navigation`은 독립 ROS 2 패키지이므로 빌드 후 워크스페이스를 source한다.
- Nav2와 Isaac Sim은 동일한 `ROS_DOMAIN_ID`를 사용해야 한다.

## 정적 맵 추가

- `standalone_amr_world_nav2.py`의 20 m × 12 m 월드와 외곽 벽을 기준으로
  `maps/factory_map.pgm` 및 `maps/factory_map.yaml`을 생성했다.
- 맵 해상도는 0.05 m/pixel, 원점은 `[-10.0, -6.0, 0.0]`이다.
- Supermarket과 Cell 구역은 목적지로 진입해야 하는 바닥 표시이므로 점유
  장애물에서 제외했다.
- 월드 크기나 벽 구조가 변경되면 `tools/generate_factory_map.py`를 실행해
  맵을 다시 생성한다.
- `setup.py`에 맵 설치 규칙을 추가해 패키지 share 디렉터리에서도 사용할 수
  있도록 했다.
- `nav2.launch.py`의 `map`과 `params_file` 기본값을 각각 패키지 내부의
  `maps/factory_map.yaml`, `config/nav2_params.yaml`로 연결했다.




## 향후 구현 기능

- FMS가 10초마다 가동 중인 AMR의 상태를 조회하는 Heartbeat를 구현한다.
- FMS에서 AMR별 마지막 응답 시각과 `ONLINE`/`OFFLINE` 통신 상태를 관리한다.
- 긴급 작업 발생 시 전체 AMR의 상태를 조회하고 적합한 AMR을 선정한다.
- AMR 장애나 긴급도 변경에 따른 작업 예약 및 재할당 정책을 구현한다.
