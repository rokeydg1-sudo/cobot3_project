# Codex 작업 컨텍스트 --- FMS / AMR Mission Node 통합 설계

## 0. 문서 목적

이 문서는 현재 통합 프로젝트에서 **FMS, cuOpt, AMR Mission Node, Nav2,
Isaac Sim 간 역할과 인터페이스를 정리하고, Codex가 기존 Workspace를
분석한 뒤 통합 작업을 진행하기 위한 최신 설계 기준**이다.

### Codex 적용 원칙

1.  **이 문서를 현재 프로젝트의 최신 설계 기준으로 사용한다.**
2.  기존 코드가 이 문서와 충돌하면 즉시 대규모 수정하지 말고 **차이점을
    먼저 분석해서 보고한다.**
3.  기존 Workspace의 패키지, ROS2 Node, msg/srv/action, launch, config
    구조를 최대한 재사용한다.
4.  새로운 Interface를 임의로 추가하기 전에 기존 Interface의 재사용
    가능성을 먼저 확인한다.
5.  분석 결과와 수정 대상 파일을 먼저 제시한 뒤 코드 작업을 시작한다.
6.  현재는 AMR 1대로 기능을 검증할 수 있지만, **향후 Multi-AMR 확장을
    고려하여 ID와 상태 관리 및 Interface를 설계한다.**

------------------------------------------------------------------------

# 1. 프로젝트 목표

현재 프로젝트는 **Assembly Cell에서 발생한 부품 운송 요청을 FMS가
종합하고, cuOpt가 작업 우선순위를 결정한 뒤, AMR이 Parts Supermarket에서
Kit를 가져와 요청 Cell로 배송하는 구조**를 목표로 한다.

기본 흐름은 다음과 같다.

``` text
Assembly Cell
    ↓ Task Request
FMS
    ↓ 전체 운송 Task 관리
cuOpt
    ↓ Logical Ordered Tasks
FMS
    ↓ Task + Pickup/Delivery 물리 좌표
AMR Mission Node
    ↓
Nav2
    ↓ 실제 Path / 주행 제어
Isaac Sim AMR
```

Mission 완료 후 AMR Mission Node가 다시 FMS에 다음 Task를 요청한다.

------------------------------------------------------------------------

# 2. 시스템 역할 분리

## Assembly Cell

``` text
생산 Task 생성
생산 Queue 관리
작업 상태 관리
Processing
현재 작업에 필요한 Kit 운송 요청
```

각 Assembly Cell은 독립적으로 자신의 생산 작업을 관리한다.

FMS에 Assembly Queue 전체를 넘기는 것이 아니라 **현재 실행할 Task에
필요한 Kit 운송 요청**을 전달한다.

## FMS

``` text
여러 Assembly Cell의 운송 Task 종합
FMS Task Queue 관리
AMR 할당
AMR 마지막 상태 저장 및 관리
cuOpt 호출
Location DB 관리
논리 위치 → 물리 좌표 변환
Parts Supermarket Pickup 추가
RequestTask 처리
긴급 Task 발생 시 필요에 따라 적극적인 재할당 판단
```

## cuOpt

``` text
어떤 Task를 어떤 순서로 수행할지 최적화
```

향후 Multi-AMR에서는:

``` text
Task 순서 최적화
+
AMR Assignment 최적화
```

까지 확장할 수 있다.

cuOpt는 물리 좌표, Nav2 Path 또는 중간 Waypoint를 생성하지 않는다.

## AMR Mission Node

이 문서에서 **AMR Mission Node는 AMR의 Mission 실행 및 통신을 담당하는
AMR Node**로 본다.

``` text
FMS에 다음 Task 요청
FMS에서 받은 Task 저장 및 실행
Pickup → Delivery Mission 관리
Nav2를 통한 실제 이동 수행
상태 변화 발생 시 FMS에 이벤트 전달
Mission 결과 전달
Mission 완료 후 다음 Task 요청
Isaac Sim AMR과 필요한 ROS2 통신 수행
```

## Nav2

``` text
실제 목적지까지 Path 계산
중간 경로 생성
장애물 회피
주행 제어
```

## Isaac Sim

``` text
AMR의 실제 물리 이동 시뮬레이션
```

------------------------------------------------------------------------

# 3. Task 구조

기존 `defined.py`의 dataclass 구조를 유지한다.

`service_time`은 사용하지 않고 **`processing_time`으로 통일한다.**

``` text
Task
├─ task_id
├─ kit_id
├─ delivery_cell
├─ urgency
├─ requested_at
├─ deadline
├─ processing_time
├─ quantity
└─ status
```

`processing_time`은 Kit가 Assembly Cell에 도착한 후 해당 생산 작업을
수행하는 예상 작업시간이다.

예:

``` text
KIT_STAR     → 3.0
KIT_SQUARE   → 5.0
KIT_TRIANGLE → 8.0
KIT_CIRCLE   → 10.0
```

`shape`는 필수 통신 필드가 아니라 Kit를 설명하는 메타데이터 성격으로
본다.

------------------------------------------------------------------------

# 4. Assembly Queue와 FMS Queue

두 Queue는 서로 다른 책임을 가진다.

``` text
Assembly Queue
= 생산 예정 작업

FMS Queue
= 현재 운송이 필요한 작업
```

실제 통합 구조:

``` text
Assembly Request
        ↓
FMS Task Queue
```

FMS가 시작할 때 고정 Task를 미리 넣어두는 구조는 기능 테스트 및
재현용으로만 사용한다.

------------------------------------------------------------------------

# 5. 일반 Task 할당은 Pull 방식

일반적인 Task 할당은 FMS가 AMR을 계속 polling하면서 Push하는 방식으로
구현하지 않는다.

기본 흐름:

``` text
AMR Mission 완료
        ↓
AMR Mission Node
        ↓
RequestTask
        ↓
FMS
        ↓
Task Queue 확인
        ↓
cuOpt
        ↓
Task 결정
        ↓
Pickup / Delivery 생성
        ↓
논리 위치 → 물리 좌표
        ↓
RequestTask Response
        ↓
AMR Mission Node
        ↓
Mission 실행
```

AMR의 Task Request는 다음 두 의미를 동시에 가진다.

``` text
"현재 새로운 작업을 받을 수 있습니다."
+
"다음 Task를 주세요."
```

따라서 일반 Scheduling에서는 FMS가 모든 AMR의 IDLE 여부를 계속 polling할
필요가 없다.

------------------------------------------------------------------------

# 6. AMR 상태 관리

AMR의 Runtime 상태는 AMR Mission Node가 판단한다.

상태가 실제로 변경되었을 때만 FMS에 **이벤트 기반 ROS2 통신**으로
전달한다.

예:

``` text
IDLE → BUSY
BUSY → MOVING_TO_PICKUP
MOVING_TO_PICKUP → ARRIVED_PICKUP
ARRIVED_PICKUP → LOADED
LOADED → MOVING_TO_DELIVERY
MOVING_TO_DELIVERY → DELIVERED
BUSY → ERROR
```

상태 후보:

``` text
IDLE
BUSY
MOVING_TO_PICKUP
ARRIVED_PICKUP
LOADED
MOVING_TO_DELIVERY
DELIVERED
ERROR
```

정확한 Enum은 기존 코드의 상태 정의를 먼저 확인하고 필요할 때 정리한다.

FMS는 각 AMR이 전달한 **마지막 상태를 저장하고 관리**한다.

예:

``` text
AMR_01
├─ state = MOVING_TO_DELIVERY
├─ current_task_id = task_17
└─ load_state = LOADED
```

Multi-AMR에서는 다음과 같은 Registry 형태로 확장할 수 있다.

``` text
AMR_01 → ...
AMR_02 → ...
AMR_03 → ...
```

------------------------------------------------------------------------

# 7. State Event와 Scheduling Trigger를 분리

AMR State Event는 Scheduling Trigger가 아니다.

잘못된 흐름:

``` text
AMR State Event
      ↓
cuOpt 실행
```

사용하지 않는다.

올바른 흐름:

``` text
AMR State 변경
      ↓
State Event
      ↓
FMS의 해당 AMR 상태 갱신
```

일반 작업의 Scheduling Trigger는:

``` text
RequestTask
```

이다.

Mission 완료 시 개념적인 순서:

``` text
Mission Complete
      ↓
AMR 상태 변경
      ↓
State Event → FMS
      ↓
RequestTask → FMS
      ↓
Scheduling
```

------------------------------------------------------------------------

# 8. Heartbeat

필요한 경우 AMR Mission Node는 FMS에 주기적인 Heartbeat를 전달한다.

Heartbeat의 목적은:

``` text
AMR Mission Node 생존 확인
통신 단절 / OFFLINE 판단 보조
```

이다.

Heartbeat는 AMR 상태 Event와 별개이며 **Scheduling Trigger가 아니다.**

``` text
Heartbeat
    ↓
FMS
    ↓
last_seen 갱신
```

AMR의 실제 상태 변경은 State Event로 관리한다.

Heartbeat 주기와 timeout은 기존 통합 환경을 확인한 후 Parameter 또는
Config로 설정 가능하게 하는 방향을 검토한다.

------------------------------------------------------------------------

# 9. 긴급 Task 처리

일반 작업은 Pull 방식으로 처리한다.

하지만 긴급 Task가 발생하면 FMS가 예외적으로 적극 개입할 수 있다.

``` text
URGENT Task 발생
       ↓
FMS 내부 상태 확인
       ↓
IDLE AMR 존재?
 ├─ YES
 │   → 후보 중 최적 AMR 선택
 │
 └─ NO
     → 필요할 때만 BUSY AMR 상세 상태 확인
     → cuOpt 재최적화
     → 필요 시 Preemption 판단
```

긴급 재할당 판단 후보:

``` text
AMR 상태
현재 위치
Load State
현재 Task 진행 정도
현재 Task Priority
긴급 Task Priority
Deadline
재할당 전체 비용
```

FMS가 이미 가지고 있는 정보는 재사용한다.

예:

``` text
amr_id
current_task_id
task priority
deadline
마지막 AMR 상태
```

다음과 같은 상세 Runtime 정보는 **긴급 재할당이 실제로 필요할 때만**
AMR에서 추가 확인하는 방향을 고려한다.

``` text
current_position
load_state
mission progress
```

즉:

``` text
평상시
= 가벼운 Pull + Event State

긴급 상황
= FMS 적극 개입 + 필요 시 상세 상태 조회
```

구조다.

------------------------------------------------------------------------

# 10. 기존 ExecuteMission.action

기존:

``` text
ExecuteMission.action

string[] route
---
bool success
string message
---
string status
```

는 FMS가:

``` text
route = ["SP", "A"]
```

같은 Mission을 AMR에 Push하던 구조에서 사용했다.

새 구조에서는:

``` text
AMR → RequestTask → FMS
FMS → Task + Pickup/Delivery → AMR
```

Pull 방식으로 변경한다.

따라서:

``` text
ExecuteMission.action
→ 제거 방향
```

으로 본다.

단, Codex는 먼저 기존 Workspace에서 `ExecuteMission.action`의 실제 사용
위치와 의존성을 모두 확인한 뒤 제거 범위를 보고한다.

장시간 실제 Navigation은 향후 Nav2의:

``` text
NavigateToPose Action
```

이 담당한다.

새로운 자체 Mission Action을 추가하는 방향으로 가지 않는다.

------------------------------------------------------------------------

# 11. 핵심 Interface: RequestTask Service

AMR과 FMS 사이의 일반 Task 할당은 Service 기반 Pull 구조를 사용한다.

개념적인 Interface:

``` text
RequestTask.srv

# Request
AMRState amr_state

---

# Response
bool has_task
Task task
Location pickup
Location delivery
string message
```

정확한 msg/srv 구조는 기존 Interface를 확인한 후 확정한다.

불필요하게 기존 타입과 중복되는 새 타입을 만들지 않는다.

## Request 의미

예:

``` text
amr_id = AMR_01
state = IDLE
current_task_id = ""
```

의미:

``` text
AMR_01이 현재 새로운 작업을 받을 수 있으며
다음 Task를 요청한다.
```

## FMS 처리

``` text
RequestTask 수신
      ↓
AMR 요청 상태 확인
      ↓
Task Queue 확인
      ↓
cuOpt 실행
      ↓
다음 Task 결정
      ↓
Task의 논리 목적지 확인
      ↓
Pickup = Parts Supermarket 추가
      ↓
Pickup / Delivery 논리 위치 → 물리 좌표
      ↓
Response 반환
```

Task가 없다면 `has_task = false`를 반환할 수 있다.

Task가 없는 경우 AMR의 재요청 정책은 기존 통합 구조를 확인한 뒤
결정한다. IDLE 상태에서 무한 Service 호출 루프를 만들지 않는다.

------------------------------------------------------------------------

# 12. cuOpt 출력

cuOpt는 **논리적인 작업 순서만 반환**한다.

예:

``` text
Task 17 → CELL_A
Task 21 → CELL_C
Task 10 → CELL_B
```

cuOpt 결과:

``` text
Task 21
Task 17
Task 10
```

`OrderedTask`는 개념적으로:

``` text
sequence
task_id
delivery_cell
```

정도를 가진다.

기존 `OrderedTask`에 다음 필드가 있다면 제거 방향으로 검토한다.

``` text
x
y
yaw
```

cuOpt가 다음을 담당하지 않기 때문이다.

``` text
물리 좌표 변환
Nav2 Path 계산
중간 Waypoint 계산
```

------------------------------------------------------------------------

# 13. Location의 소유권

현재 통합 설계에서는 **논리 위치 → 물리 좌표 변환 책임을 FMS가 가진다.**

예:

``` text
CELL_A
   ↓ FMS Location DB
(7.0, 3.5)
```

향후 별도의 Location Manager / Map Manager 모듈로 분리할 수 있지만 현재
책임은 FMS에 둔다.

즉:

``` text
cuOpt
= Logical Task / Location

FMS
= Logical → Physical

AMR / Nav2
= Physical Goal을 이용한 실제 주행
```

이다.

------------------------------------------------------------------------

# 14. Parts Supermarket Pickup

cuOpt는 배송 Task의 논리적 우선순위를 결정한다.

예:

``` text
Task 17 → CELL_A
```

실제 운송 Mission으로 변환할 때 FMS가 Pickup을 추가한다.

``` text
pickup = SP
delivery = CELL_A
```

그리고 FMS가 Location DB를 사용해:

``` text
SP
→ (-7.0, 0.0)

CELL_A
→ (7.0, 3.5)
```

처럼 물리 좌표로 변환한다.

즉 Parts Supermarket은:

``` text
cuOpt가 자동 추가하지 않음
Nav2가 자동 추가하지 않음
AMR이 임의로 판단하지 않음
```

**FMS가 운송 업무 규칙으로 추가한다.**

------------------------------------------------------------------------

# 15. FMS → AMR 반환 데이터

개념적인 Response 예:

``` text
has_task = true

task:
  task_id = task_17
  kit_id = KIT_STAR
  delivery_cell = CELL_A
  processing_time = 3.0

pickup:
  location_id = SP
  x = -7.0
  y = 0.0

delivery:
  location_id = CELL_A
  x = 7.0
  y = 3.5
```

AMR Mission Node는 다음을 전달받는다.

``` text
어떤 Task인지
어떤 Kit인지
Pickup은 어디인지
Delivery는 어디인지
```

------------------------------------------------------------------------

# 16. Nav2 Path는 FMS Response에 포함하지 않음

FMS가 AMR에 반환하는 Navigation 정보는:

``` text
Pickup 목적지
Delivery 목적지
```

까지다.

다음 정보는 FMS Response에 넣지 않는다.

``` text
중간 Waypoint
Global Path
Local Path
장애물 회피 경로
```

개념:

``` text
FMS
 ↓
Pickup / Delivery Physical Goal
 ↓
AMR Mission Node
 ↓
Nav2 NavigateToPose
 ↓
실제 Path 계산 및 주행
```

Nav2 Path는 현재 위치, 장애물, costmap, 통로 상태 등에 따라 달라질 수
있기 때문이다.

책임을 다음처럼 유지한다.

``` text
cuOpt
= 어떤 Task를 먼저 수행할까?

FMS
= Pickup과 Delivery가 어디인가?

Nav2
= 그 위치까지 어떻게 갈까?
```

------------------------------------------------------------------------

# 17. AMR Mission Node Mission 흐름

Task를 정상적으로 할당받은 AMR Mission Node는 개념적으로:

``` text
RequestTask
      ↓
Task 수신
      ↓
현재 Task 저장
      ↓
Pickup 이동
      ↓
Pickup 도착
      ↓
필요한 Pickup 상태 처리
      ↓
Delivery 이동
      ↓
Delivery 도착
      ↓
Mission Complete
      ↓
FMS에 결과 / 상태 전달
      ↓
RequestTask
```

을 수행한다.

상태가 변경될 때마다 필요한 State Event를 FMS에 전달한다.

------------------------------------------------------------------------

# 18. Multi-AMR 확장 원칙

현재 기능 검증은 AMR 1대로 수행할 수 있지만 Interface는 특정 한 대에
종속되지 않도록 한다.

각 AMR Mission Node는 최소한:

``` text
amr_id
```

로 구분할 수 있어야 한다.

향후:

``` text
AMR_01
AMR_02
AMR_03
```

이 존재하면 FMS는 각 AMR의 마지막 상태와 현재 Task를 관리한다.

개념:

``` text
FMS AMR Registry

AMR_01
├─ state
├─ current_task_id
└─ ...

AMR_02
├─ state
├─ current_task_id
└─ ...
```

Namespace를 AMR별로 분리할지, 공통 Topic/Service에 `amr_id`를 사용할지는
현재 Workspace의 기존 ROS2 구조를 확인한 뒤 결정한다.

------------------------------------------------------------------------

# 19. 전체 최종 Flow

``` text
Assembly Cell A/B/C
       │
       │ Task Request
       ▼
      FMS
       │
       ├── Task Queue
       ├── AMR State Registry
       ├── Location DB
       └── cuOpt
              │
              │ Logical Ordered Tasks
              ▼
             FMS
              │
              │ Pickup(SP) 추가
              │ Logical → Physical
              │
              ▲
              │ RequestTask Service
              │
       AMR Mission Node
              │
              │ Task + Pickup/Delivery 좌표
              ▼
             Nav2
              │
              │ 실제 Path 계산 / 주행 제어
              ▼
         Isaac Sim AMR
              │
              │
              └── 상태 변화 → AMR Mission Node
                                  │
                                  └── State Event → FMS

Mission 완료
      ↓
AMR Mission Node
      ↓
RequestTask
      ↓
FMS
```

------------------------------------------------------------------------

# 20. 반드시 지킬 설계 원칙

``` text
Assembly Queue != FMS Queue
```

``` text
processing_time 사용
service_time 사용하지 않음
```

``` text
일반 Task Assignment = AMR Pull 방식
```

``` text
RequestTask = 일반 Scheduling Trigger
```

``` text
State Event != Scheduling Trigger
```

``` text
Heartbeat != Scheduling Trigger
```

``` text
AMR State = 이벤트 기반으로 FMS에 전달
```

``` text
FMS = 각 AMR의 마지막 상태 저장 및 관리
```

``` text
긴급 Task = 필요할 때만 FMS 적극 개입
```

``` text
cuOpt = Logical Task Ordering
```

``` text
FMS = Logical Location → Physical Coordinate
```

``` text
FMS = Pickup(SP) 추가
```

``` text
FMS Response에 Nav2 중간 Path/Waypoint를 넣지 않음
```

``` text
AMR Mission Node = Pickup → Delivery Mission 실행 + 상태 보고
```

``` text
ExecuteMission.action = 제거 방향
```

------------------------------------------------------------------------

# 21. Codex 작업 지시

## Phase 1 --- Workspace 분석

**아직 코드를 수정하지 않는다.**

현재 Workspace를 먼저 분석하고 다음 내용을 보고한다.

1.  FMS 관련 패키지 및 주요 파일
2.  AMR Mission Node 관련 패키지 및 주요 파일
3.  Assembly → FMS 현재 ROS2 Interface
4.  FMS Task Queue 구현 위치
5.  `defined.py`와 현재 `Task` dataclass
6.  `service_time` / `processing_time` 사용 위치 전체
7.  cuOpt 호출 위치
8.  cuOpt 입력 생성 위치
9.  cuOpt 결과 및 `OrderedTask` 구조
10. `OrderedTask`의 `x`, `y`, `yaw` 사용 위치
11. 기존 `request_route` Service 정의 및 호출 위치
12. 기존 `ExecuteMission.action` 정의 및 모든 사용 위치
13. AMR Mission Node의 현재 Task 요청/수신 방식
14. AMR Mission Node의 현재 상태 변수와 상태 전환 위치
15. Mission 완료 판단 위치
16. FMS가 현재 AMR 상태를 관리하는 방식
17. Location/좌표 데이터가 현재 어디에 정의되어 있는지
18. SP(Pickup)를 현재 어느 계층이 추가하는지
19. Nav2 관련 Node/Client/Action 사용 여부
20. Isaac Sim과 AMR Mission Node 사이의 ROS2 통신
21. Multi-AMR namespace 또는 `amr_id` 지원 여부
22. launch/config/interface 패키지 구조
23. 이 문서의 최신 설계와 현재 코드가 충돌하는 부분

분석 결과를 먼저 제시한다.

------------------------------------------------------------------------

## Phase 2 --- 변경 계획 제시

Phase 1 결과를 바탕으로 다음을 보고한다.

``` text
수정할 파일
각 파일을 수정해야 하는 이유
삭제 또는 Deprecated 처리할 Interface
재사용 가능한 기존 Interface
새로 필요한 Interface
기존 테스트에 미치는 영향
권장 구현 순서
```

특히 다음 변경을 우선 검토한다.

``` text
Task.service_time → processing_time 통일

RequestTask.srv 기반 Pull 구조

AMR State Event Interface

FMS AMR State Registry

기존 request_route의 재사용/대체 범위

ExecuteMission.action 제거 범위

OrderedTask의 x/y/yaw 제거

FMS Location DB 및 Logical → Physical 변환

FMS의 Pickup(SP) 추가

AMR Mission Node의 Pickup → Delivery 실행 구조

Nav2 NavigateToPose 연결 구조
```

이 단계에서도 불필요한 대규모 리팩터링을 먼저 하지 않는다.

------------------------------------------------------------------------

## Phase 3 --- 구현

Phase 1과 Phase 2 결과가 확인된 후 코드 변경을 시작한다.

권장 순서:

``` text
1. Task / Interface 데이터 모델 정리

2. AMR State 모델 및 State Event 통신

3. FMS AMR State Registry

4. RequestTask Service

5. FMS Pull Scheduling 처리

6. cuOpt 결과 구조 정리

7. FMS Location DB / 좌표 변환

8. Pickup(SP) 추가

9. AMR Mission Node의 RequestTask Client

10. Pickup → Delivery Mission 실행 연결

11. ExecuteMission.action 의존성 제거

12. Nav2 NavigateToPose 연결 또는 기존 Navigation 구조와 통합

13. 단일 AMR E2E 검증

14. Multi-AMR 확장 가능성 검증
```

각 단계에서 기존 동작을 가능한 한 유지하고, 변경으로 인해 깨지는
Interface와 테스트를 함께 수정한다.

------------------------------------------------------------------------

# 22. Codex가 임의로 결정하지 말아야 할 항목

Workspace 분석 없이 다음을 임의로 확정하지 않는다.

``` text
새로운 ROS2 패키지 생성
새로운 msg/srv/action 남발
AMR별 Namespace 정책
Heartbeat 주기 / Timeout
Task가 없을 때 RequestTask 재요청 정책
긴급 Task Preemption 세부 정책
Multi-AMR cuOpt 모델
Nav2 세부 설정
Isaac Sim Topic 이름
Location DB 파일 형식
```

기존 구현을 먼저 확인하고 필요한 경우 변경안을 제안한다.

------------------------------------------------------------------------

# 23. 완료 기준

최소 통합 흐름이 다음과 같이 연결되어야 한다.

``` text
Assembly Task Request
        ↓
FMS Queue 등록
        ↓
AMR RequestTask
        ↓
FMS → cuOpt
        ↓
Next Task 결정
        ↓
FMS Pickup(SP) 추가
        ↓
Logical → Physical 변환
        ↓
Task + Pickup/Delivery Response
        ↓
AMR Mission Node
        ↓
Nav2
        ↓
Isaac Sim AMR
        ↓
Mission 상태 변화
        ↓
State Event → FMS
        ↓
Mission Complete
        ↓
AMR RequestTask
```

그리고 FMS는 각 AMR의 마지막 상태를 저장하고 있어야 한다.

------------------------------------------------------------------------

# 핵심 한 문장

> **Assembly가 Task를 생성하고, FMS가 Task·AMR 상태·Location을 관리하며,
> cuOpt는 논리적인 작업 순서를 결정한다. FMS가 논리 위치를 실제
> Pickup/Delivery 물리좌표로 변환해 AMR에 전달하고, AMR Mission Node는
> Nav2를 통해 실제 주행하면서 상태 변화가 발생할 때만 FMS에 이벤트를
> 전달한다. Mission이 끝나면 AMR이 RequestTask Service를 통해 Pull
> 방식으로 다음 Task를 요청한다.**
