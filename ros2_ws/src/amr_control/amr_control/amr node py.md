3.2 amr_node.py

기존 역할

기존 AMR 제어에서는 목적지 좌표를 직접 /amr/goal로 전달하고,
/amr/odom의 현재 위치와 목표 위치 사이의 거리를 직접 계산하여 도착 여부를 판단했다.

현재 역할

AMR Node는 이제 직접 주행 경로를 계산하지 않는다.

FMS
 ↓
RequestTask Service
 ↓
AMR Node
 ↓
NavigateToPose
 ↓
Nav2

주요 변경

1. FMS Pull 방식 적용

AMR이 IDLE 상태일 때 일정 주기로 FMS에 다음 Task를 요청한다.

Service:

/fms/request_task

AMR이 전달하는 상태:

amr_id

현재 state

현재 task

현재 x/y 위치

load state

2. Nav2 ActionClient 적용

기존:

/amr/goal Publish

현재:

NavigateToPose Action

실제 Action:

/navigate_to_pose

Goal frame:

map

3. 이동 성공 판정 변경

기존:

/amr/odom
 ↓
목표와 거리 계산
 ↓
threshold 이하이면 성공

현재:

NavigateToPose
 ↓
Feedback
 ↓
Action Result
 ↓
SUCCEEDED / ABORTED / CANCELED

즉 이동 성공 여부를 AMR Node가 임의로 계산하지 않고 Nav2의 결과를 사용한다.

4. AMR 상태 Event 추가

Topic:

/amr/status

대표 상태:

READY
TASK_ASSIGNED
MOVING_TO_PICKUP
ARRIVED_PICKUP
LOADING
LOAD_COMPLETE
MOVING_TO_DELIVERY
ARRIVED_DELIVERY
DELIVERY_COMPLETE
MISSION_COMPLETE
IDLE
TASK_FAILED

FMS는 이 이벤트를 이용해 AMR 상태와 Active Task를 관리한다.

5. MultiThreadedExecutor 적용

동시에 처리해야 하는 통신이 증가하여
다음 Callback을 분리했다.

/amr/odom

FMS Service

Task Request Timer

Nav2 Action
