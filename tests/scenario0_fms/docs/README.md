# Scenario 0 FMS-cuOpt 실행 안내

이 문서는 Scenario 0의 FMS와 cuOpt Task 정렬 프로그램을 처음 실행하는 사용자를 위한 환경 구축 및 실행 절차다.

현재 검증 범위는 다음과 같다.

```text
고정 Task 10개
-> FMS Task Queue
-> AMR Route Request Service
-> cuOpt Task 순서 최적화
-> OrderedTask 10개 반환
-> FMS 결과 보관 및 로그 출력
```

Nav2와 Isaac Sim 주행은 아직 이 실행 범위에 포함되지 않는다.

아래의 `/path/to/cobot3_project`는 clone한 프로젝트의 실제 절대 경로로 바꾼다. 현재 검증 PC의 경로는 `/home/rokey/Desktop/EXWeek1/PRJT/cobot3_project`다.

## 1. 프로그램 실행에 필요한 것

### 운영체제 및 하드웨어

- Ubuntu 24.04
- NVIDIA CUDA 13과 호환되는 NVIDIA GPU 및 driver
- Python 3.12
- ROS 2 Jazzy
- 인터넷 연결: 최초 Python package 설치 시 필요
- 디스크 여유 공간: 프로젝트 `.venv` 기준 약 4.5 GB 이상

GPU와 driver를 확인한다.

```bash
nvidia-smi
```

Python 버전을 확인한다.

```bash
python3 --version
```

예상 결과:

```text
Python 3.12.x
```

ROS 2 Jazzy 설치 여부를 확인한다.

```bash
test -f /opt/ros/jazzy/setup.bash && echo "ROS2_JAZZY_OK"
```

### Python 가상환경에 설치되는 핵심 package

프로젝트는 `requirements/control.txt`로 다음 핵심 package를 설치한다.

- `cuopt-cu13==26.8.0`
- `cudf-cu13==26.8.0`: cuOpt 의존성으로 설치
- CUDA 13 Python runtime 및 수치 라이브러리
- `numpy`, `pandas`, `scipy`, `pyarrow`
- ROS 2 Python 호환용 `Jinja2`, `typeguard`, `setuptools`

직접 사용하는 package는 `requirements/control.txt`, 실제 설치된 전체 하위 의존성 버전은 `requirements/control-lock.txt`에서 확인한다.

ROS 2의 `rclpy`, `std_msgs`, `std_srvs`는 pip로 설치하지 않는다. `/opt/ros/jazzy`의 시스템 ROS 2 설치를 source하여 가상환경에서 사용한다.

## 2. 저장소 준비 방법

### 권장: 환경이 이미 구축된 프로젝트에서 pull

같은 PC에 프로젝트와 `.venv`가 이미 있다면 다시 clone하거나 cuOpt를 재설치하지 않는다.

```bash
cd /path/to/cobot3_project
git pull
```

`.venv`는 Git 관리 대상이 아니므로 `git pull` 후에도 기존 환경이 유지된다.

requirements가 바뀌었을 때만 다음 명령으로 환경을 갱신한다.

```bash
source .venv/bin/activate
python -m pip install -r requirements/control.txt
```

### 최초 실행 PC: clone 후 환경 구축

처음 사용하는 PC에는 `.venv`가 없으므로 저장소를 clone한 뒤 한 번만 환경을 구축한다.

```bash
git clone <repository-url>
cd cobot3_project

python3 -m venv .venv
source .venv/bin/activate

python -m pip install --upgrade pip wheel
python -m pip install -r requirements/control.txt
```

가상환경은 복사하거나 Git에 커밋하지 않는다. PC마다 requirements를 이용해 생성한다.

정확히 검증된 전체 버전을 설치해야 할 때는 다음을 사용한다.

```bash
python -m pip install -r requirements/control-lock.txt
```

일반적인 최초 설치에는 `control.txt` 사용을 권장한다.

## 3. 실행 전 환경 확인

프로젝트 루트에서 ROS 2와 프로젝트 가상환경을 순서대로 적용한다.

```bash
cd /path/to/cobot3_project

source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
hash -r
```

현재 사용 중인 Python을 확인한다.

```bash
which python3
```

예상 결과:

```text
/path/to/cobot3_project/.venv/bin/python3
```

필수 모듈을 한 번에 확인한다.

```bash
python3 -c "import rclpy, cudf; from std_msgs.msg import String; from std_srvs.srv import Trigger; from cuopt import routing; print('ENVIRONMENT_OK')"
```

예상 결과:

```text
ENVIRONMENT_OK
```

설치된 핵심 package 버전을 확인한다.

```bash
python -m pip show cuopt-cu13 cudf-cu13 numpy
```

전체 package 버전을 확인하려면 다음을 실행한다.

```bash
python -m pip freeze
```

의존성 충돌 여부를 확인한다.

```bash
python -m pip check
```

예상 결과:

```text
No broken requirements found.
```

## 4. 프로그램 실행 순서 및 방법

현재 프로그램은 두 개의 터미널을 사용한다.

### 터미널 1: Scenario 0 FMS Node 실행

```bash
cd /path/to/cobot3_project

source /opt/ros/jazzy/setup.bash
source .venv/bin/activate

cd tests/scenario0_fms
python3 scenario0_fms.py
```

정상 시작 로그:

```text
Loaded 10/10 demo tasks
Waiting for AMR route request on /amr/request_route
```

FMS는 시작 시 `defined.py`의 고정 Task 10개를 최대 길이 10의 작업큐에 적재하고 Service 요청을 기다린다.

### 터미널 2: Service 등록 확인

새 터미널에서 ROS 2 환경을 적용한다. Service를 호출하는 터미널에는 cuOpt 가상환경이 필요하지 않다.

```bash
source /opt/ros/jazzy/setup.bash
```

Service가 등록됐는지 확인한다.

```bash
ros2 service list | grep /amr/request_route
```

Service type을 확인한다.

```bash
ros2 service type /amr/request_route
```

예상 결과:

```text
std_srvs/srv/Trigger
```

### 터미널 2: AMR Route 요청을 수동으로 발생

```bash
ros2 service call /amr/request_route std_srvs/srv/Trigger "{}"
```

이 요청이 cuOpt Solve의 트리거가 된다.

정상 Service 응답:

```text
success=True
message='Optimized 10 tasks.'
```

### 터미널 1: cuOpt 결과 확인

FMS 터미널에 정렬된 Task 10개가 출력되는지 확인한다.

```text
Sending 10 tasks to cuOpt
=== Optimized Assembly Cell Order ===
01. task_05 -> cell_c (10.0, 4.0)
...
10. task_03 -> cell_c (10.0, 4.0)
Optimized route stored in FMS
```

Task 순서는 solver 조건이나 입력값 변경에 따라 달라질 수 있다. 다음 조건을 만족하면 정상이다.

- `ordered_tasks`가 10개다.
- `task_01`부터 `task_10`까지 누락과 중복이 없다.
- 각 Task에 `delivery_cell`과 좌표가 포함된다.
- Service 응답의 `success`가 `True`다.

FMS 종료는 터미널 1에서 `Ctrl+C`를 입력한다.

## 5. 현재 실행에서 colcon build가 필요한가?

현재 Scenario 0는 standard ROS 2 message와 service만 사용하고 Python 파일을 직접 실행하므로 `colcon build`가 필요하지 않다.

향후 `TaskRequest.msg`, `RequestRoute.srv`, `AMRState.msg` 같은 custom interface를 추가하면 ROS 2 interface package를 `colcon build`해야 한다.

## 6. 대표 오류 확인

### `ModuleNotFoundError: No module named 'cudf'`

기본 `/usr/bin/python3`로 실행한 경우 발생한다. 프로젝트 가상환경을 활성화하고 Python 경로를 확인한다.

```bash
cd /path/to/cobot3_project
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
hash -r
which python3
```

`which python3`가 프로젝트 `.venv/bin/python3`를 가리켜야 한다.

### `ModuleNotFoundError: No module named 'rclpy'`

가상환경만 활성화하고 ROS 2를 source하지 않은 경우 발생한다.

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
```

### `cudaErrorNoDevice`

GPU 또는 NVIDIA driver가 보이지 않을 때 발생한다.

```bash
nvidia-smi
```

GPU가 정상 표시되는지 확인한다. VM, container 또는 sandbox에서는 GPU device 전달 설정도 확인해야 한다.

### Service가 보이지 않음

- 터미널 1의 FMS Node가 실행 중인지 확인한다.
- 두 터미널에서 같은 `ROS_DOMAIN_ID`를 사용하는지 확인한다.
- 두 PC 통신이면 방화벽과 DDS multicast를 확인한다.

```bash
echo $ROS_DOMAIN_ID
```

필요하면 양쪽 터미널에 같은 값을 설정한다.

```bash
export ROS_DOMAIN_ID=129
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## 7. 관련 파일

- `../defined.py`: Task, AMR 상태, 최적화 요청·결과와 고정 데이터
- `../scenario0_fms.py`: ROS 2 FMS Node와 Task Queue, Service callback
- `../scenario0_cuopt_solver.py`: cuOpt Task 정렬 solver
- `../../../requirements/control.txt`: 핵심 Python 의존성
- `../../../requirements/control-lock.txt`: 검증된 전체 Python package 버전
- `FMS 시나리오 정리.md`: 시스템 역할과 Scenario 0 정의
- `작업내역.md`: 구현 및 검증 이력
