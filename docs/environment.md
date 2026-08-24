# 개발 환경

## Control / FMS 환경

- OS: Ubuntu 24.04
- Python: 3.12
- ROS 2: Jazzy
- cuOpt: 26.8.0 (CUDA 13)
- RMW: Fast DDS (`rmw_fastrtps_cpp`)
- ROS_DOMAIN_ID 허용 범위: 129~135
- 기본 ROS_DOMAIN_ID: 129

### 최초 환경 구축

저장소 루트에서 실행한다.

```bash
cd ~/cobot3_project

python3 -m venv .venv
source .venv/bin/activate

python -m pip install --upgrade pip wheel
python -m pip install -r requirements/control.txt
```

`empy`는 ROS 2가 `.msg`와 `.srv` 정의로부터 언어별 인터페이스 코드를
생성할 때 사용하는 템플릿 엔진이다. Python에서는 `em`이라는 이름으로
import한다. `lark`는 인터페이스 문법 해석에 사용된다.

### 가상환경 기반 ROS 2 빌드

시스템 ROS 환경을 불러온 뒤 가상환경의 Python으로 `colcon`을 실행한다.

```bash
cd ~/cobot3_project/ros2_ws

source /opt/ros/jazzy/setup.bash
source ../.venv/bin/activate

python -m colcon build
source install/setup.bash
```

가상환경 기반 빌드가 완료되면 FMS launcher의 shebang도 `.venv/bin/python`을
가리키므로 다음 명령에서 cuDF와 cuOpt를 사용할 수 있다.

```bash
ros2 run fms scenario0_fms
```
