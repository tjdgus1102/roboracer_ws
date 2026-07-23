# roboracer_ws

Nav2의 MPPI 컨트롤러를 f1tenth_gym 시뮬레이터와 실제 F1TENTH 차량에 붙이는 워크스페이스.

- **시뮬레이션** — Silverstone 등 트랙에서 MPPI 경로 추종
- **실차** — SLAM으로 맵을 만들고 AMCL 로컬라이제이션 위에서 자율주행

원 참고 프로젝트: https://github.com/lsw23101/CDSLST_roborace (README 7절 = 실차 이관)

---

## 이 저장소에 담긴 것 (우리가 만든 것)

```
src/f1tenth_mppi_nav/   MPPI 연동 패키지 (경로 추종, cmd_vel→ackermann, 경로 기록/추출)
maps/                   SLAM 으로 만든 실차 트랙 맵
paths/                  기록/추출한 주행 경로 CSV
*.md, simul_guide       실행 가이드 & 작업 기록 (아래 문서 참고)
check_pose.py           출발 전 위치/AMCL 상태 확인
check_path.py           기록한 경로가 주행 가능한지 곡률 검사
```

**외부 저장소(f1tenth_gym, f1tenth_racetracks, f1tenth_gym_ros)는 포함하지 않는다.**
남의 코드라 아래 「설치」대로 각자 클론해야 한다.

---

## 문서 (읽는 순서)

| 문서 | 내용 |
|---|---|
| **`REAL_CAR_GUIDE.md`** | 실차 실행 가이드 + 오늘 작업 기록 + 겪은 함정 전부. **실차 하려면 이것부터** |
| **`RUNBOOK_SLAM_TO_DRIVE.md`** | 맵을 새로 만들 때 SLAM→자율주행 명령어 순서 (설명 최소) |
| `simul_guide` | 시뮬레이션 실행/맵 교체 + 실차 명령어 모음 |

---

## 설치

### 1. 워크스페이스 클론

```bash
cd ~
git clone <이 저장소 URL> roboracer_ws
cd roboracer_ws
```

### 2. 외부 저장소 클론 (이 저장소에 없는 것들)

```bash
# 시뮬레이터 ROS2 브릿지
git clone https://github.com/f1tenth/f1tenth_gym_ros.git src/f1tenth_gym_ros

# 시뮬레이터 물리엔진
git clone https://github.com/f1tenth/f1tenth_gym.git
cd f1tenth_gym && pip install -e . && cd ..

# (선택) 공식 트랙 맵/센터라인 데이터
git clone https://github.com/f1tenth/f1tenth_racetracks.git
```

> `env.sh`가 `PYTHONPATH`로 `~/roboracer_ws/f1tenth_gym/gym`을 우선시켜, 전역 pip에 깔린
> 다른 f110_gym fork와 격리한다. 시뮬을 돌리는 터미널에서는 `source env.sh` 를 해야 한다.

### 3. 의존성 & 빌드

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-ackermann-msgs ros-humble-slam-toolbox

cd ~/roboracer_ws
colcon build --base-paths src
```

### 4. 실차 드라이버 (실차만)

실차는 별도 워크스페이스 `~/f1tenth_ws`의 `f1tenth_stack`(라이다 + VESC)을 쓴다.
설치는 https://github.com/f1tenth/f1tenth_system 참고. 자세한 실행은 `REAL_CAR_GUIDE.md`.

---

## 빠른 실행

### 시뮬레이션 (터미널 2개)

```bash
# 터미널 1
cd ~/roboracer_ws && source env.sh
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# 터미널 2
cd ~/roboracer_ws && source env.sh
ros2 launch f1tenth_mppi_nav mppi_path_follow_launch.py
```

맵 교체 방법은 `simul_guide` 참고.

### 실차

`REAL_CAR_GUIDE.md`(처음) 또는 `RUNBOOK_SLAM_TO_DRIVE.md`(맵 새로 만들 때) 참고.

---

## 현재 상태

- 시뮬: Silverstone 완주 성공 (약 198초)
- 실차: SLAM→로컬라이제이션→경로기록→자율주행까지 동작. **트랙 약 60%까지 주행, 위쪽 헤어핀에서 정지.**
  원인 분석과 다음 단계는 `REAL_CAR_GUIDE.md` 「현재 블로커」 참고.
