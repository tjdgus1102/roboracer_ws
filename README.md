# roboracer_ws

Nav2의 MPPI 컨트롤러를 f1tenth_gym 시뮬레이터와 실제 F1TENTH 차량에 붙이는 워크스페이스.

- **시뮬레이션** — 실차 트랙(`big_0723`)을 시뮬로 가져와 레이스라인 추종 + MPPI / RPP / DWB 제어기 비교
- **실차** — SLAM으로 맵을 만들고 AMCL 로컬라이제이션 위에서 자율주행

원 참고 프로젝트: https://github.com/lsw23101/CDSLST_roborace (README 7절 = 실차 이관)

---

## 이 저장소에 담긴 것 (우리가 만든 것)

```
src/f1tenth_mppi_nav/   MPPI 연동 패키지
  f1tenth_mppi_nav/       노드: path_follower, cmd_vel_to_ackermann, goal_pose_relay,
                          record_path(경로 기록), record_lap(랩 주행 기록), speed_profile(속도 프로파일 공급)
  config/                 nav2_params.yaml(시뮬 MPPI), nav2_params_real.yaml(실차 MPPI),
                          nav2_params_rpp.yaml / nav2_params_dwb.yaml(비교용 제어기), amcl.yaml
  launch/                 mppi_path_follow_launch.py(시뮬), mppi_real_launch.py(실차), localization_launch.py
raceline_opt/           TUM 최소곡률 레이스라인 생성 파이프라인 + 결과 big_0723_raceline.csv
tools/                  제어기 비교 자동 실행, 비교표 생성, 맵에 장애물 추가, 라이다 장착각 측정
maps/                   SLAM 으로 만든 실차 트랙 맵 (현재 트랙: big_0723, 장애물 버전: big_0723_obs)
paths/                  기록/추출한 주행 경로 CSV
*.md, simul_guide       실행 가이드 & 작업 기록 (아래 문서 참고)
check_pose.py           출발 전 위치/AMCL 상태 확인
check_path.py           기록한 경로가 주행 가능한지 곡률 검사
```

**외부 저장소(f1tenth_gym, f1tenth_racetracks, f1tenth_gym_ros)는 포함하지 않는다.**
남의 코드라 아래 「설치」대로 각자 클론해야 한다.
주행 로그(`*.log`), 랩 기록(`laps/`), 참고용 Nav2 소스(`reference/`)도 올리지 않는다.

---

## 문서 (읽는 순서)

| 문서 | 내용 |
|---|---|
| **`REAL_CAR_GUIDE.md`** | 실차 실행 가이드 + 작업 기록 + 겪은 함정 전부. **실차 하려면 이것부터** |
| **`RUNBOOK_SLAM_TO_DRIVE.md`** | 맵을 새로 만들 때 SLAM→자율주행 명령어 순서 (설명 최소) |
| `simul_guide` | 시뮬 실행, 제어기 비교 실행/결과, 실차 명령어 모음 |

> `REAL_CAR_GUIDE.md`의 「현재 블로커 — 위쪽 헤어핀 곡률」은 2026-07-18 기준 기록이다.
> 2026-07-23에 해결됐다 (아래 「현재 상태」 참고).

---

## 설치

### 1. 워크스페이스 클론

```bash
cd ~
git clone https://github.com/tjdgus1102/roboracer_ws.git roboracer_ws
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

### 3. 시뮬 맵 설정 (`sim.yaml` 직접 수정)

클론한 `src/f1tenth_gym_ros/config/sim.yaml`은 기본 맵(levine)을 가리킨다.
런치가 이 파일을 직접 읽어서 **맵은 CLI 인자로 바꿀 수 없다.** 아래 값으로 고친다.

```yaml
    map_path: '/home/<사용자>/roboracer_ws/maps/big_0723_obs'   # 장애물 없는 버전은 big_0723
    map_img_ext: '.pgm'
    sx: 8.3098
    sy: 1.2521
    stheta: 0.0386
    kb_teleop: False
```

`tools/run_comparison.sh`는 맵과 출발 포즈를 스스로 넘기므로 이 수정 없이도 돈다.

### 4. 의존성 & 빌드

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-ackermann-msgs ros-humble-slam-toolbox

cd ~/roboracer_ws
colcon build --base-paths src
```

레이스라인을 **다시 생성할 때만** `raceline_opt/`의 TUM 옵티마이저 의존성
(`trajectory_planning_helpers==0.76`, `quadprog`, `casadi`, `scikit-learn`)이 필요하다.
결과 CSV는 이미 들어 있으니 주행만 할 거면 필요 없다.

### 5. 실차 드라이버 (실차만)

실차는 별도 워크스페이스 `~/f1tenth_ws`의 `f1tenth_stack`(라이다 + VESC)을 쓴다.
설치는 https://github.com/f1tenth/f1tenth_system 참고. 자세한 실행은 `REAL_CAR_GUIDE.md`.

---

## 빠른 실행

### 시뮬레이션 (터미널 2개)

```bash
# 터미널 1
cd ~/roboracer_ws && source env.sh
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# 터미널 2 — 기본 경로는 raceline_opt/big_0723_raceline.csv
cd ~/roboracer_ws && source env.sh
ros2 launch f1tenth_mppi_nav mppi_path_follow_launch.py
```

다른 제어기로 바꾸려면 `params_file:=` 로 설정 파일만 교체한다 (맵·경로·costmap은 동일).

```bash
ros2 launch f1tenth_mppi_nav mppi_path_follow_launch.py \
  params_file:=$HOME/roboracer_ws/src/f1tenth_mppi_nav/config/nav2_params_dwb.yaml   # 또는 _rpp.yaml
```

### 제어기 비교 (MPPI / RPP / DWB 자동 주행)

```bash
cd ~/roboracer_ws
tools/run_comparison.sh obs 3 120    # [clean|obs] [반복횟수] [1회주행초]
python3 tools/compare_laps.py obs    # 비교표
```

결과는 `laps/compare/<시나리오>/<제어기>/run<N>/`에 쌓인다. 비교 조건과 DWB의 구조적 한계는 `simul_guide` 참고.

### 실차

`REAL_CAR_GUIDE.md`(처음) 또는 `RUNBOOK_SLAM_TO_DRIVE.md`(맵 새로 만들 때) 참고.

```bash
ros2 launch f1tenth_mppi_nav mppi_real_launch.py   # 기본 경로: raceline_opt/big_0723_raceline.csv
```

---

## 현재 상태 (2026-09-14)

- **실차: 트랙 완주 성공 (2026-07-23).** 헤어핀 정지의 원인은 조향 한계 과소평가였다.
  공식값(최대 조향 ±24°, 축거 0.307 m, 최소 회전반경 0.69 m)으로 다시 맞춰 해결했다.
  - 남은 검증: 서보 gain(−0.7630)이 가정으로 계산한 값이라 스탠드에서 실제 바퀴각 측정이 필요하다.
- **현재 트랙은 `big_0723`.** AMCL과 실차/시뮬 런치 기본 경로 모두 이 트랙을 가리킨다.
- **레이스라인:** `raceline_opt/`에서 TUM 최소곡률 라인을 생성했다 (최소 곡률반경 0.79 m ≥ 차량 0.69 m).
  `speed_profile` 노드가 오프라인 속도 프로파일을 MPPI에 공급한다.
- **진행 중 — 제어기 비교:** 장애물 맵(`big_0723_obs`)에서 MPPI vs RPP vs DWB.
  목표는 랩타임이 아니라 **장애물 회피 성능** 비교다.
  - RPP는 장애물 앞에서 피하지 않고 정지한다.
  - DWB는 Ackermann 제약을 표현하지 못하는 한계가 있다.
  - 장애물 최소 이격거리 지표는 아직 추가하지 않았다.
