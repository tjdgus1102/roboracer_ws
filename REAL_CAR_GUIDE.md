# F1TENTH 실차 자율주행 — 실행 가이드 & 작업 기록

작성 2026-07-18. 참고 문서: https://github.com/lsw23101/CDSLST_roborace README **7절**

이 문서는 두 부분이다.

- **1부** — 지금 상태에서 주행을 재현하는 방법 (따라 하기)
- **2부** — 오늘 어떤 과정을 거쳤는지, 왜 그렇게 됐는지 (배경)

바로 돌려보려면 1부만 보면 된다.

---

# 1부. 주행 재현하기

## 0. 준비물 확인

| 장치 | 연결 방식 | 확인 명령 |
|---|---|---|
| VESC (모터) | USB → `/dev/sensors/vesc` | `ls -l /dev/sensors/vesc` |
| 라이다 | **이더넷** `192.168.0.10:10940` | `ping 192.168.0.10` |
| 조이패드 | DS4 블루투스 → `/dev/input/js0` | `ls /dev/input/js0` |

- VESC는 udev 규칙(`/etc/udev/rules.d/99-vesc.rules`)이 `MODE="0666"`을 주므로 **`dialout` 그룹 불필요**.
- 라이다는 시리얼이 아니라 **이더넷**이다. Jetson 쪽 인터페이스는 `enP8p1s0` = `192.168.0.15/24`.
- **조이패드는 필수다.** SLAM 주행에도 쓰고, ackermann_mux에서 joystick priority 100 / navigation 10이라 **킬스위치 역할**을 한다.

> ⚠️ **워크스페이스가 두 개다. 헷갈리지 말 것.**
> - 하드웨어 드라이버 = `~/f1tenth_ws`
> - MPPI / 경로 / 로컬라이제이션 = `~/roboracer_ws`

---

## 1. 터미널 1 — 실차 브링업

```bash
cd ~/f1tenth_ws
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

**라이다에 전원을 넣고 몇 초 기다렸다가** 실행할 것. 부팅 전에 urg_node가 붙으려 하면 실패한다.

### 실행 직후 반드시 — 조이패드로 조향을 좌우 한 번 움직인다

`/odom`과 `odom → base_link` tf는 **서보 명령을 한 번 받아야 생긴다.**
안 하면 다음 단계에서 `Invalid frame ID "odom"`이 뜬다. 고장이 아니다. → 2부 함정①

### 확인

```bash
ros2 topic hz /scan     # 40 Hz
ros2 topic hz /odom     # 50 Hz
ros2 run tf2_ros tf2_echo odom base_link
```

---

## 2. 터미널 2 — 로컬라이제이션 (AMCL)

```bash
cd ~/roboracer_ws
source env.sh
ros2 launch f1tenth_mppi_nav localization_launch.py
```

맵은 `config/amcl.yaml`의 `yaml_filename`이 가리킨다. 현재 `maps/track_real.yaml`.

> 처음 받은 워크스페이스라 `localization_launch.py`가 없다고 나오면 빌드가 안 된 것이다:
> ```bash
> cd ~/roboracer_ws && colcon build --base-paths src --packages-select f1tenth_mppi_nav
> ```

---

## 3. 터미널 3 — RViz

```bash
rviz2
```

설정:

1. **Fixed Frame** → `map`
   - `frame [map] does not exist` 빨간 경고가 뜨면 **조이스틱으로 차를 조금 움직이면 사라진다.**
     AMCL은 odom 정보를 받아야 `map→odom` tf를 발행하는데, 그 `/odom` 자체가 서보 명령을 받아야
     생기기 때문이다(함정①). 즉 1번 단계의 조향 한 번을 빠뜨렸거나, 그 뒤로 차가 전혀 안 움직인 경우다.
2. **Add → By topic → `/map` → Map**
   - **`No map received`가 뜨면 QoS 문제다.** Map 디스플레이의 `Topic`을 펼쳐서
     `Durability Policy` → **`Transient Local`**, `Reliability Policy` → **`Reliable`** 로 바꾼다. → 2부 함정③
3. **Add → By topic → `/particle_cloud` → PoseArray** (AMCL 수렴 확인용)
4. **Add → By topic → `/scan` → LaserScan**

---




## 4. 차량 배치 & AMCL 수렴

**이 단계가 오늘 가장 시간을 많이 잡아먹었다. 순서를 지킬 것.**

1. 차를 **트랙 맨 아래 곡선부(헤어핀)** 에 놓는다. 목표 지점:

   ```
   x = 0.763   y = -3.530   머리 방향 = 174.4도 (거의 정서쪽 = 맵 그림에서 왼쪽)
   ```

   근처 어디든 괜찮다. 참고 그림: `target.png`, `pos_earlier.png`
   (맵 프레임의 **+X는 그림에서 오른쪽, +Y는 위쪽**)

2. RViz에서 **`2D Pose Estimate`** — 위치를 클릭하고 **차 머리 방향으로 드래그**해서 놓는다.
   방향까지 맞추는 게 중요하다. 방향이 반대면 스캔이 벽에 안 붙는다.

3. 조이패드로 **경로를 따라 천천히 3~4 m 주행**한다.

4. 아래 기준을 확인한다.

### 출발 판정 기준

| 항목 | 기준 | 비고 |
|---|---|---|
| 경로까지 거리 | ≤ 0.30 m | |
| 경로 방향과 차이 | ≤ 20° | **가장 중요.** 반대를 보고 있으면 절대 안 됨 |
| AMCL 1σ x, y | ≤ 0.10 m | y는 0.11~0.12까지 허용 가능 (아래 참고) |
| AMCL 1σ yaw | ≤ 6° | |

확인용 스크립트: `check_pose.py` (아래 5번 참고)

> **왜 직선 구간에서는 수렴이 안 되나**
> 이 트랙은 긴 직선 회랑이 대부분이다. 회랑을 따라가는 방향(세로 구간에서는 y)은
> 라이다 스캔 모양이 어디서나 같아서 **위치를 관측할 수 없다.** 그래서 그 구간에서는
> y 오차가 아무리 주행해도 0.10 밑으로 안 떨어진다.
> **헤어핀 곡선부로 가면** 벽이 여러 방향으로 꺾여 있어 몇 초 만에 수렴한다.
>
> 손으로 차를 옮기면 휠 오도메트리에 변화가 없어 AMCL이 못 따라온다.
> **손으로 옮겼으면 반드시 `2D Pose Estimate`를 다시 찍을 것.**

---

## 5. 상태 확인 스크립트

```bash
cd ~/roboracer_ws
python3 check_pose.py
```

현재 위치, 경로상 최근접점, 방향 차이, AMCL 1σ를 한 번에 출력하고 출발 가능 여부를 판정한다.

---

## 6. 터미널 4 — MPPI 자율주행

```bash
cd ~/roboracer_ws
source env.sh
ros2 launch f1tenth_mppi_nav mppi_real_launch.py 2>&1 | tee ~/roboracer_ws/mppi_run.log
```

- 경로 기본값은 `paths/track_real_lap3.csv` (오늘 기록한 것 중 최선)
- `tee`로 로그를 남기면 나중에 원인 분석이 가능하다. **터미널을 잃어버려도 파일이 남는다.**

### 🔴 조이패드를 손에 들고 실행할 것

ackermann_mux에서 joystick priority가 100이라 스틱을 건드리는 즉시 제어를 가져온다.
MPPI를 끄지 않아도 개입할 수 있다.

### 정상 로그

```
Sending FollowPath goal with 63 waypoints
Received a goal, begin computing control effort.
```

### 파라미터를 수정한 뒤 재실행하기

`nav2_params_real.yaml`을 고쳤을 때 **터미널 4만 다시 띄우면 된다.**
터미널 1(브링업) · 2(AMCL) · 3(RViz)는 건드리지 말 것. **AMCL을 죽이면 수렴을 처음부터 다시 잡아야 한다.**

```bash
# 터미널 4에서 Ctrl+C 후
cd ~/roboracer_ws
colcon build --packages-select f1tenth_mppi_nav      
source env.sh
ros2 launch f1tenth_mppi_nav mppi_real_launch.py 2>&1 | tee ~/roboracer_ws/mppi_run.log
```

> ⚠️ **`colcon build`가 필수다.** 이 워크스페이스는 `--symlink-install` 없이 빌드돼 있어서
> `install/.../config/nav2_params_real.yaml`이 src의 **복사본**이다. src만 고치면 아무 일도 안 일어난다.
>
> 빌드를 건너뛰려면 src 경로를 직접 넘긴다:
> ```bash
> ros2 launch f1tenth_mppi_nav mppi_real_launch.py \
>   params_file:=$HOME/roboracer_ws/src/f1tenth_mppi_nav/config/nav2_params_real.yaml
> ```

또한 이 런치는 `planner_server`를 띄우지 않으므로 yaml의 **`global_costmap` 섹션은 효과가 없다.**
`local_costmap`은 `controller_server` 내부에서 돌기 때문에 터미널 4 재실행만으로 같이 반영된다.

### 재실행 없이 주행 중에 바꾸기 (튜닝할 때 권장)

nav2 1.1.20의 MPPI는 파라미터 대부분이 **dynamic**이고, 값이 바뀌면
`optimizer.cpp`의 post-callback이 `reset()`을 불러 내부 버퍼까지 다시 잡는다.
즉 주행 중에 그냥 바꿔도 먹는다.

```bash
ros2 param set /controller_server FollowPath.vx_max 0.5
ros2 param set /controller_server FollowPath.PathAlignCritic.cost_weight 6.0
ros2 param set /controller_server FollowPath.AckermannConstraints.min_turning_r 0.93
ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 0.3
```

**예외 2개는 런타임 변경이 안 된다** → yaml 수정 + 위의 재실행이 필요하다.

| 항목 | 이유 |
|---|---|
| `controller_frequency` | `ParameterType::Static`으로 선언됨 |
| `critics` 리스트 | critic 추가/제거는 플러그인 재로딩이 필요 |

값이 잡히면 **yaml에 옮겨 적고 빌드해서 영구 반영할 것.** `param set`은 재실행하면 사라진다.

### 현재 알려진 결과

트랙의 **약 60%(경로 idx 38/64)까지 자율주행에 성공**한다.
그 지점 = **위쪽 헤어핀**에서 속도가 0.1 m/s 수준으로 떨어지며 사실상 멈춘다.
이건 버그가 아니라 **곡률 한계**다. → 2부 「현재 블로커」

---

# 2부. 오늘 작업 기록

## 오늘의 결과 요약

| README 7절 | 상태 |
|---|---|
| 7.1 실차 드라이버 브링업 | ✅ 완료 (`~/f1tenth_ws`에 이미 빌드돼 있었음) |
| 7.2 SLAM 맵 작성 | ✅ 완료 → `maps/track_real.*` |
| 7.3 로컬라이제이션 | ✅ 완료 (AMCL 수렴 확인) |
| 7.4 전역 경로 생성 | ✅ 완료 → `paths/track_real_lap3.csv` |
| 7.5 파라미터 재조정 | ✅ 완료 + **모터 부호 버그 발견·수정** |
| 첫 자율주행 | ⚠️ **트랙 60% 주행 성공, 위쪽 헤어핀에서 정지** |

---

## 만들어진 / 수정된 파일

### 새로 만든 것

| 파일 | 설명 |
|---|---|
| `maps/track_real.pgm` / `.yaml` | slam_toolbox로 만든 실제 트랙 맵. 99×144 px, 0.05 m/px (≈4.95×7.20 m), origin `[-0.817, -3.980]` |
| `paths/track_real_lap3.csv` | **실주행에 쓰는 경로.** 64점, 13.7 m |
| `paths/track_real_lap*.csv` | 다른 기록 시도들 (아래 비교표) |
| `paths/track_real_center.csv` | `extract_centerline.py`로 뽑은 중심선 |
| `src/f1tenth_mppi_nav/config/nav2_params_real.yaml` | 실차용 Nav2/MPPI 설정 (시뮬용 `nav2_params.yaml`은 그대로 둠) |
| `src/f1tenth_mppi_nav/launch/mppi_real_launch.py` | 실차용 런치 |
| `check_pose.py` | 출발 조건 확인 스크립트 |
| `target.png`, `pos_earlier.png` | 차량 배치 안내 그림 |

### 수정한 것

| 파일 | 변경 | 이유 |
|---|---|---|
| `config/amcl.yaml` | `yaml_filename` → `maps/track_real.yaml` | 예전 시뮬 맵(`/home/swlee/obstacle.yaml`)을 가리키고 있었음 |
| `f1tenth_mppi_nav/cmd_vel_to_ackermann.py` | `out.drive.speed = -speed` | **모터 부호 버그.** 아래 함정④ |
| `simul_guide` | 실차 섹션 추가 | 명령어 모음 |

> `~/f1tenth_ws`는 **건드리지 않았다.** `vesc.yaml`, `joy_teleop.yaml` 원본 그대로다.

---

## 실차 제원 (실측 계산값)

`~/f1tenth_ws/.../f1tenth_stack/config/vesc.yaml` 로부터 계산:

```
wheelbase                  0.25 m
steering_angle_to_servo_gain  -1.2135,  offset 0.5304
servo_min 0.15 → 조향  +0.3135 rad (18.0°)
servo_max 0.85 → 조향  -0.2634 rad (15.1°)

대칭으로 쓸 수 있는 조향 한계 = 15.1°
=> 최소 회전반경 = 0.25 / tan(0.2634) = 0.927 m
```

> ⚠️ **시뮬레이션의 0.742 m를 실차에 쓰면 안 된다.**
> 그 값은 `cmd_vel_to_ackermann.py`의 시뮬용 기본값(wheelbase 0.3302, max_steer 0.4189)에서 나온 것이다.
> 실차는 **0.927 m**로 더 불리하다. 조향이 비대칭이라 한쪽 코너가 더 어렵다.

---

## MPPI 설정값 (`nav2_params_real.yaml`)

| 항목 | 시뮬 | 실차 | 근거 |
|---|---|---|---|
| `robot_base_frame` | `ego_racecar/base_link` | `base_link` | 실차 tf |
| `odom_topic` | `/ego_racecar/odom` | `/odom` | 실차 토픽 |
| `vx_max` | 6.0 | **1.0** | 첫 주행, 벽 여유가 좁음 |
| `min_turning_r` | 0.75 | **0.93** | 실측 한계 (처음엔 1.00으로 뒀다가 낮춤) |
| `wz_max` | 1.8 | **1.0** | `vx_max ÷ min_turning_r` |
| `time_steps` | 56 | **40** | Jetson 연산 부담 |
| `batch_size` | 2000 | **1000** | 위와 동일 |
| `vx_std` | 0.6 | **0.2** | `vx_max`에 비례. 안 줄이면 샘플 절반이 상한에 잘림 |
| `wz_std` | 0.6 | **0.4** | 위와 동일 |
| `prune_distance` | 6.0 | **2.0** | 예측 구간 2 m에 맞춤 |

`cmd_vel_to_ackermann` (런치에서 주입):
```
wheelbase           0.3302 → 0.25
max_steering_angle  0.4189 → 0.2634
```

---

## 겪은 함정 (팀원들이 똑같이 만날 것들)

### ① `/odom`이 안 나온다 — 정상이다

브링업 직후 `/odom`이 비어 있고 tf가 `Invalid frame ID "odom"`을 뱉는다.

`vesc.yaml`의 `use_servo_cmd_to_calc_angular_velocity: true` 때문에
`vesc_to_odom_node`가 **`/sensors/servo_position_command`를 한 번이라도 받아야** odom 계산을 시작한다.

**해결: 조이패드로 조향을 한 번 움직인다.**
VESC 생존 여부는 `/sensors/core`로 따로 확인할 수 있다(전압 등이 나옴).

### ② 라이다 `Not Connected` — 포트를 떠보지 말 것

증상: ping은 되는데 `/diagnostics`의 `urg_node: Hardware Status`가 `Not Connected`,
`ss -tnp | grep 10940`이 `SYN-SENT`.

Hokuyo는 **TCP 세션을 하나만 허용한다.** 연결 확인한다고 포트를 직접 열면
(`bash -c "</dev/tcp/192.168.0.10/10940"`) 그 세션이 남아 urg_node가 못 붙는다.

**해결: 라이다 전원 재투입 후 브링업 재실행.**
연결 확인은 `/diagnostics`와 `ros2 topic hz /scan`으로만 한다.

### ③ RViz `No map received` — QoS 문제

`map_server`는 맵을 **Transient Local(래치)** 로 한 번만 발행한다.
RViz Map 디스플레이가 기본 `Volatile`로 구독하면 영원히 못 받는다.

**해결: Map 디스플레이 → Topic → Durability Policy = `Transient Local`.**

같은 이유로 `ros2 topic echo /map`도 아무것도 안 나온다. `ros2 topic hz /map`을 쓰거나
QoS를 맞춘 구독자를 따로 만들어야 한다.

### ④ 🔴 모터 부호가 반대다 — 오늘 가장 큰 버그

**증상:** MPPI를 실행하면 차가 후진하면서 중앙 벽으로 돌진.

**측정 결과:**

```
조이패드 전진 시   /teleop speed = -0.250  (음수!)
                  /odom  실제속도 = +0.398  (양수)

MPPI 주행 시      /cmd_vel linear.x = +0.270  (양수, 정상)
                  /drive   speed    = +0.270  (그대로 전달)
                  → 실제로는 후진
```

이 차는 **모터 배선이 VESC 규약과 반대**로 되어 있다.
그래서 `/drive`에 **음수를 보내야 전진**한다. 조이패드는 `joy_teleop.yaml`의
`scale: -0.25`로 이미 그 보정이 들어가 있었고, MPPI 경로만 보정이 없었다.

**❌ `vesc.yaml`의 `speed_to_erpm_gain` 부호를 뒤집으면 안 된다.**
`ackermann_to_vesc`(명령)와 `vesc_to_odom`(피드백)이 **같은 파라미터를 공유**한다.

```cpp
ackermann_to_vesc.cpp:71   erpm  = gain * speed
vesc_to_odom.cpp:102       speed = (-state.speed) / gain
```

부호를 뒤집으면 명령은 고쳐지지만 **`/odom`이 전진을 음수로 보고**하게 되어
AMCL과 MPPI가 전부 틀어진다. 지금 `/odom`은 (배선 반전 덕분에 우연히) **올바르다.**

**✅ 적용한 해결:** `cmd_vel_to_ackermann.py`에서 **명령 경로만** 뒤집었다.

```python
out.drive.speed = -speed
```

`steering_angle` 계산에는 원래 부호의 `speed`를 쓴다(자전거 모델은 실제 전진 속도 기준).

**근본 해결은 VESC 펌웨어에서 모터 방향을 반전시키거나 3상 배선 두 가닥을 바꾸는 것.**
그렇게 하면 `joy_teleop.yaml`의 `scale`을 `+0.25`로 되돌리고 위 한 줄도 원복해야 한다.

### ⑤ `record_path`가 조용히 죽는다

두 가지 문제가 있다.

1. **기본 `base_frame`이 시뮬 값(`ego_racecar/base_link`)이다.** 그냥 실행하면 tf를 못 찾아
   웨이포인트 0개로 조용히 끝난다. **반드시 `-p base_frame:=base_link`를 넘길 것.**
2. **`ConnectivityException`을 안 잡는다.** AMCL이 `map→odom`을 아직 안 올린 상태에서
   실행하면 "map과 base_link 연결 불가"로 **노드가 강제 종료**된다.
   (`paths/track_real_lap5.csv`가 그때 만들어진 **빈 파일**이다.)

   `record_path.py`는 `LookupException`, `ExtrapolationException`만 잡고 있다.
   고치려면 `ConnectivityException`을 추가하면 된다. **아직 안 고쳤다.**

### ⑥ 죽인 노드가 `ros2 node list`에 계속 보인다

ROS2 데몬이 캐시한 것이다. 프로세스는 이미 없다.

```bash
ros2 daemon stop     # 재시작하면 정리됨
```

반대로 데몬 재시작 직후에는 탐색이 덜 되어 **살아 있는 노드가 안 보일 수 있다.** 몇 초 기다렸다 다시 조회할 것.

---

## 경로 기록 시도 기록

`record_path`로 사람이 직접 몰아 기록했다. **최소 곡률반경이 0.927 m 이상**이어야 MPPI가 돈다.

```bash
ros2 run f1tenth_mppi_nav record_path --ros-args \
  -p base_frame:=base_link \
  -p min_distance:=0.2 \
  -p output_file:=/home/swlee/roboracer_ws/paths/track_real_lapN.csv
```

한 바퀴 돌고 `Ctrl+C`로 저장.

| 파일 | 점 | 최소 R | R<0.93 위반 | 시작-끝 | 비고 |
|---|---|---|---|---|---|
| `track_real_lap.csv` | 108 | 0.053 m | 25/106 | 0.407 m | 위치 튐 노이즈 다수 |
| `track_real_lap2.csv` | 104 | 0.454 m | 20/102 | 0.812 m | |
| **`track_real_lap3.csv`** | **64** | **0.412 m** | **6/62** | **0.421 m** | **최선. 현재 사용 중** |
| `track_real_lap4.csv` | 61 | 0.574 m | 7/59 | 0.217 m | 시작-끝이 너무 가까움 ⚠️ |
| `track_real_lap5.csv` | 0 | — | — | — | 빈 파일 (함정⑤) |
| `track_real_center.csv` | 57 | 0.545 m | 15/55 | 0.142 m | 스켈레톤 추출. 벽 여유는 최고(0.412 m) |

### 기록 요령

- **`min_distance:=0.2`를 넣을 것.** 기본값 0.1이면 멈칫할 때 점이 뭉쳐 지그재그 노이즈가 생긴다.
- **헤어핀에서 중앙 벽에 붙지 말 것.** 벽에서 멀어질수록 반경이 커진다 (`R ≈ 0.2 + 벽까지 거리`).
- 일정한 속도로. 멈추면 그 자리에 점이 뭉친다.
- **닫힌 루프 함정:** 시작점과 끝점이 `xy_goal_tolerance`(0.3 m)보다 가까우면
  출발 즉시 "도착" 판정이 난다. 끝을 조금 못 미쳐서 마치거나, CSV 끝 몇 점을 잘라낼 것.
  `lap4`의 0.217 m가 이 경우다.

### 중심선 자동 추출도 시도했다

```bash
python3 src/f1tenth_mppi_nav/scripts/extract_centerline.py maps/track_real.yaml paths/track_real_center.csv
```

결과: **벽 여유는 훨씬 좋지만(0.412 m vs 0.158 m) 곡률은 더 나쁘다(위반 15개 vs 6개).**
중심선은 헤어핀에서 중앙 벽 끝을 바짝 감아 돌기 때문이다.

**이 트랙의 근본 딜레마 = 벽을 피하는 것과 곡률을 확보하는 것이 서로 반대 방향이다.**

---

## 🚧 현재 블로커 — 위쪽 헤어핀 곡률

MPPI는 경로 **idx 38/64**에서 사실상 정지한다. 그 앞의 곡률:

```
idx 36   R = 2.791 m
idx 37   R = 1.661 m
idx 38   R = 0.793 m   위반   <<< 여기서 멈춤
idx 41   R = 0.412 m   위반
idx 42   R = 0.798 m   위반
```

멈춘 시점의 실측:

```
/cmd_vel  linear.x  최대 +0.106 m/s   (설정 vx_max 1.0의 1/10)
          angular.z -0.024 ~ +0.004   (거의 0)
경로까지 0.100 m,  방향 차이 +2.6도   ← 경로 추종 자체는 완벽했다
```

노드는 살아 있고 명령도 계속 나온다. 다만 **제약을 만족하는 궤적 중 최선이 거의 정지**인 상태다.

### 시도했지만 안 된 것

- `min_turning_r` 1.00 → 0.93 (실측 한계까지 완화) — **부족했다**
- 문제 지점 국소 스무딩 — 최선 R 0.625 m, 목표 미달
- 여유거리 기반으로 코너를 바깥으로 밀기 — 위반이 2개 → 7개로 **악화** (풍선효과)
- 중심선 자동 추출 — 위반 15개로 **더 나쁨**
- 사람이 직접 4번 재주행 — 위/아래 헤어핀을 **동시에** 잘 도는 라인이 안 나옴

### 왜 어려운가

```
회랑 폭            1.15 ~ 1.30 m
차량 최소 회전반경   0.927 m
문제 지점 벽 여유    0.212 m  (차 반폭 0.15 m를 빼면 6 cm)
```

기하학적으로 가능은 하지만(`R = 0.2 + 벽까지 거리` → 벽에서 0.73 m 떨어지면 R=0.93)
사람이 손으로 그 라인을 그리기가 매우 어렵다. 실제로 시도 중 **벽에 한 번 닿았다.**

### 다음에 해볼 것 (권장 순서)

1. **중앙 벽을 물리적으로 옮기거나 얇게 만든다** ← **가장 확실**
   시뮬레이션에서도 실차에서도 같은 지점에서 같은 이유로 막혔다.
   벽을 5~10 cm만 옮겨도 헤어핀 반경이 크게 늘어난다.
   (참고: 시뮬에서 벽을 0.5 m 잘라낸 `obstacle_short`를 만들었더니 끝이 뭉툭해져
   곡률이 **오히려 나빠졌다**. 자르지 말고 **옮기거나 얇게** 할 것.)
2. 위쪽 헤어핀 구간만 다시 기록해서 이어붙이기
3. `vx_max`를 더 낮춰(0.5 등) 재시도 — 저속이면 MPPI가 더 조밀하게 탐색할 여지가 있다

---

## 참고 — 시뮬레이션

시뮬레이션 실행 방법과 맵 교체 방법은 `simul_guide` 파일에 있다.
실차 명령어 모음도 그 파일 아래쪽에 정리돼 있다.

시뮬에서는 Silverstone 트랙 완주(약 198초)에 성공했다.
`obstacle_clean` / `obstacle_short` 맵은 **지금 실차와 똑같은 헤어핀 문제**로 완주하지 못한다.
