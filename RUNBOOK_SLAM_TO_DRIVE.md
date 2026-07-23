# 맵 새로 만들어서 자율주행까지 — 명령어 순서

맵을 수정했을 때 처음부터 다시 하는 순서.
`NEW` = 새 맵 이름 (예: `track_real2`)

---

## 1. 브링업

**터미널 1**
```bash
cd ~/f1tenth_ws
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

→ **조이패드로 조향 좌우 한 번** (안 하면 `/odom` 안 생김)

확인:
```bash
ros2 topic hz /scan     # 40 Hz
ros2 topic hz /odom     # 50 Hz
```

---

## 2. SLAM

**터미널 2**
```bash
cd ~/f1tenth_ws
source install/setup.bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/swlee/f1tenth_ws/install/f1tenth_stack/share/f1tenth_stack/config/f1tenth_online_async.yaml
```

주행 전 확인 (**`base_link` 나와야 함**):
```bash
ros2 param get /slam_toolbox base_frame
```

**터미널 3**
```bash
rviz2
```
- Fixed Frame → `map`
- Add → By topic → `/map` → Map → **Topic 펼쳐서 Durability = `Transient Local`**
- Add → By topic → `/scan` → LaserScan

→ 조이패드로 **천천히** 한 바퀴. 출발점으로 정확히 복귀.

---

## 3. 맵 저장

**터미널 4**
```bash
cd ~/roboracer_ws
source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/roboracer_ws/maps/NEW
```

→ **터미널 2의 slam_toolbox 종료 (`Ctrl+C`)**

---

## 4. 맵 경로 수정

`~/roboracer_ws/src/f1tenth_mppi_nav/config/amcl.yaml` 50번째 줄:
```yaml
    yaml_filename: "/home/swlee/roboracer_ws/maps/NEW.yaml"
```

```bash
cd ~/roboracer_ws
colcon build --base-paths src --packages-select f1tenth_mppi_nav
```

---

## 5. 로컬라이제이션

**터미널 2**
```bash
cd ~/roboracer_ws
source env.sh
ros2 launch f1tenth_mppi_nav localization_launch.py
```

**터미널 3 (RViz)**
- Add → By topic → `/particle_cloud` → PoseArray
- `frame [map] does not exist` 경고 → 조이패드로 차를 조금 움직이면 사라짐
- **`2D Pose Estimate`** 로 차의 실제 위치 클릭 + **머리 방향으로 드래그**
- 조이패드로 **헤어핀 곡선부에서** 천천히 3~4 m 주행

---

## 6. 경로 기록

**터미널 4**
```bash
cd ~/roboracer_ws
source env.sh
ros2 run f1tenth_mppi_nav record_path --ros-args -p base_frame:=base_link -p min_distance:=0.2 -p output_file:=/home/swlee/roboracer_ws/paths/NEW_lap.csv
```

- `base_frame:=base_link` 없으면 웨이포인트 0개로 끝남
- 한 바퀴 → `Ctrl+C`
- **헤어핀에서 벽에 붙지 말 것.** 최소 곡률반경 0.927 m 이상 필요
- 시작점 조금 못 미쳐서 마칠 것

---

## 7. 경로 검사

**터미널 4**
```bash
cd ~/roboracer_ws
python3 check_path.py paths/NEW_lap.csv
```

`>>> 주행 가능` 나올 때까지 6번 반복.

---

## 8. 자율주행

**터미널 4**
```bash
cd ~/roboracer_ws
source env.sh
ros2 launch f1tenth_mppi_nav mppi_real_launch.py path_file:=/home/swlee/roboracer_ws/paths/NEW_lap.csv 2>&1 | tee ~/roboracer_ws/mppi_run.log
```

출발 전 확인:
```bash
python3 check_pose.py paths/NEW_lap.csv
```

### 🔴 조이패드 손에 들고 실행

---

## 터미널 정리

| 터미널 | 계속 켜둠 | 용도 |
|---|---|---|
| 1 | ✅ | 브링업 |
| 2 | | SLAM(2단계) → 종료 후 로컬라이제이션(5단계) |
| 3 | ✅ | RViz |
| 4 | | 맵 저장 → 경로 기록 → 검사 → MPPI |

---

## 막힐 때

| 증상 | 해결 |
|---|---|
| `/odom` 없음, `Invalid frame ID "odom"` | 조이패드로 조향 한 번 |
| `frame [map] does not exist` | 조이패드로 차를 조금 움직이기 |
| RViz `No map received` | Map 디스플레이 Durability → `Transient Local` |
| 라이다 `Not Connected`, `SYN-SENT` | 라이다 전원 재투입 후 브링업 재실행. 포트 직접 열지 말 것 |
| `base_frame` 이 `base_footprint` | slam_params_file 경로 확인 (절대경로로) |
| 차가 후진 | `cmd_vel_to_ackermann.py` 의 `-speed` 가 살아있는지 확인 |
| 죽인 노드가 목록에 남음 | `ros2 daemon stop` |

자세한 배경은 `REAL_CAR_GUIDE.md`.
