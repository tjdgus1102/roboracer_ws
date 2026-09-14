#!/usr/bin/env bash
# MPPI / RPP / DWB 비교 주행 자동 실행.
#
#   tools/run_comparison.sh [clean|obs] [반복횟수] [1회주행초]
#   예: tools/run_comparison.sh obs 3 120
#
# 시나리오별로 브리지를 직접 띄우고 내린다 (맵이 다르므로). rviz 는 안 띄운다.
# 이미 /bridge 가 떠 있으면 중단한다 -- 브리지가 둘이면 토픽이 섞인다.
#
# 결과: laps/compare/<시나리오>/<컨트롤러>/run<N>/ 에
#         lap_*.csv   50Hz 주행 기록 (t,x,y,v_odom,v_cmd,v_cap)
#         lap_*.png   record_lap 이 그린 속도 프로파일 대조
#         controller.log  컨트롤러 로그 (실패 횟수 집계용)
# set -u 는 ROS setup.bash 가 미정의 변수를 참조해서 못 쓴다
set -eo pipefail

WS="$HOME/roboracer_ws"
SCEN="${1:-obs}"
REPS="${2:-3}"
DUR="${3:-120}"

case "$SCEN" in
  clean) MAP="$WS/maps/big_0723";     MAPYAML="$WS/maps/big_0723.yaml" ;;
  obs)   MAP="$WS/maps/big_0723_obs"; MAPYAML="$WS/maps/big_0723_obs.yaml" ;;
  *) echo "시나리오는 clean 또는 obs"; exit 1 ;;
esac

# 레이스라인 idx 95. 통과폭 2.50 m, 최근접 장애물 3.04 m,
# 진행방향 10 m 앞까지 장애물 없음 -- 세 컨트롤러 모두 출발할 수 있는 지점.
SX=8.3098; SY=1.2521; STHETA=0.0386; QZ=0.019299; QW=0.999814

PATH_CSV="$WS/raceline_opt/big_0723_raceline.csv"
CFG="$WS/src/f1tenth_mppi_nav/config"
SIMYAML="$WS/src/f1tenth_gym_ros/config/sim.yaml"

source "$WS/env.sh"

if ros2 node list 2>/dev/null | grep -qx "/bridge"; then
  echo "이미 /bridge 가 떠 있습니다. 그 터미널을 먼저 종료하세요."
  exit 1
fi

echo "=== 시나리오 $SCEN / 반복 $REPS / 1회 ${DUR}s ==="
ros2 run f1tenth_gym_ros gym_bridge --ros-args -r __node:=bridge \
  --params-file "$SIMYAML" \
  -p map_path:="$MAP" -p sx:=$SX -p sy:=$SY -p stheta:=$STHETA \
  > /tmp/bridge_$SCEN.log 2>&1 &
BRIDGE=$!
trap 'kill $BRIDGE 2>/dev/null || true' EXIT
until ros2 node list 2>/dev/null | grep -qx "/bridge"; do sleep 1; done
echo "브리지 기동 (pid $BRIDGE)"

for CTRL in mppi rpp dwb; do
  case "$CTRL" in
    mppi) PARAMS="$CFG/nav2_params.yaml" ;;
    rpp)  PARAMS="$CFG/nav2_params_rpp.yaml" ;;
    dwb)  PARAMS="$CFG/nav2_params_dwb.yaml" ;;
  esac
  for R in $(seq 1 "$REPS"); do
    OUT="$WS/laps/compare/$SCEN/$CTRL/run$R"
    rm -rf "$OUT"; mkdir -p "$OUT"
    echo "--- $CTRL run$R ---"

    # 매 실행 전 차를 출발점으로 되돌린다 (gym 은 속도까지 0 으로 초기화한다)
    ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
      "{header: {frame_id: map}, pose: {pose: {position: {x: $SX, y: $SY}, orientation: {z: $QZ, w: $QW}}}}" \
      > /dev/null 2>&1
    sleep 2

    # ros2 run 래퍼로 띄우면 SIGINT 가 노드까지 전달되지 않아 save() 가 안 돈다.
    # 설치된 실행파일을 직접 띄워야 kill -INT 로 저장시킬 수 있다.
    "$WS/install/f1tenth_mppi_nav/lib/f1tenth_mppi_nav/record_lap" --ros-args \
      -r odom:=/ego_racecar/odom \
      -p base_frame:=ego_racecar/base_link \
      -p path_file:="$PATH_CSV" -p map_yaml:="$MAPYAML" -p output_dir:="$OUT" \
      > "$OUT/record.log" 2>&1 &
    REC=$!
    sleep 2

    # SIGINT 로 끊어야 ros2 launch 가 자식 노드까지 정리하고 내려간다
    timeout --signal=INT "$DUR" ros2 launch f1tenth_mppi_nav mppi_path_follow_launch.py \
      params_file:="$PARAMS" path_file:="$PATH_CSV" \
      > "$OUT/controller.log" 2>&1 || true

    # record_lap 은 KeyboardInterrupt 에서 save() 한다
    kill -INT $REC 2>/dev/null || true
    wait $REC 2>/dev/null || true

    # ros2 launch 에 SIGINT 를 줘도 자식 노드가 남는 경우가 있다. 다음 실행에
    # 섞이지 않게 확실히 정리한다. 브리지와 사용자의 localization lifecycle
    # manager 는 건드리지 않도록 패턴을 좁게 잡는다.
    pkill -f 'f1tenth_mppi_nav/lib/f1tenth_mppi_nav/cmd_vel_to_ackermann' 2>/dev/null || true
    pkill -f 'f1tenth_mppi_nav/lib/f1tenth_mppi_nav/path_follower' 2>/dev/null || true
    pkill -f 'f1tenth_mppi_nav/lib/f1tenth_mppi_nav/speed_profile' 2>/dev/null || true
    pkill -f 'f1tenth_mppi_nav/lib/f1tenth_mppi_nav/record_lap' 2>/dev/null || true
    pkill -f 'nav2_controller/controller_server' 2>/dev/null || true
    pkill -f '__node:=lifecycle_manager_navigation' 2>/dev/null || true
    sleep 3
  done
done

echo "=== 완료. 분석: python3 tools/compare_laps.py $SCEN ==="
