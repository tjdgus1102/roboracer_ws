source /opt/ros/humble/setup.bash
source ~/roboracer_ws/install/local_setup.bash
# 공식 f110_gym을 이 터미널에서만 우선 사용 (전역 pip의 race_stack fork는 건드리지 않음)
export PYTHONPATH=$HOME/roboracer_ws/f1tenth_gym/gym:$PYTHONPATH
