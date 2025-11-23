# Crazy Controller

simulation이든 real이든 제일 성능 좋은 제어기는 AUG입니다!!
AUG는 서브 함수들까지 다 적용되어있는데, PP랑 MAP는 순수한 메인 알고리즘만 들어있습니다!!

mu 0.95에 속도 리미트 없는 트랙 기준으로 실차에서 mean CTE 15cm 이내 나와야 정상입니다!! 혹시 15cm 넘으면 이상한거니까 얘기 ㄱㄱ

1) Launch examples:

```bash
# MAP (default):
ros2 launch crazy_controller controller_launch.py mod:=real controller_mode:=MAP

# PP:
ros2 launch crazy_controller controller_launch.py mod:=sim controller_mode:=PP

# AUG:
ros2 launch crazy_controller controller_launch.py mod:=sim controller_mode:=AUG

```