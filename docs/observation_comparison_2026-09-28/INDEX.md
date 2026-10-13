# 실기–sim 관측 비교 그래프

입력: /home/geonhu/sim2real/run2.csv, /home/geonhu/sim2real/sim2.csv.
비교 시간: 정책 시작 기준 0–26.84 s. 실기 로그의 전원 차단 구간은 제외했다.
같은 이동 명령으로 각각 정책을 실행한 폐루프 결과이며, 관절 target이 같은 실험은 아니다.

| 항목 | 전체 구간 | 서기 확대 | 보행 확대 |
|---|---|---|---|
| IMU: gyro xyz + projected gravity xyz | [전체](/home/geonhu/deploy_ws/src/kid_rl/docs/observation_comparison_2026-09-28/imu.png) | [서기 5–7 s](/home/geonhu/deploy_ws/src/kid_rl/docs/observation_comparison_2026-09-28/zoom_5-7_imu.png) | [보행 15–17 s](/home/geonhu/deploy_ws/src/kid_rl/docs/observation_comparison_2026-09-28/zoom_15-17_imu.png) |
| 다리 12관절 위치 | [전체](/home/geonhu/deploy_ws/src/kid_rl/docs/observation_comparison_2026-09-28/legs_pos.png) | [서기 5–7 s](/home/geonhu/deploy_ws/src/kid_rl/docs/observation_comparison_2026-09-28/zoom_5-7_legs_pos.png) | [보행 15–17 s](/home/geonhu/deploy_ws/src/kid_rl/docs/observation_comparison_2026-09-28/zoom_15-17_legs_pos.png) |
| 상체 13관절 위치 | [전체](/home/geonhu/deploy_ws/src/kid_rl/docs/observation_comparison_2026-09-28/upper_pos.png) | [서기 5–7 s](/home/geonhu/deploy_ws/src/kid_rl/docs/observation_comparison_2026-09-28/zoom_5-7_upper_pos.png) | [보행 15–17 s](/home/geonhu/deploy_ws/src/kid_rl/docs/observation_comparison_2026-09-28/zoom_15-17_upper_pos.png) |
| 다리 12관절 속도 | [전체](/home/geonhu/deploy_ws/src/kid_rl/docs/observation_comparison_2026-09-28/legs_vel.png) | [서기 5–7 s](/home/geonhu/deploy_ws/src/kid_rl/docs/observation_comparison_2026-09-28/zoom_5-7_legs_vel.png) | [보행 15–17 s](/home/geonhu/deploy_ws/src/kid_rl/docs/observation_comparison_2026-09-28/zoom_15-17_legs_vel.png) |
| 상체 13관절 속도 | [전체](/home/geonhu/deploy_ws/src/kid_rl/docs/observation_comparison_2026-09-28/upper_vel.png) | [서기 5–7 s](/home/geonhu/deploy_ws/src/kid_rl/docs/observation_comparison_2026-09-28/zoom_5-7_upper_vel.png) | [보행 15–17 s](/home/geonhu/deploy_ws/src/kid_rl/docs/observation_comparison_2026-09-28/zoom_15-17_upper_vel.png) |
| 다리 action (참고) | [전체](/home/geonhu/deploy_ws/src/kid_rl/docs/observation_comparison_2026-09-28/legs_action.png) | [서기 5–7 s](/home/geonhu/deploy_ws/src/kid_rl/docs/observation_comparison_2026-09-28/zoom_5-7_legs_action.png) | [보행 15–17 s](/home/geonhu/deploy_ws/src/kid_rl/docs/observation_comparison_2026-09-28/zoom_15-17_legs_action.png) |

[주파수별 성분]( /home/geonhu/deploy_ws/src/kid_rl/docs/observation_comparison_2026-09-28/spectra.png ) · [수치 요약](/home/geonhu/deploy_ws/src/kid_rl/docs/observation_comparison_2026-09-28/comparison_stats.txt)

## 정책 입력과 그래프의 관계

- gyro: 몸통 좌표계 각속도, 단위 rad/s. 기울기 각도와는 다른 값이다.
- projected gravity: 몸통 좌표계의 정규화된 중력 방향, 단위 없음.
- joint_pos: 그래프는 실제 관절 위치(rad)를 표시한다. 정책에는 여기서 해당 관절의 기본 위치를 뺀 값이 들어간다. real과 sim의 기본 위치는 같은 설정이므로 차이 비교는 동일하다.
- joint_vel: real은 Dynamixel 속도 레지스터 값(rad/s), sim은 시뮬레이터의 관절 속도(rad/s)다.
- 속도 그림의 real Δpos/Δt는 50 Hz 위치 로그의 중앙 차분 근사다. 센서 처리 차이를 확인하기 위한 참고이며 실제 순간 속도의 참값으로 간주하지 않는다. 시간 양자화·누락·위치 양자화의 영향도 받을 수 있다.
- sim 곡선은 실기 policy_t_s에 선형 보간했다. 시간 원점을 맞췄으며, 파형을 일치시키기 위한 임의의 지연 보정이나 필터는 적용하지 않았다.
- 위치 그림의 target은 CSV에 기록된 값이다. 현재 sim 폐루프 코드에서 기록된 target의 관절 한계 처리와 실제 액추에이터 전달 경로가 다르므로 실제 입력 목표각이라고 단정하지 않는다.

## 해석 범위

폐루프에서 관측 차이는 센서 처리뿐 아니라 서로 다른 실제 움직임에서도 생긴다. 파형이 다르다는 사실만으로 센서가 잘못됐거나 필터를 맞추면 문제가 해결된다고 결론 내릴 수 없다. 같은 입력을 넣은 open-loop 비교 또는 같은 움직임을 측정한 센서 비교가 추가로 필요하다.
