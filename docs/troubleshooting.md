# 트러블슈팅 — 치명적 오류 기록부

시뮬/RL 작업 중 만난 **치명적이고 재현성 있는 오류**를 원인·해결과 함께 1항목씩 누적한다.
(세팅 절차 중 만난 일반 설치 오류는 `docs/setup_process.md`에 둔다. 여기는 "원인을 파고들어야 했던" 버그용.)

> **항목 추가 규칙:** 최신 항목을 위에. 각 항목은 **증상 → 배경 → 원인 → 진단 → 해결 → 검증** 순서로.

---

## 학습 처리량이 실시간(RTF≈1.0)에 묶임 — Fortress 는 `real_time_factor` 만 유효 (해결, 2026-06-08)

**증상**
headless 로 돌려도 학습이 ~8fps(RTF≈1.0)에서 더 안 빨라짐. 그런데 서버 자원은 **CPU<10%·GPU<25%로 남아돎** —
연산 포화처럼 보이지 않는데도 속도가 안 오름.

**배경**
world physics 가 `<real_time_factor>1.0</real_time_factor>` 로 실시간에 묶여 있었다. env 는 step 당
sim 0.1초를 `_sleep_sim` 으로 대기(`control.rate_hz=10`)하므로, RTF=1.0이면 wall 기준 최대 10fps 가 천장.

**원인**
real-time **throttle**. gz-sim SimulationRunner 가 `updatePeriod = max_step_size / real_time_factor` 로
목표 스텝 주기를 잡고, 물리를 일찍 끝낸 뒤 **남는 시간을 sleep** 한다 → 자원이 노는 건 연산 포화가 아니라
"빨리 끝내고 일부러 기다리는" 상태. RTF 가 1.0 *아래*가 아니라 정확히 1.0에 붙어 있던 게 throttle 의 신호.

**진단(요약)**
- 1차로 `<real_time_update_rate>0</real_time_update_rate>` 추가 → **무효**(RTF 그대로 1.0). 이유: 그건
  **Gazebo Classic 전용 태그**라 Fortress 의 SimulationRunner 가 읽지 않는다.
- gz-sim `ign-gazebo6/src/SimulationRunner.cc` 확인: `desiredRtf = physics->RealTimeFactor()`,
  `updatePeriod = stepSize / desiredRtf`, 그리고 **`if (desiredRtf < 1e-9) updatePeriod = 0`**(→ throttle 해제).
  즉 Fortress 의 노브는 `real_time_factor` 뿐이고, `0`이 "가능한 한 빠르게".

**해결**
physics 의 `<real_time_factor>` 를 `1.0` → **`0`**(무제한)으로. `greenhouse.sdf` 와 생성기
`gen_greenhouse_world.py` **양쪽** 수정(한쪽만 고치면 world 재생성 때 되돌아감). 1차 시도의
`real_time_update_rate` 라인은 제거. `--symlink-install` 이라 재빌드 불필요 — 런치 재시작만.
(무제한이 불안정하면 유한값 `5`~`10` 으로 캡 가능.)

**검증**
`ign topic -e -t /stats` 의 `real_time_factor` 가 1.0 → **1~5x 로 상승**(센서 렌더 버스트·GPU 경합으로
출렁이는 건 정상 — 순간값 말고 평균 fps 로 판단). 학습 fps 동반 상승. 거동·학습 데이터는 `_sleep_sim` 이
sim-time 기준이라 RTF 가 변해도 불변(상세: `rl_code_guide.md` §검증·보정 4).

---

## `robot_state_publisher` "Moved backwards in time" 경고 폭주 (해결, 2026-06-05)

**증상**
`rl_sim.launch.py` 실행 직후부터 `[robot_state_publisher] Moved backwards in time, re-publishing joint transforms!`
경고가 초당 수십 건씩 무한 반복. 시뮬 자체는 동작하나 로그가 묻히고 TF 재발행이 계속 발생.

**배경**
launch를 Ctrl+C로 종료해도 가끔 `parameter_bridge` 프로세스만 살아남는 경우가 있다(gz 서버는 죽고 브리지만 고아화).
gz-transport는 머신 전역 디스커버리라, 좀비 브리지가 **새로 띄운 시뮬의 gz 토픽에도 자동으로 붙는다**.

**원인**
이전 세션의 좀비 `parameter_bridge`가 새 시뮬의 `/clock`을 같이 발행 → ROS `/clock` 발행자가 2개가 되어
두 스트림이 수~수십 ms 어긋나게 겹침 → sim time이 비단조(역행) → `use_sim_time` 노드(robot_state_publisher)가
시계가 뒤로 갔다고 판단해 경고 폭주.

**진단(요약)**
- `ros2 topic echo /clock` → 타임스탬프가 역행하는 구간 확인 (…861ms → 847ms → 다시 진행).
- `ros2 topic info /clock --verbose` → **Publisher count: 2** (둘 다 `ros_gz_bridge`).
- `ps aux | grep parameter_bridge` → 이전 세션 시각에 시작된 브리지 2개 잔존.
- 기각: launch의 `joint_state_publisher` 중복 발행 — 제거해도 경고 지속(단, 그 자체로 불필요한 중복이라 별도 제거함).

**해결**
좀비 브리지 kill. 이후 `/clock` 발행자 수가 2로 계속 보이면 `ros2 daemon stop`으로 데몬 캐시 갱신(실제는 1).
```bash
ps aux | grep parameter_bridge   # 이전 세션 잔존 프로세스 확인
kill <PID...>
ros2 daemon stop                 # topic info 가 stale 카운트를 보여줄 때
```
재발 예방: 시뮬 재실행 전 잔존 프로세스 확인 습관. (launch 종료 후 `ps aux | grep -E "parameter_bridge|ign gazebo"`)

**검증**
좀비 kill 후 `/clock` Publisher count = 1, 동일 시뮬 15초 관찰 동안 경고 0건 (kill 전 ~45건/초).

---

## `/depth_cam` 대각선(45/135/225/315°) 회색 렌더 (해결, 2026-06-03)

**증상**
커스텀 온실(`greenhouse_sim`)에서 로봇 yaw가 **정확히 대각선(45/135/225/315°, ~±5° 폭)** 일 때만
`/depth_cam/depth_cam` RGB가 화면 전체 균일 회색([120,121,121] = ogre2 센서 기본 clear color, 렌더된 지오메트리 0개).
인접 각도·축정렬(0/90/180/270°)·LiDAR는 모두 정상. 제조사 빈 월드는 무증상, 데모 월드는 간헐적, 커스텀 온실은 일관적.

**배경**
로봇 몸체를 키우려고 균일 스케일 포크(`urdf/robot_scaled.gazebo.xacro`, `S=1.83`)를 만들어 온실 런치가
이를 스폰하도록 했다. 스케일은 mesh `scale`·마운트 origin·질량(S³)·관성(S⁵)에 일괄 적용했었다.

**원인**
카메라/라이다가 붙은 **센서 링크(`camera_link0`, `lidar_frame`)의 visual/collision mesh `scale`까지 `${S}`로 키운 것**이 단독 원인.
스케일된 센서-링크 mesh가 ogre2 카메라의 view/frustum을 **대각선 회전에서 degenerate**시켜 씬 전체가 컬링됨.
축정렬 방위에서는 정상이라 각도 의존적으로만 드러났다.

**진단(요약)**
- **S=1.0(원본 크기) → 전 각도 OK / S=1.83 → 대각선만 회색** ⇒ 스케일이 단독 원인.
- GUI on/off 동일(헤드리스도 회색) → GUI↔센서 렌더 경합 아님.
- 단순화 월드(바닥+벽4, 작물·타일 제거)도 동일 → 워크로드/텍스처량 아님.
- 주행으로 대각선을 통과시키면 회색 구간을 지나 정상 복귀 → 상태 손상이 아니라 **방위별 프레임 렌더 실패**.
  (단, `set_pose` teleport·스폰 시 yaw 지정은 비-0 각도에서 항상 회색이 되는 별도 아티팩트가 있어 재현엔 주행만 충실했다.)
- 기각: plane→box, shadows off, near/far clip, 배경색 — 모두 무효.

**해결**
센서 링크의 visual/collision **mesh `scale`만 `1 1 1`로 되돌리고**, 마운트 joint origin은 **`×S` 그대로 유지**
(큰 몸체 표면에 장착 = 비가림). 몸체·바퀴 mesh는 `${S}` 유지 → 로봇 크기는 그대로 커진 채 버그만 제거.
- `urdf/ascamera_scaled.xacro` — 카메라 mesh 2곳(visual·collision) `scale` → `1 1 1`
- `urdf/rosorin_scaled.xacro` — 라이다 mesh 2곳(visual·collision) `scale` → `1 1 1`
- 카메라 광학 스펙(FOV 60°/clip 0.1–10 m/640×400)은 원래부터 스케일 대상이 아니라 기능 영향 없음.

**검증**
주행 제자리회전 스윕(0/30/44/45/46/60/90/135°) 전 각도 OK(대각선 픽셀 std≈44, 회색 0프레임), yaw0 씬 정상 렌더(비가림 확인).
`--symlink-install` 빌드라 xacro 수정 후 재빌드 불필요(재소싱·재실행만).
