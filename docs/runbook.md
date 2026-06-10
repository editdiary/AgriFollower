# 🏃 [런북] 상황별 실행 명령어 모음

> "지금 이 상황에서 뭘 치면 되는가"만 모은 문서. 복사해서 그대로 실행하면 된다.
> 지표 해석·튜닝은 [`rl_code_guide.md`](rl_code_guide.md), 설계 근거는 [`rl_design/`](rl_design/) 참조.

---

## 0. 공통 준비 — 모든 새 터미널에서 제일 먼저

```bash
source /opt/ros/humble/setup.bash
source ~/rosorin_sim_ws/install/setup.bash
source ~/rosorin_sim_ws/.typerc
```

> 💡 `ros2: command not found`, `package 'rosorin_rl' not found` 류 오류의 99%는 이 소싱 누락이다.

**재빌드가 필요한 경우** (필요할 때만):

```bash
cd ~/rosorin_sim_ws
colcon build --symlink-install --packages-select rosorin_rl
source install/setup.bash   # 빌드 후 재소싱
```

| 무엇을 바꿨나 | 재빌드 필요? |
|---|---|
| 기존 `.py` 코드 수정, `rl_params.yaml` 값 수정 | ❌ (`--symlink-install` 덕분에 즉시 반영) |
| **새 파일** 추가, `setup.py`/`package.xml` 수정, launch 파일 **추가** | ✅ |

---

## 1. 로봇을 학습시킬 때 (터미널 2개)

**터미널 1 — 시뮬레이션 켜기** (학습엔 headless 권장, 처리량↑):

```bash
ros2 launch rosorin_rl rl_sim.launch.py headless:=true
```

| launch 인자 | 기본값 | 설명 |
|---|---|---|
| `headless` | `false` | `true`면 GUI 없이 서버만 (센서 렌더링은 EGL로 유지됨). 학습용 |
| `scenario` | `2` | 학습 시나리오 (1=정속 왕복, 2=StopGo 작물 단위 정지·이동). 기본 2 |

**터미널 2 — 학습 시작** (시뮬이 뜬 뒤에):

```bash
# 새 런마다 --modeldir 에 새 폴더를 지정하는 게 관례 (sac_1, sac_2 가 이미 있으니 다음은 sac_3)
ros2 run rosorin_rl train_sac --timesteps 200000 \
    --modeldir ~/rosorin_sim_ws/src/rosorin_rl/models/sac_3
```

> ⚠️ `--modeldir` 기본값은 `models/` 평면 저장이라, 런을 구분하려면 **반드시 런별 폴더를 지정**할 것.
> 안 그러면 다음 학습이 이전 런의 `sac_follow_final.zip` 을 덮어쓴다.
> 폴더 이름은 TensorBoard 런 번호(`sac_N`, 자동 증가)와 맞추면 나중에 찾기 쉽다.

**`train_sac` 전체 플래그:**

| 플래그 | 기본값 | 설명 |
|---|---|---|
| `--algo {sac,ppo}` | `sac` | 알고리즘 (SAC 1순위, PPO는 베이스라인 비교용) |
| `--timesteps N` | `100000` | 총 학습 스텝 수 |
| `--modeldir PATH` | `~/rosorin_sim_ws/src/rosorin_rl/models` | 체크포인트/최종 모델 저장 폴더 (위 ⚠️ 참조) |
| `--logdir PATH` | `~/rosorin_sim_ws/rl_logs` | TensorBoard 로그 + 에피소드 CSV 폴더 |
| `--ckpt-freq N` | `10000` | N 스텝마다 중간 체크포인트(.zip + 리플레이 버퍼 .pkl ~120MB) 저장 |
| `--config PATH` | (패키지 share의 `rl_params.yaml`) | 다른 파라미터 yaml로 학습 |
| `--resume ZIP` | — | 체크포인트에서 **이어서** 학습 (→ §2) |
| `--warm-start ZIP` | — | 정책 가중치만 이식해 **새 런** 시작 (→ §2). `--resume`과 동시 사용 불가 |

**자주 쓰는 변형:**

```bash
# 파이프라인이 도는지만 빠르게 확인하는 짧은 검증 런
ros2 run rosorin_rl train_sac --timesteps 1500

# PPO 베이스라인 비교 학습
ros2 run rosorin_rl train_sac --algo ppo --timesteps 100000 \
    --modeldir ~/rosorin_sim_ws/src/rosorin_rl/models/ppo_1
```

**학습을 중간에 멈추고 싶으면** 터미널 2에서 `Ctrl+C` 한 번 — 그 시점까지의 모델이
`{modeldir}/sac_follow_final.zip`(+리플레이 버퍼)으로 **자동 저장**되므로 안심하고 끊어도 된다.

---

## 2. 학습을 이어서 하거나, 보상을 바꿔 재학습할 때

두 가지 모드가 있고 **용도가 다르다**:

| | `--resume` | `--warm-start` |
|---|---|---|
| 복원 범위 | 정책 + 리플레이 버퍼 + 옵티마이저 + ent_coef + 스텝 카운터 | **정책 가중치만** (나머지 전부 초기화) |
| TensorBoard | 같은 `sac_N` 런에 이어서 기록 | 새 `sac_N+1` 런 생성 |
| 언제 쓰나 | 같은 설정으로 단순히 더 돌리고 싶을 때 (중단 재개) | **보상 함수/계수를 바꾼 뒤** 재학습할 때 |

> ⚠️ 보상을 바꿨는데 `--resume` 하면 안 된다 — 버퍼에 쌓인 옛 보상값이 그대로 재사용돼 학습이 오염된다.

```bash
# 중단 재개: 마지막 체크포인트(또는 final)에서 이어서
ros2 run rosorin_rl train_sac --timesteps 300000 \
    --resume ~/rosorin_sim_ws/src/rosorin_rl/models/sac_2/sac_follow_200000_steps.zip \
    --modeldir ~/rosorin_sim_ws/src/rosorin_rl/models/sac_2

# 보상 변경 후 재학습: 가중치만 이식, 새 런으로
ros2 run rosorin_rl train_sac --timesteps 200000 \
    --warm-start ~/rosorin_sim_ws/src/rosorin_rl/models/sac_2/sac_follow_final.zip \
    --modeldir ~/rosorin_sim_ws/src/rosorin_rl/models/sac_3
```

`--resume` 시 같은 폴더의 리플레이 버퍼 `.pkl` 을 **자동으로 찾아 복원**한다
(`sac_follow_replay_buffer_200000_steps.pkl` / `sac_follow_final_replay_buffer.pkl`).
"⚠️ Replay Buffer 파일 없음" 경고가 뜨면 빈 버퍼로 재개되어 초반에 잠시 출렁일 수 있다.

---

## 3. 학습 도중 들여다보고 싶을 때

```bash
# TensorBoard로 학습 곡선 보기 (→ §5 접속 방법)
tensorboard --logdir ~/rosorin_sim_ws/rl_logs --bind_all

# headless 시뮬에 GUI만 붙여서 거동 육안 확인 (noVNC 데스크톱 안 터미널에서)
ign gazebo -g

# 센서/토픽 시각화
rviz2

# 토픽 직접 확인 (진단)
ros2 topic hz /scan                    # LiDAR 주기 확인
ros2 topic echo /target/features       # 타겟 특징 [x_norm, y_norm, d_t, θ_t, visible]
ros2 topic echo /worker/pose --once    # 작업자 현재 위치
ros2 topic list                        # 전체 토픽 목록
```

> 💡 GUI를 붙이면 학습 처리량이 떨어지므로 확인 후 GUI 창만 닫으면 된다 (서버는 계속 돈다).

---

## 4. 학습이 끝났을 때 — 무엇을 확인하나

### 4-1. 산출물 위치

| 산출물 | 경로 | 비고 |
|---|---|---|
| 최종 모델 | `{modeldir}/sac_follow_final.zip` | Ctrl+C 중단 시에도 저장됨 |
| 최종 리플레이 버퍼 | `{modeldir}/sac_follow_final_replay_buffer.pkl` | ~120MB, resume용 |
| 중간 체크포인트 | `{modeldir}/sac_follow_{N}_steps.zip` (+ `..._replay_buffer_{N}_steps.pkl`) | 10k 스텝마다 |
| TensorBoard 로그 | `rl_logs/sac_N/events.out.tfevents.*` | **학습 종료 후에도 영구 보존** |
| 에피소드 CSV | `rl_logs/monitor_sac_N.monitor.csv` | 에피소드별 리턴/길이/종료사유 |

### 4-2. 확인 순서 (권장 루틴)

```bash
# ① TensorBoard 켜기 — 학습이 끝나도 된다! (→ §5 접속 방법)
tensorboard --logdir ~/rosorin_sim_ws/rl_logs --bind_all

# ② 에피소드 CSV 꼬리 확인 — "마지막엔 어떻게 죽었나"의 ground truth
tail -30 ~/rosorin_sim_ws/rl_logs/monitor_sac_2.monitor.csv
# 컬럼: r(리턴), l(길이), t(경과초), terminal(종료사유: success/lost/env_collision/target_collision/stuck)

# ③ 어느 체크포인트가 최적인지 선정 + 거동 육안 확인 (→ §6-1: analyze_log 사전선별 → eval_sweep 확정)
```

**TensorBoard에서 보는 순서** (상세는 [`rl_code_guide.md`](rl_code_guide.md) §4):

1. **잘 됐나?** — `rollout/ep_rew_mean` · `ep_len_mean` · `episode/success_rate` 우상향 여부
2. **왜 죽었나?** — `episode/*_rate` (lost / env_collision / target_collision / stuck 비율)
3. **어느 보상 항이 끌고/막았나?** — `reward/r_*_mean` 분해
4. **어떻게 행동했나?** — `action/abs_*_mean` · `state/d_t_mean`(0.87 수렴이 이상적) · `in_aisle_ratio`
5. **최적화는 건강했나?** — `train/ent_coef` · `critic_loss` (actor_loss는 음수로 깊어지는 게 정상)

> ⚠️ `episode/*_rate` 는 최근 100 에피소드 롤링 윈도라 급변 직후엔 과거가 섞여 보인다.
> "마지막 상태" 판단은 ②의 monitor CSV 꼬리가 정확하다.

pandas 분석 예시:

```python
import pandas as pd
df = pd.read_csv('~/rosorin_sim_ws/rl_logs/monitor_sac_2.monitor.csv', skiprows=1)
print(df.tail(50)['terminal'].value_counts())   # 최근 50 에피소드 종료사유 분포
```

---

## 5. TensorBoard 보는 법 (학습 중·후 공통)

> ✅ **TensorBoard는 학습이 끝난 뒤에도 된다.** 이벤트 파일이 `rl_logs/sac_N/` 에 영구 저장되므로
> 언제든 다시 열 수 있다. "안 열렸던" 원인은 TB가 아니라 **포트** 문제다 (아래).

```bash
# 컨테이너 안에서 실행 (--bind_all 필수: 외부 접속 허용)
tensorboard --logdir ~/rosorin_sim_ws/rl_logs --bind_all
```

### 호스트 PC 브라우저에서 보기

이 컨테이너는 noVNC용 `6080` 포트만 포워딩되어 있어 (`-p 6080:8080`),
호스트 브라우저에서 `localhost:6006` 으로는 **열리지 않는다**. 방법:

```bash
# ① (호스트 터미널에서) 컨테이너 IP 확인
docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' <컨테이너명>
# → 예: 172.17.0.2

# ② 호스트 브라우저에서 접속
#    http://172.17.0.2:6006
```

원격 서버라면 (호스트에 SSH로 붙는 경우) 내 PC에서 터널을 뚫는다:

```bash
ssh -L 6006:<컨테이너IP>:6006 <user>@<호스트주소>
# → 내 PC 브라우저에서 http://localhost:6006
```

**근본 해결 (선택):** 다음에 컨테이너를 재생성할 일이 있으면 `docker run` 에 `-p 6006:6006` 을
추가해 두면 호스트에서 `http://<호스트주소>:6006` 으로 바로 열린다 (`setup_process.md` 의 run 명령 참조).

**폴백:** noVNC 데스크톱(`http://<호스트>:6080`) 안의 브라우저에서 `http://localhost:6006` — 포워딩 불필요.

### 런 폴더 규칙

- 새 학습마다 `rl_logs/sac_1/`, `sac_2/`, … 자동 증가 (PPO는 `ppo_1/`, …)
- `--resume` 은 **같은 폴더에 이어서** 기록 (곡선에 불연속이 보이는 게 정상)
- TB 왼쪽 사이드바에서 런별 체크박스로 비교 가능 (SAC vs PPO 수렴 속도 비교 등)

---

## 6. 학습된 모델을 평가할 때

**거동을 눈으로 보려면 시뮬을 GUI 모드로** 다시 띄운다 (터미널 1):

```bash
ros2 launch rosorin_rl rl_sim.launch.py    # headless 없이 = GUI 포함
```

**평가 실행** (터미널 2):

```bash
ros2 run rosorin_rl eval_policy \
    --model ~/rosorin_sim_ws/src/rosorin_rl/models/sac_2/sac_follow_final.zip \
    --episodes 10
```

**`eval_policy` 전체 플래그:**

| 플래그 | 기본값 | 설명 |
|---|---|---|
| `--model ZIP` | **(필수)** | 평가할 모델 .zip 경로 (final 또는 중간 체크포인트) |
| `--episodes N` | `5` | 평가 에피소드 수 |
| `--algo {sac,ppo}` | `sac` | 모델 알고리즘과 일치시킬 것 |
| `--config PATH` | — | 다른 yaml로 평가 (노이즈 강건성 테스트 등, 아래 예시) |
| `--dump-csv PATH` | — | 스텝별 계측값(보상 항·센서 거리·action)을 CSV로 저장 → 임계값 보정·구간별 진단용 |

**출력 해석** — 에피소드마다 한 줄 + 마지막 요약:

| 항목 | 의미 |
|---|---|
| 리턴 / 길이 | 에피소드 누적 보상 / 생존 스텝 (1500 스텝 생존 = success) |
| 종료 | `success` / `lost`(타겟 놓침) / `env_collision` / `target_collision` / `stuck` |
| ω 부드러움 | 각속도 명령 변화량의 std — **낮을수록 부드러운 주행** (proposal §6.3-②) |
| 밴드 점유율 | \|d_t − 0.87m\| < 0.15m 인 스텝 비율 — **높을수록 거리 유지 잘함** |
| d_t 표준편차 | 타겟 거리 변동 — 낮을수록 일정한 간격 유지 |

**자주 쓰는 변형:**

```bash
# 중간 체크포인트끼리 비교 (어느 시점 모델이 제일 나은지)
ros2 run rosorin_rl eval_policy \
    --model ~/rosorin_sim_ws/src/rosorin_rl/models/sac_2/sac_follow_150000_steps.zip --episodes 10

# 스텝별 상세 데이터를 CSV로 떠서 분석
ros2 run rosorin_rl eval_policy \
    --model ~/rosorin_sim_ws/src/rosorin_rl/models/sac_2/sac_follow_final.zip \
    --episodes 10 --dump-csv ~/rosorin_sim_ws/rl_logs/eval_sac2_detail.csv

# 노이즈 강건성 평가 (proposal §6.3-③): noise.target_pct 를 0.03→0.3 으로 올린 yaml 을 만들어 지정
cp ~/rosorin_sim_ws/src/rosorin_rl/config/rl_params.yaml /tmp/rl_params_noise03.yaml
#   → /tmp/rl_params_noise03.yaml 에서 noise: target_pct: 0.3 으로 수정 후
ros2 run rosorin_rl eval_policy \
    --model ~/rosorin_sim_ws/src/rosorin_rl/models/sac_2/sac_follow_final.zip \
    --config /tmp/rl_params_noise03.yaml --episodes 10
```

### 6-1. 최적 체크포인트 고르기 — 2단계 (로그 사전선별 → 소수 확정평가)

체크포인트 수십 개를 전부 sim 평가하는 건 비싸다. **① 학습 로그로 후보를 좁히고(no sim)
→ ② 소수만 deterministic 평가** 하는 2단계가 빠르고 정확하다.

**Stage 0 — `analyze_log` 로 후보 추천 (수 초, sim 불필요)**

```bash
ros2 run rosorin_rl analyze_log \
    --logdir rl_logs/sac_1 \
    --models-dir src/rosorin_rl/models/1_main-train_sac1 --top 4 --min-step 300000
```
- step별 success/reward/실패율 곡선 + **정점 구간·후반 열화 진단**을 출력한다.
- **reward 1차·success 2차**로 후보 N개(정점 구간 + 최신 ckpt)를 골라 바로 붙여넣을
  `eval_sweep --models ...` 명령을 생성한다. `--min-step` 은 미수렴 초반을 표에서 숨길 뿐.

> ⚠️ **학습 로그(monitor·TB)는 탐색(stochastic)·rolling window 통계라 그것만으로 고르면 안 된다.**
> 특히 resume 직후 success_rate 가 일시적으로 "100%" 로 보이는 건 윈도 아티팩트(같은 구간 reward 가
> 낮으면 가짜) → **후보 랭킹은 reward 우선**. 사전선별일 뿐 최종 선정은 Stage 1 이 한다.

**Stage 1 — `eval_sweep` 로 후보만 deterministic 평가·랭킹 (sim 필요)**

`eval_sweep` 은 후보들을 **한 sim 연결로 순차 deterministic 평가**해 비교 CSV +
정렬된 랭킹 표를 출력한다 (성공률 → 평균 리턴 순, 1위 `★`).

```bash
# 터미널 1: sim (headless 권장 — 빠름)
ros2 launch rosorin_rl rl_sim.launch.py headless:=true

# 터미널 2: Stage 0 이 출력한 명령을 그대로 실행 (후보 4개 → 30ep 권장)
ros2 run rosorin_rl eval_sweep \
    --models <Stage 0 가 추천한 .zip 4개> \
    --episodes 30 --out rl_logs/eval_sweep_sac1.csv
#   후반 ckpt(800k)도 포함해 로그의 "후반 열화" 신호를 deterministic 으로 교차검증한다.
#   직접 넓게 보고 싶으면 --glob "...sac_follow_[4-8][05]0000_steps.zip" 로 50k 간격 스윕도 가능.
```

**소요 시간** — sim은 uncapped RTF(`real_time_factor=0`)지만 RGB-D 렌더링 병목으로 **~20 steps/s**
(학습 실측 19 steps/s). 1 에피소드(최대 1500스텝=150초 sim) ≈ **벽시계 75~80초**.
- 4 모델 × 30ep(=120 에피소드) ≈ **1.5~2.5시간**, × 20ep ≈ 1~1.7시간.
- 💡 **먼저 보정:** `eval_policy --model <후보> --episodes 2` 로 1 에피소드 벽시계를 재고 ×120 으로
  전체를 예측. 너무 길면 `--episodes 20` 으로 낮춰도 차이 식별엔 충분.

**Stage 2 — 랭킹 상위 육안 검증 후 확정 (필수)**

> 💡 곡선·지표가 좋아도 **거동 육안 확인은 필수** — 제자리 진동 같은 꼼수(reward hacking)는
> 숫자만으론 안 보인다 (`rl_code_guide.md` §4 "꼼수 의심 거동").
> → 랭킹 1~2위만 GUI(§6 상단 `rl_sim.launch.py` headless 없이)로 `eval_policy --episodes 5` 관찰 후 확정.

**최종 확정 체크리스트** — 표 1위를 바로 확정하지 말고 다음을 거친다:
1. **육안 검증 통과** — 위 GUI 관찰에서 추종 거동이 자연스럽고 reward hacking 없음.
2. **박빙이면 숫자에 과적합 금지** — 30ep에서 27/30 vs 26/30 차이는 통계적으로 무의미할 수 있음.
   이럴 땐 부차 지표(**밴드 점유율↑ / d_t σ↓ / ω 부드러움↓**)가 나은 쪽을 택한다.
3. **(선택) 배포 전 노이즈 강건성 1회** — 1위만 `--config` 로 noise 0.3 재평가(위 §6 노이즈 예시).
4. **확정 모델 보관** — 고른 `.zip` 을 명확한 이름으로 복사:
   `cp .../sac_follow_670000_steps.zip src/rosorin_rl/models/1_main-train_sac1/best_sac1.zip`
   → 이후 PPO 비교·보고서에서 이 파일을 참조.

---

## 7. 빠른 트러블슈팅

| 증상 | 원인 / 해결 |
|---|---|
| `ros2: command not found` / 패키지 못 찾음 | §0 소싱 3종 누락 — 새 터미널마다 다시 |
| TensorBoard가 브라우저에서 안 열림 | §5 포트 문제 — `--bind_all` + 컨테이너 IP 접속 (TB 자체는 학습 후에도 동작) |
| 학습 시작 직후 멈춰 있음 | 시뮬(터미널 1)이 안 떠 있거나 일시정지 상태 — `ros2 topic hz /scan` 으로 확인 |
| 코드 고쳤는데 반영 안 됨 | 새 파일/`setup.py` 변경은 재빌드 필요 (§0 표) |
| 작업자 속도를 바꿨는데 절반만 반영됨 | `target.speed` 는 **두 곳** 수정: `config/rl_params.yaml` + `launch/rl_sim.launch.py` 의 `speed` 파라미터 (컨트롤러 노드는 yaml을 읽지 않음) |
| resume 했더니 "Replay Buffer 파일 없음" 경고 | 체크포인트 `.zip` 옆에 `.pkl` 이 없는 경우 — 빈 버퍼로 재개되며 초반 출렁임은 정상 |
| noVNC만 "서버에 연결하지 못했습니다" (SSH·학습은 정상) | x11vnc 크래시 — 아래 §7-1 복구 명령으로 재시작 |
| 깊은 원인 분석이 필요한 오류 | [`troubleshooting.md`](troubleshooting.md) (치명적 오류 기록부) |

### 7-1. noVNC 연결 실패 복구 (x11vnc 크래시)

noVNC 체인은 `브라우저 → websockify(:8080, 호스트 6080) → x11vnc(:5900) → Xvfb(:0)`.
이 중 **x11vnc가 가끔 Xlib 스레드 race로 크래시**한다
(`xcb_io.c: Assertion '!xcb_xlib_unknown_req_in_deq' failed` → SIGABRT, `-threads` 옵션의 알려진 이슈).
websockify는 살아 있어서 noVNC 페이지는 뜨지만 5900 연결이 거부되는 게 증상.
**Xvfb·KDE·학습 프로세스는 무사하므로 x11vnc만 재시작하면 된다** (컨테이너 안 터미널에서):

```bash
# 진단: 5900 리스너가 없고 x11vnc 프로세스도 없으면 이 케이스
pgrep -af x11vnc || echo "x11vnc 죽음"

# 복구: entrypoint 프로세스에서 환경변수(DISPLAY/비밀번호)를 가져와 동일 설정으로 재기동
eval "$(tr '\0' '\n' < /proc/$(pgrep -f novnc_proxy | head -1)/environ \
    | grep -E '^(DISPLAY|PASSWD|BASIC_AUTH_PASSWORD)=' | sed 's/^/export /')"
nohup /usr/local/bin/x11vnc -display "${DISPLAY}" -passwd "${BASIC_AUTH_PASSWORD:-$PASSWD}" \
    -shared -forever -repeat -xkb -snapfb -threads -xrandr "resize" -rfbport 5900 \
    >/tmp/x11vnc_restart.log 2>&1 &
```

재기동 후 noVNC를 새로고침하면 바로 붙는다 (비밀번호 동일).
자주 재발하면 위 명령에서 `-threads` 를 빼고 띄우면 크래시 원인 자체가 사라진다 (화면 갱신이 약간 느려지는 트레이드오프).
