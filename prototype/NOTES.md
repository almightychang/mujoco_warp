# Prototype notes: aligned stacks that never settle (#1798)

develop 브랜치다. 여러 해결책을 한 브랜치에서 실험한다. upstream PR로 낼 때는 원인별로 나눠서 새 브랜치에 정리한다.
`prototype/`은 PR에 넣지 않는다.

base: main `c384ac5`. 마지막 갱신 2026-10-18.

## 결론

#1798의 두 증상(mm 단위 drift, 높은 CG iteration)은 모두 collision 쪽 원인이다. solver에는 결함이 없다. contact가
맞으면 Warp CG는 같은 옵션의 MuJoCo C와 같은 속도로 수렴한다.

원인은 세 가지이고, 패턴이 같다. 정렬된 기하에서는 exact arithmetic에서 tie가 생긴다(대칭 평면, 겹치는 면,
일직선상의 점, 정다각형). 여기에 float32 반올림이 끼면, 엄격한 비교를 하는 알고리즘이 그 잡음을 신호로 착각한다.

```
 GJK ──▶ EPA ─────────────────────────▶ multicontact ─────────────▶ solver
          ▲ R1: 관통 pair를 버림           ▲ R2: 4점 선택이 틀리고 흔들림
          ▲ R1': 깊이 0에서 기울어진 법선
```

| 원인 | 어디서 | 무슨 일 | 증상 | 수정 |
| --- | --- | --- | --- | --- |
| R1 | EPA 시작 도형 | 원점이 도형의 면/모서리 위면 "닿기만 함"으로 판정해 pair를 버림 | plate가 mm 단위로 꺼짐 (18.75 mm drift) | 면을 평면으로 표현 (`2310f42`) |
| R1' | EPA 면 가지치기 | 깊이 0에서 진짜 답인 면이 "하한보다 가깝다"며 버려지고, 확인되지 않은 면이 contact로 나감 | 처음 닿을 때 최대 45° 기울어진 법선 | 같은 커밋 (2차 프로토타입에서 생긴 회귀) |
| R2 | multicontact `_polygon_quad` | 최대 넓이 사각형을 hill climbing으로 찾다 모서리를 버리거나 같은 점을 두 번 고름. 대칭 다각형에서는 반올림으로 다른 4점을 고름 | step의 24%에서 받침점이 틀림, CG 20–50 iteration, 0.1 mm 흔들림 | 고정 방향 greedy (`b2c5ae6`) |

## R1, R1': EPA 면을 평면으로 표현

EPA는 면을 "원점을 그 면에 투영한 벡터"로 저장하고 탐색 방향도 그 벡터에서 얻었다. 원점이 면 위에 있으면 그
벡터가 0이 된다. 그래서 시작 도형 단계가 절대 임계값(`MIN_DIST2/3/4`, #1532 fallback)으로 그런 면을 거부했고,
그 pair는 관통 아님으로 처리됐다. float32에서 그 임계값은 10 µm다.

수정 후 면은 바깥 단위 법선과 원점까지의 부호 있는 거리로 저장한다. 둘 다 면의 꼭짓점에서만 나오므로 support
방향, 상한, horizon 판정이 원점 위치와 무관하다. 원점은 면이나 모서리 위에 있어도 되고, 반올림으로 살짝 바깥에
있어도 된다. 시작 도형은 넓이가 0인 면만 거부한다. 원점 위치에 거는 임계값은 없어졌고, 남은 tolerance는 EPA
수렴 판정(깊이 정확도)뿐이다.

새 면은 상한보다 먼 것만 버린다. 하한보다 가까운 면은 반올림 때문에만 생긴다. 그때는 원점이 경계 위에 있는
경우라 그 면이 답이다(R1').

지나온 단계:
- 1차 (`bench/proto1.diff`): 시작 도형에서 원점이 면 위면 support 1회로 경계 면인지 확인했다. 면 하나는 풀었지만
  원점이 모서리 위(면 두 개)인 경우를 못 풀었다. EPA 한 iteration을 시작 도형의 면 하나에서만 손으로 돌린 셈이라
  3차의 특수한 경우다.
- 2차: 면을 평면으로 표현. 정적 sweep은 통과했지만, 깊이 0에서 기울어진 법선이 나와 6–8장 stack이 main보다
  나빠졌다(R1').
- 3차: 가지치기 조건 수정. 현재 커밋.

## R2: multicontact 4점 선택

### 무엇을 풀려는 단계인가

면-면 contact의 patch는 다각형(8–32 꼭짓점)이고, solver 비용 때문에 pair당 4점으로 줄인다. 4점은 patch의 지지력을
대신해야 한다. 조건은 두 가지다.
- 공간: 받침 영역이 patch를 덮어 압력 중심이 그 안에 있을 것
- 시간: step마다 같은 점을 고를 것(warmstart가 맞도록)

"최대 넓이"는 공간 조건의 대리 지표이고, 시간 조건은 다루지 않는다. 대칭 다각형(원판, 냄비 바닥)에서는 최대
넓이 사각형이 여러 개라, 정확하게 풀수록 그중 어느 것을 고를지를 반올림이 정한다.

### 원래 알고리즘

C의 `hull4`와 Warp의 `_polygon_quad`는 Dobkin–Snyder(1979)의 사각형 알고리즘이다. 네 점을 넓이가 엄격히 늘 때만
한 칸씩 옮긴다. Keikha 외(2017, arXiv:1708.00681)가 exact arithmetic에서도 최적해를 놓치는 16각형 반례를 냈다.
float32에서는 두 plate가 1e-4 rad 기울면 변끼리 얕게 교차해 생긴 여분의 꼭짓점(실제 기하, µm 단위)에서 tie가 생기고,
hill climbing이 잡음을 따라간다.
- 같은 index를 두 번 고름: `pick=4 4 0 2` → 모서리 (105, 148) 두 번, (105, −148) 빠짐
- 넓이 최대가 아닌 곳에서 멈춤: 변 위의 점 (38.2, 148)을 고르고 모서리 (105, 148)을 버림

되먹임은 이렇게 생긴다. 모서리가 틀린 step에서 받침점이 비대칭이라 torque가 생기고, 다음 step에 정상으로
돌아오면서 contact 집합이 바뀌어 warmstart가 맞지 않는다. CG가 덜 수렴하면 plate가 다시 기울고, 다시 여분
꼭짓점이 생긴다. Newton이나 impratio 1은 한 step에 완전히 수렴해 plate를 평평하게 두므로 이 버그를 가린다.

### 검토한 선택지

| 선택지 | 공간 | 시간 | 견고성 | 비용 | 결과 |
| --- | --- | --- | --- | --- | --- |
| 원래 hill climbing | ✗ plate에서 평균 11% 손실 | ✗ 대칭에서 35% 흔들림 | ✗ exact에서도 반례 | O(n) | 버그 원인 |
| 전수 탐색 (`7d5e8ff`) | ✓ | ✗ | ✓ | O(n⁴) | 원인 확인용 |
| 대각선 + scan (`3f6cbe4`) | ✓ 정확 | ✗ 대칭에서 48% 흔들림 | ✓ | O(n³), aloha_pot −10% | 정확하지만 느리고 일관성 없음 |
| 대각선 + 포인터 | ✓ (볼록할 때) | ✗ | ✗ 1e-8 잡음에서 7% 실패 | O(n²) | 단봉성 가정이 반올림에 깨짐 |
| Rote 선형 (2019) | ✓ (볼록할 때) | ✗ | ✗ 같은 가정 (추정) | O(n) | 구현 안 함 |
| 귀 제거 (Visvalingam–Whyatt) | △ | ✗ 대칭에서 85% | ✓ | O(n²) | 탈락 |
| **고정 방향 greedy** (`b2c5ae6`) | ✓ plate 정확, 대칭 7.5e-7 이내, 무작위 평균 3.5% | **✓ 0%** | ✓ 최댓값 찾기만 씀 | **O(n)** | **채택** |

정확도와 일관성 수치는 CPU 검증기에서 나왔다(`bench/quad_check.py`, `quad_coherence.py`, `quad_shapes.py`).

### 채택: 고정 방향 greedy

Gregorius, "Robust Contact Creation for Physics Simulations"(Valve, GDC 2015)의 방식이고, Jolt의 `PruneContactPoints`도
같은 구조다.
1. contact frame에 고정된 방향 `2 t1 + t2`로 가장 먼 꼭짓점
2. 1에서 가장 먼 꼭짓점
3. 1, 2와 가장 큰 삼각형을 만드는 꼭짓점
4. 삼각형 바깥으로 넓이를 가장 많이 더하는 꼭짓점

`t1`, `t2`는 MuJoCo가 법선에서 contact frame을 만들 때 쓰는 `orthogonals`의 접선이다. 출발점을 프레임에 고정하므로
대칭에서도 같은 4점을 고른다. 방향을 frame 축에서 비튼 것은 frame에 정렬된 면에서 tie가 생기지 않게 하려는 것이다.
퇴화 다각형에서는 2점 또는 3점을 돌려준다.

C와는 불규칙한 다각형에서 결과가 다르다. 근거는 `hull4`가 반증된 알고리즘이라는 점과 대칭에서의 일관성이다.

## 결과 (develop, R1 + R1' + R2)

정적 sweep (`bench/drop_rate.py`, 정렬 근처, world 4096):

| | main | develop |
| --- | --- | --- |
| 1–10 µm 누락 (box / mesh / plate끼리) | 55% / 50% / 56% | 0% / 0% / 0.1% |
| 10 µm–10 mm 누락 (최대) | 13% | 0.02% |
| 과대 contact (C보다 1 µm 넘게 깊음) | 0% | 0% |
| C 대비 깊이 오차 | ≤ 2.4e-9 m | 같음 |

drift grid (mesh/box × 2·4·8 mm × 2–8장, 42개 config, `results/greedy/`):

| | main | develop | MuJoCo C (mesh) |
| --- | --- | --- | --- |
| CG iteration 평균 (최대 config) | 25.9 | 2.1 | 2.5 |
| 최대 drift | 18.75 mm | 0.006 mm | 0.001 mm |
| iteration cap 도달 | config당 최대 37 | 0 | 0 |

MuJoCo C의 primitive box-box collider는 같은 grid에서 최대 30 iteration, 0.85 mm drift가 나온다.

solver 옵션 실험 (`bench/solver_exp.py`, 4 mm mesh 3/4/5장): CG, impratio 10, tol 1e-6 기준 1.4 / 1.7 / 2.0. impratio,
tolerance, Newton, cone을 바꿔도 0.8–2.0이고, contact 집합 변화는 모든 조건에서 0%다.

성능 (`results/quad_cost/greedy.txt`, nworld 4096/8192):

| 장면 | R1만 (`2310f42`, 원래 hill climbing) | develop |
| --- | --- | --- |
| aloha_pot | 2.156M sps, narrowphase 82.0 µs | 2.171M, 79.4 µs |
| aloha_clutter | 302k, 779 µs | 302k, 779 µs |
| cylinder 정렬 stack | 81.6k, 450 µs | 82.0k, 438 µs |
| box / mesh 정렬 stack | 1.2M / 1.06M | 2.3–2.4M / 2.8–3.1M |

CCD kernel은 register 128로 같고 spill은 비슷하다(`results/proto3/ccd_resources.txt`).

test:
- red test 10개 (`collision_gjk_test.py`의 `*_aligned_*`, `*_settling_*`): R1, main에서 7개 실패
- `collision_driver_test.py`에 3개:
  - `test_mesh_mesh_tilted_plates_corners`: R2, main에서 실패
  - `test_cylinder_box_contacts_coherent`: R2, main에서 실패
  - `test_mesh_mesh_touching_plates_normal`: R1' 회귀 방지, main에서 통과하고 R1' 버그 사본에서 실패
- `collision_gjk_test.py`와 `collision_driver_test.py` 전체가 통과한다(새 test 3개 추가 전에 250개).

## 관련 이슈 재실행 (`issues/RESULTS.md`)

- #1782 코멘트 사례(누운 cylinder, 16.5 µm): main에서는 −190 mm 깊이에 법선 (1,0,0)이었고, develop은 C와 같다. R1만으로
  고쳐진다.
- #1782 본문(contact가 410 m 떨어짐), #1510, mujoco #2471: main에서 재현되지 않는다.
- #1739, #1740: main과 결과가 같다. sphere/capsule의 GJK 경로(EPA를 거치지 않음)라 범위 밖이다. 같은 뿌리(반올림이
  엄격한 비교나 상쇄 계산에 끼어듦)지만 고칠 곳은 GJK다.
- PR #1797 (A1, J'f scatter): 파일과 기능 모두 독립이다.

## 바로잡은 해석

- "impratio 10 + CG + float32의 되먹임 루프"라는 solver 쪽 해석은 틀렸다. 루프를 돌린 건 R2다. 로컬 이슈 문서 D2
  초안이 이 해석으로 쓰여 있어서 다시 써야 한다.
- `solver/same_input.py`에서 "Warp와 C가 둘 다 느린" 상태들은 모서리가 틀린 step 바로 다음 step이었다. warmstart가
  틀린 문제의 답이라 C도 느렸다.
- R2를 "정확한 최대 넓이를 빠르고 robust하게"로 풀려던 것은 local minimum이었다. 최대 넓이 자체가 대칭에서 ill-posed다.

## 남은 것

- 정적 sweep의 누락 5건(box 1, plate끼리 4). 원인 미확인.
- EPA 루프에서 새 면의 넓이가 0이면 EPA가 contact 없이 끝난다. 남아 있는 누락 경로다.
- RFC 문서는 1차 설계 기준이다. 평면 표현과 R2를 반영해 다시 써야 한다.
- upstream 제출 계획: R1/R1'과 R2를 별도 PR로 낼지, C에도 port할지, #1798 후속 코멘트.
- 관련 관찰, 미확인: C의 pyramidal cone 흔들림(9.6–11.8 iteration), C primitive box-box의 높은 iteration.

## 파일

- `bench/`: 정적 sweep, red test 상태 탐색, CCD 출구 분류, stack/aloha 성능, kernel 자원, 4점 선택 비교. `run.sh <label>`로
  한꺼번에 돌리고 `results/<label>/`에 쌓는다.
- `solver/`: 같은 step 입력을 Warp와 C에 넣는 비교(`same_input.py`), iteration별 궤적(`trajectory.py`), 상태
  저장(`capture.py`, `states3.npz`), 모서리 집합 검사(`wrong_corners.py`, `corner_sequence.py`). C의 cost 함수를
  oracle로 쓴다(`common.py`).
- `issues/`: 관련 이슈 재실행 스크립트와 결과.

실행: `PYTHONPATH=<worktree>:prototype/bench uv run python prototype/bench/drop_rate.py`. `prototype/solver`도 같은 방식이다.
