# Prototype notes: aligned stacks that never settle (#1798)

develop 브랜치다. 여러 해결책을 한 브랜치에서 실험한다. upstream PR로 낼 때는 원인별로 나눠서 새 브랜치에 정리한다.
`prototype/`은 PR에 넣지 않는다.

base: main `c384ac5`. 2026-10-18 기준.

## 결론

#1798 (exactly aligned convex-mesh stack이 settle하지 않음)의 원인은 전부 collision 쪽이다. solver에는 결함이 없다.
세 원인 모두 같은 패턴이다. 정렬된 기하에서 exact arithmetic이면 생기는 tie (대칭 평면, 겹치는 면, 일직선상의 점)에
float32 반올림이 끼어들고, 엄격한 비교를 하는 알고리즘이 그 잡음을 신호로 착각한다.

```
 GJK ──▶ EPA ─────────────────────────▶ multicontact ─────────────▶ solver
          ▲ R1: 관통 pair를 버림           ▲ R2: 4점 중 하나가 틀림
          ▲ R1': 깊이 0에서 기울어진 법선
```

| 원인 | 어디서 | 무슨 일 | 증상 | 상태 |
| --- | --- | --- | --- | --- |
| R1 | EPA 시작 도형 | 원점이 도형의 면/모서리 위에 있으면 "닿기만 함"으로 판정해 pair를 버림 | plate가 mm 단위로 꺼짐 (18.75 mm drift) | 수정 (`2310f42`) |
| R1' | EPA 면 가지치기 | 깊이 0일 때 진짜 답인 면이 "하한보다 가깝다"며 버려지고, 확인되지 않은 면이 contact로 나감 | 처음 닿을 때 법선이 최대 45° 기울어진 contact | 수정 (`2310f42`) |
| R2 | multicontact `_polygon_quad` | 넓이 최대 사각형을 고르는 hill climbing이 tie에서 잘못 이동함 | 모서리 중복이나 변 위의 점 → step의 24%에서 받침점이 틀림 → CG 20–50 iteration, 0.1 mm 흔들림 | 전수 탐색으로 확인 (실험용) |

## R1, R1': EPA 면을 평면으로 표현

EPA는 면을 "원점을 그 면에 투영한 벡터"로 저장하고, 탐색 방향도 그 벡터에서 얻었다. 원점이 면 위에 있으면 그
벡터가 0이 된다. 그래서 시작 도형 단계가 절대 임계값(`MIN_DIST2/3/4`, #1532 fallback)으로 그런 면을 거부했고,
그 pair는 관통 아님으로 처리됐다.

수정 후에는 면을 바깥 단위 법선과 원점까지의 부호 있는 거리로 저장한다. 둘 다 면의 꼭짓점에서만 나오므로,
support 방향, 상한, horizon 판정이 원점 위치와 무관해진다. 원점은 면이나 모서리 위에 있어도 되고, 반올림으로
살짝 바깥에 있어도 된다. 시작 도형은 넓이 0인 면만 거부한다.

새 면은 상한보다 먼 것만 버린다. 하한보다 가까운 면은 반올림 때문에만 생기고, 그때는 원점이 경계 위에 있는
경우라 그 면이 답이다 (R1').

지나온 단계:
- 1차 (`bench/proto1.diff`): 시작 도형에서 원점이 면 위면 support 1회로 경계 면인지 확인. 면 하나는 풀었지만,
  원점이 모서리 위인 경우 (면 두 개)를 못 풀었다. 1차는 3차의 특수한 경우다. EPA 한 iteration을 시작 도형의
  면 하나에서만 손으로 돌린 셈이다.
- 2차: 면 = 평면 표현. 정적 sweep은 모두 통과했지만, 깊이 0에서 기울어진 법선이 나와 6–8장 stack이 main보다
  나빠졌다 (R1').
- 3차: 가지치기 조건 수정. 현재 커밋.

## R2: multicontact 사각형 선택

면-면 contact는 한 면을 다른 면의 변들로 잘라 만든 다각형에서, 꼭짓점이 5개 이상이면 넓이 최대 사각형의 네
점을 고른다. 두 plate가 1e-4 rad 정도 기울면 변끼리 얕게 교차해서, 다각형에 변 위에 놓인 여분의 꼭짓점이
생긴다. exact arithmetic에서는 그 점을 넣든 빼든 넓이가 같다 (tie). float32에서는 그 차이가 반올림 잡음이고,
"넓이가 엄격히 늘 때만 한 점씩 옮기는" hill climbing이 잡음 쪽으로 걸어간다. 관찰된 결과는 이렇다.

- 같은 index를 두 번 고름: `pick=4 4 0 2` → 모서리 (105, 148) 두 번, (105, −148) 빠짐
- 넓이 최대가 아닌 사각형에서 멈춤: `pick=0 1 2 3` → 윗변 위의 점 (38.2, 148)을 고르고 모서리 (105, 148)을 버림

C의 `hull4`도 같은 알고리즘이다. main에서도 같은 상태에서 같은 오류가 난다 (`solver/wrong_corners.py`, 30개
상태 중 8개). 그러니 R1과 독립이다.

되먹임은 이렇게 생긴다. 모서리가 틀린 step이 생기면 받침점 불균형으로 torque가 생기고, 다음 step에 정상으로
돌아오면서 contact 집합이 바뀌어 warmstart가 맞지 않는다. CG가 덜 수렴하면 plate가 다시 기울고, 다시 여분
꼭짓점이 생긴다. Newton이나 impratio 1은 한 step에 완전히 수렴해서 plate가 기울지 않으므로 이 버그를 가린다.

실험용 수정은 a < b < c < d인 모든 조합 중 넓이 최대를 고르는 전수 탐색이다. O(n⁴)이고, n은 최대 20 (mesh)에서
32 (cylinder)다. 빠르면서 tie에 강한 알고리즘이 필요하다.

## 결과

정적 sweep (`bench/drop_rate.py`, 정렬 근처, world 4096):

| | main | 3차 |
| --- | --- | --- |
| 1–10 µm 누락 (box / mesh / plate끼리) | 55% / 50% / 56% | 0% / 0% / 0.1% |
| 10 µm–10 mm 누락 (최대) | 13% | 0.02% |
| 과대 contact (C보다 1 µm 넘게 깊음) | 0% | 0% |
| C 대비 깊이 오차 | ≤ 2.4e-9 m | 같음 |

drift grid (`D-work/grid.py`, mesh/box × 2·4·8 mm × 2–8장, 42개 config):

| | main | 3차 | 3차 + R2 전수 탐색 | MuJoCo C |
| --- | --- | --- | --- | --- |
| CG iteration 평균 (최대 config) | 25.9 | 23.3 | 2.1 | 2.5 |
| 최대 drift | 18.75 mm | 0.099 mm | 0.007 mm | 0.001 mm |
| iteration cap 도달 (config당 최대) | 37 | 1 | 0 | 0 |

solver 옵션 실험 (`bench/solver_exp.py`, 4 mm mesh 3/4/5장): R2 수정 후에는 CG, impratio 10, tol 1e-6 기준이
1.4 / 1.7 / 2.1로 C (1.3 / 1.8 / 2.0)와 같다. impratio, tolerance, Newton, cone을 바꿔도 모두 0.8–2.1이다.

성능 (3차, nworld 4096): aloha_pot과 aloha_clutter는 변화 없음 (narrowphase ±1%). 정렬된 stack은 17–21% 빨라졌다
(solve −20%, narrowphase +15–24%). CCD kernel은 register 128로 같고 spill도 비슷하다. R2 전수 탐색의 비용은
아직 재지 않았다.

test: red test 10개 (`collision_gjk_test.py`의 `*_aligned_*`, `*_settling_*`)가 main에서 7개 실패하고 3차에서 모두
통과한다. `collision_gjk_test.py`와 `collision_driver_test.py`는 250개 모두 통과한다.

## 바로잡은 해석

- "impratio 10 + CG + float32의 되먹임 루프"라는 solver 쪽 해석은 틀렸다. 루프를 돌린 건 R2다. contact가
  맞으면 Warp CG는 C와 같은 속도로 수렴한다. 로컬 이슈 문서 D2 초안은 이 해석으로 쓰여 있어서 다시 써야 한다.
- `solver/same_input.py`에서 "Warp와 C가 둘 다 느린" 상태들은 모서리가 틀린 step 바로 다음 step이었다. warmstart가
  틀린 문제의 답이라 C도 느렸다.

## 남은 것

- R2: 빠르고 tie에 강한 사각형 선택 알고리즘. 전수 탐색의 실제 비용 측정. 가능하면 C에도 port한다.
- 정적 sweep에서 남은 누락 5건 (box 1, plate끼리 4). 원인 미확인.
- EPA 루프에서 새 면의 넓이가 0이면 EPA가 contact 없이 끝난다. 남아 있는 누락 경로다.
- 처음 닿는 순간 (깊이 0, R1')을 고정하는 red test가 없다.
- RFC 문서는 1차 설계 기준이다. 평면 표현과 R2를 반영해 다시 써야 한다.
- 관련 관찰, 미확인: C의 pyramidal cone에서도 흔들림 (9.6–11.8 iteration). primitive box-box collider의 높은
  iteration. #1510 (welded box stack 흔들림, closed)이 R2였는지.

## 파일

- `bench/`: 정적 sweep, red test 상태 탐색, CCD 출구 분류, stack/aloha 성능, kernel 자원. `run.sh <label>`로 한 번에
  돌리고 `results/<label>/`에 쌓는다. `results/quad/`는 R2 전수 탐색을 넣은 run이다.
- `solver/`: 같은 step 입력을 Warp와 C에 넣는 비교 (`same_input.py`), iteration별 궤적 (`trajectory.py`), 상태 저장
  (`capture.py`, `states3.npz`), 모서리 집합 검사 (`wrong_corners.py`, `corner_sequence.py`). C의 cost 함수를
  oracle로 쓴다 (`common.py`).

실행: `PYTHONPATH=<worktree>:prototype/bench uv run python prototype/bench/drop_rate.py`. 같은 방식으로
`prototype/solver`도 실행한다.
