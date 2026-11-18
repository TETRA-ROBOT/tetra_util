# tetra_relocalization

AMCL 이 위치를 잃으면 라이다 스캔을 지도에 맞춰 찾은 자세로 `/initialpose` 를 보내 스스로 바로잡는다.
AMCL 을 쓰는 모든 모델(M, H, EDU, MH, S, SM)의 `tetra_navigation_*.launch.py` 가 `tetra_navigation2/launch/auto_relocalization_launch.py` 로 띄운다
(`auto_relocalize:=false` 로 끈다). `scan_topic` 은 AMCL 과 같아야 한다 (H·MH 는 `scan_merge`). 3D(rtabmap)·지도 없는 주행에는 AMCL 이 없어 띄우지 않는다.

## 왜 AMCL 의 복구 기능을 쓰지 않나

AMCL 의 무작위 파티클 주입(`recovery_alpha_fast/slow`)은 사람이 지나가 점수가 잠깐 떨어져도 지도 곳곳에 파티클을 뿌려
위치를 몇 m 씩 튀게 했다 (EDU 2026-11-10). 그래서 끄고, 이 노드가 '확실히 더 잘 맞고 헷갈리는 후보가 없는' 자세일 때만 바로잡는다.

## 동작

1. `check_period`(1 s)마다 지금 자세에서 스캔 점이 지도 벽 `hit_tolerance`(10 cm) 안에 떨어지는 비율을 재 `~/scan_match_ratio` 로 낸다.
2. 비율이 `lost_ratio`(0.45) 밑으로 `lost_duration`(3 s) 넘게 이어지면 근처(±1 m, ±30°)를, 못 찾으면 넓게(±3 m, 전 방향) 찾는다.
3. 찾은 자세가 아래를 모두 만족할 때만 `/initialpose`(표준편차 0.1 m, 3°)를 보내고 `request_nomotion_update` 를 3 번 부른다.
   - 일치율 `accept_ratio`(0.70) 이상
   - 지금 자세보다 `min_gain`(0.25) 이상 높음: 지도와 현장이 달라진 곳에서 헛보정하지 않는다
   - 0.5 m·15° 넘게 떨어진 다른 후보보다 `min_margin`(0.10) 이상 높음: 특징 없는 복도에서는 바로잡지 않는다
4. 결과는 `~/event` 에 JSON 으로 낸다 (`decision`: corrected, no_match, no_gain, ambiguous, already_aligned).

- AMCL 이 active 일 때만 동작한다. 맵핑으로 바꿀 때 관제는 Nav2 를 lifecycle 로 멈출 뿐이라 이 노드는 떠 있지만 쉰다.
- `/initialpose` 가 오면(웹 위치 지정 포함) `hold_after_initial_pose`(10 s) 동안 쉰다.
- 빨리 돌거나 달릴 때(`max_angular_speed` 0.4 rad/s, `max_linear_speed` 0.8 m/s)는 스캔이 휘므로 재지 않는다.
- `apply:=false` 면 찾기만 하고 바로잡지 않는다 (현장 시험용).

## 실측 (M, 2026-11-19. 다른 모델은 아직 실기 확인 전)

42 시간 운행 뒤 AMCL 자세가 0.25 m·10~12° 틀어져 일치율 23% 였다. 노드가 일치율 89% 자세를 찾아(다른 후보 46%) 바로잡았고,
AMCL 표준편차가 2.1 m·59° 에서 8 cm·2.3° 로, 일치율이 88% 로 돌아왔다. 맞는 자세에서 강제로 찾게 하면 `no_gain` 으로 손대지 않았다.

## 테스트

계산(`scan_matcher.py`, `policy.py`)은 ROS 없이 `python3 -m pytest test` 로 확인한다.
