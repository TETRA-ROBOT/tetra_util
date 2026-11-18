# tetra_util - AI Agent Context

## 개요
자동 충전 도킹(Auto Docking) 및 로봇 운영 보조 유틸리티 스크립트를 관리하는 패키지입니다.

## 서브 패키지 맵

| 서브 패키지 | 역할 | 주요 통신 (Action/Service) |
|---|---|---|
| `tetra_docking` | OpenNav 기반 2단계 자동 충전 도킹 (AR 태그/레이저 라인 감지 -> 충전 패드 진입) | Action: `/docking_control`<br>Topic: `/docking_status` |
| `tetra_camera_calibration` | 웹관제 관리자 화면의 카메라 캘리브레이션 계산 노드 (체커보드 → set_camera_info). 화면을 열 때만 API 가 띄운다 | 로컬 HTTP 127.0.0.1:8091, Service client: `/camera/set_camera_info` |
| `etc` | 맵핑, 데이터 변환 및 레거시 유틸리티 스크립트 | Bash/Python 스크립트 |

## 도킹 시퀀스 다이어그램

```
  [ 도킹 시작 요청 (Action) ]
              │
              ▼
  [ 1단계: staging 자세로 접근 ] (Nav2 자율주행)
     tag_staging 이 켜져 있고 태그가 보이면 검출 평균으로 구한 태그 정면 축 위 지점,
     아니면 목표 도크 자세 + staging_x_offset (src/tag_staging.cpp)
              │
              ▼
  [ 2단계: AR 마커/라인 정밀 정렬 ] (tetra_docking 플러그인)
     heading_priority_near/far 가 켜져 있으면 도크 바로 앞에서는 도크와 나란히 곧게 들어간다 (src/approach_shaping.cpp)
     bumper_topic 이 켜져 있으면 접점 없이 앞 범퍼가 bumper_grace 넘게 눌릴 때 905 로 멈춘다 (src/bumper_guard.cpp)
     dock_offset_x/_y/_yaw (관제 관리자 설정 '도킹 영점', 도킹 직전에 API 가 넣음) 로 태그 기준 도크 자세를 로봇마다 옮긴다 (src/dock_offset.cpp)
     접근 속도는 플러그인이 아니라 docking_server 의 opennav_docking Controller(graceful)가 정한다. 관제가 도킹 목표 직전에 controller.v_linear_max /
     v_linear_min / slowdown_radius 를 관리자 설정 '도킹 속도'·'감속 시작 거리'로 넣어 도크 앞에서 남은 거리에 비례해 줄인다 (TetraSingleManagement dock-speed.ts).
     플러그인에서 도킹 중에 이 파라미터를 바꾸면 안 된다: Jazzy 도킹 서버는 도킹 내내 서버 mutex 를 잡고 파라미터 콜백도 그 mutex 를 기다려 도킹이 멈춘다 (2026-11-18 확인).
     태그를 놓친 뒤 마지막 자세로 들어가는 구간도 같은 제어기·같은 거리라 속도가 끊기지 않는다
     docked_by_depth 가 켜져 있으면 도크 축 깊이만으로 도착을 판정한다 (좌우 잔여 오차로 판정 폭을 못 맞추는 것 방지)
     undock_bumper_mask 가 켜져 있으면 언도킹 후진 중 그 범퍼 비트가 눌리면 멈춘다 (MH 뒤 범퍼 0x08)
     lock_dock_yaw_to_staging 이 켜져 있으면 접근 중에는 태그 위치만 쓰고 yaw 는 staging 때 멈춰서 평균한 값으로 고정한다 (src/staging_yaw_lock.cpp, M)
     tag_staging_frame 을 odom 으로 두면 staging 을 odom 기준으로 낸다 (map 은 AMCL 보정에 흔들려 태그 축에서 벗어났다, M)
     tag_search_angle 이 0 보다 크면 staging 에서 태그가 안 보일 때 제자리에서 천천히 좌우로 돌며 찾고, 영상 중앙에 맞춘 뒤
     멈춘 채로 확인한다 (src/tag_search.cpp, M). 지나가며 스친 검출은 확인으로 보지 않는다
     매 주기의 순서(가운데 맞추기 → 멈춰서 확인 → 못 보면 탐색 → 정밀 접근)는 src/refined_pose.cpp refineFromTagDetection 에 있다.
     탐색도 yaw 고정도 끈 모델(EDU, MH, SM)은 확인 없이 바로 접근한다 (TagSearchState::needsConfirmation, 테스트로 고정)
     후진 도킹(M, staging_yaw_offset pi): bumper_mask 8(뒤)·undock_bumper_mask 1(앞), tag staging 의 축 판정·태그 시야 계산도 후진을 따른다
              │
              ▼
  [ 3단계: 충전 패드 접촉 및 충전 전류 감지 ] (tetra_power_driver)
```

## tetra_docking 소스와 테스트

- 계산만 하는 코드는 ROS 없이 테스트한다: `src/tag_search_plan.cpp`(탐색 단계·가운데 맞추기·탐색 상태),
  `include/tetra_docking/dock_geometry.hpp`(태그 → 도크 오프셋 회전), `src/tag_view.cpp`(영상 투영·표본 평균).
  `colcon test --packages-select tetra_docking` (`test/`, ament_cmake_gtest).
- 로봇에 명령을 내거나 TF 를 읽는 쪽은 `tag_search.cpp`, `refined_pose.cpp`, `tag_dock_estimate.cpp`, `tag_staging.cpp` 다.
  새 계산을 넣을 때는 위 순수 함수 쪽에 두고 테스트를 붙인다.
- 각도 차이는 `angles::normalize_angle` 로 구한다 (파일마다 따로 만들지 않는다).
