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
     docked_by_depth 가 켜져 있으면 도크 축 깊이만으로 도착을 판정한다 (좌우 잔여 오차로 판정 폭을 못 맞추는 것 방지)
     undock_bumper_mask 가 켜져 있으면 언도킹 후진 중 그 범퍼 비트가 눌리면 멈춘다 (MH 뒤 범퍼 0x08)
     lock_dock_yaw_to_staging 이 켜져 있으면 접근 중에는 태그 위치만 쓰고 yaw 는 staging 때 멈춰서 평균한 값으로 고정한다 (src/staging_yaw_lock.cpp, M)
     tag_staging_frame 을 odom 으로 두면 staging 을 odom 기준으로 낸다 (map 은 AMCL 보정에 흔들려 태그 축에서 벗어났다, M)
     tag_search_angle 이 0 보다 크면 staging 에서 태그가 안 보일 때 제자리에서 천천히 좌우로 돌며 찾고, 영상 중앙에 맞춘 뒤
     멈춘 채로 확인한다 (src/tag_search.cpp, M). 지나가며 스친 검출은 확인으로 보지 않는다
     후진 도킹(M, staging_yaw_offset pi): bumper_mask 8(뒤)·undock_bumper_mask 1(앞), tag staging 의 축 판정·태그 시야 계산도 후진을 따른다
              │
              ▼
  [ 3단계: 충전 패드 접촉 및 충전 전류 감지 ] (tetra_power_driver)
```
