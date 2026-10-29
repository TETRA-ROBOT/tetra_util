# tetra_util - AI Agent Context

## 개요
자동 충전 도킹(Auto Docking) 및 로봇 운영 보조 유틸리티 스크립트를 관리하는 패키지입니다.

## 서브 패키지 맵

| 서브 패키지 | 역할 | 주요 통신 (Action/Service) |
|---|---|---|
| `tetra_docking` | OpenNav 기반 2단계 자동 충전 도킹 (AR 태그/레이저 라인 감지 -> 충전 패드 진입) | Action: `/docking_control`<br>Topic: `/docking_status` |
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
              │
              ▼
  [ 3단계: 충전 패드 접촉 및 충전 전류 감지 ] (tetra_power_driver)
```
