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
  [ 1단계: 스테이션 전방 1m 접근 ] (Nav2 자율주행)
              │
              ▼
  [ 2단계: AR 마커/라인 정밀 정렬 ] (tetra_docking 플러그인)
              │
              ▼
  [ 3단계: 충전 패드 접촉 및 충전 전류 감지 ] (tetra_power_driver)
```
