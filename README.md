# tetra_util

TETRA 로봇의 자동 충전 도킹 및 운영에 필요한 유틸리티 스크립트와 설정 모음입니다.

---

## 1. 패키지 구성 (Packages Overview)

- **`tetra_docking`**: OpenNav Docking 프레임워크 기반 TETRA 로봇의 자동 충전 스테이션 도킹 플러그인 패키지
- **`etc`**: 로봇 맵핑, 맵 저장, 내비게이션 및 시작 스크립트(`sh/`), 이미지 자원 모음

---

## 2. 주요 유틸리티 및 스크립트 (`etc/sh`)

- `startup.sh`: 기본 로봇 환경 초기화 및 기동 스크립트
- `mapping.sh`: Cartographer SLAM 매핑 구동 스크립트
- `mapsave.sh`: 작성된 맵 저장 스크립트
- `navigation.sh`: Navigation2 자율주행 실행 스크립트
