# capstone_project

ROS 2 Humble 기반 캡스톤 프로젝트 협업 워크스페이스입니다.

## 저장소 구조

이 저장소는 ROS 2 워크스페이스 구조를 따릅니다.

```text
capstone_project/
├── .github/
│   ├── PULL_REQUEST_TEMPLATE/
│   │   └── default.md
│   └── workflows/
│       └── colcon-build.yml
├── docker/
│   ├── Dockerfile
│   └── docker_run.sh
├── docs/
│   └── collaboration.md
├── src/
│   └── README.md
├── .dockerignore
├── .gitignore
├── docker-compose.yml
└── docker_build.sh
```

## 빠른 시작

### 환경
- Ubuntu 22.04
- ROS 2 Humble

### 의존성 설치
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 빌드
```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 팀 협업 규칙

1. `main` 브랜치에서는 절대 작업하지 않는다.
2. 작업은 항상 feature 브랜치에서 한다.
3. push 후 PR을 올린다.
4. push 전에 `colcon build`를 확인한다.

## Docker 사용 방침

이 프로젝트에서는 Docker 사용을 보류하기로 결정했습니다.

현재 팀원 모두가 같은 운영체제와 ROS 환경을 사용하고 있고, 기준 환경은 Ubuntu 22.04 + ROS 2 Humble입니다. 의존성은 `package.xml`, `setup.py`에 정확히 작성해서 커밋하고 push하면 각자 로컬 환경에서도 동일하게 동작할 가능성이 높습니다.

따라서 당분간은 Docker보다 로컬 Ubuntu 22.04 + ROS 2 Humble 환경에서 개발하고, 새 의존성을 추가할 때는 관련 설정 파일을 반드시 함께 수정합니다.

자세한 협업 절차와 GitHub 브랜치 보호 설정은 [docs/collaboration.md](docs/collaboration.md)를 참고하세요.
