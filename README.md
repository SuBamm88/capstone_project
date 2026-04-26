# capstone-project

ROS 2 Humble Docker 기반 협업 워크스페이스입니다.

ROS 2 Humble collaboration workspace using Docker.

## 저장소 구조

## Repository Layout

이 저장소는 `mingminQ/erp42_ros`와 유사한 단순한 ROS 워크스페이스 구조를 따릅니다.

This repository follows a simple ROS workspace layout similar to `mingminQ/erp42_ros`.

```text
capstone-project/
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

## Quick Start

```bash
git clone <your-repo-url>
cd capstone-project
./docker_build.sh
source docker/docker_run.sh
```

컨테이너 내부에서는 아래 순서로 작업합니다.

Inside the container, run the following:

```bash
cd /workspace
rosdep install --rosdistro humble --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## 팀 협업 규칙

## Team Workflow

1. `main` 브랜치에 직접 push하지 않습니다.
2. `feat/planning-node` 같은 작업 브랜치를 생성합니다.
3. Pull request를 열고 리뷰 후에만 merge합니다.
4. ROS 빌드 산출물과 생성 파일은 Git에 올리지 않습니다.

1. Never push directly to `main`.
2. Create a feature branch such as `feat/planning-node`.
3. Open a pull request and merge only after review.
4. Keep generated ROS build outputs out of Git.

자세한 협업 절차와 GitHub 브랜치 보호 설정은 [docs/collaboration.md](docs/collaboration.md)를 참고하세요.

See [docs/collaboration.md](docs/collaboration.md) for the full workflow and GitHub branch protection setup.
