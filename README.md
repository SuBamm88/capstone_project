# capstone-project

ROS 2 Humble collaboration workspace for Ubuntu 22.04 using Docker.

## Repository layout

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

## Quick start

```bash
git clone <your-repo-url>
cd capstone-project
./docker_build.sh
source docker/docker_run.sh
```

Inside the container:

```bash
cd /workspace
rosdep install --rosdistro humble --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Team workflow

1. Never push directly to `main`.
2. Create a feature branch such as `feat/planning-node`.
3. Open a pull request and merge only after review.
4. Keep generated ROS build outputs out of Git.

See [docs/collaboration.md](docs/collaboration.md) for the full workflow and GitHub branch protection setup.
