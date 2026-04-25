#!/usr/bin/env bash

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

docker build -t capstone-project:humble -f "${PROJECT_ROOT}/docker/Dockerfile" "${PROJECT_ROOT}"
