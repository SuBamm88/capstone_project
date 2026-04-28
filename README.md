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

처음 프로젝트에 참여하는 팀원은 아래 순서대로 진행합니다.

### 1. 기본 Git 설정

Git 커밋에 표시될 이름과 이메일을 설정합니다.

```bash
git config --global user.name "내이름"
git config --global user.email "내이메일@example.com"
```

현재 설정이 잘 들어갔는지 확인합니다.

```bash
git config --global --list
```

### 2. SSH 연결 설정 권장

GitHub 저장소를 사용할 때는 매번 비밀번호를 입력하지 않아도 되는 SSH 연결을 권장합니다.

SSH 키가 이미 있는지 확인합니다.

```bash
ls ~/.ssh
```

`id_ed25519.pub` 파일이 없다면 새 SSH 키를 생성합니다.

```bash
ssh-keygen -t ed25519 -C "내이메일@example.com"
```

생성된 공개 키 내용을 복사합니다.

```bash
cat ~/.ssh/id_ed25519.pub
```

복사한 키를 GitHub의 `Settings` -> `SSH and GPG keys` -> `New SSH key`에 등록합니다.

GitHub SSH 연결이 되는지 확인합니다.

```bash
ssh -T git@github.com
```

`successfully authenticated` 문구가 보이면 SSH 연결이 완료된 것입니다.

### 3. 저장소 clone

GitHub에 있는 프로젝트를 내 컴퓨터로 내려받습니다.

```bash
git clone git@github.com:SuBamm88/capstone_project.git
```

프로젝트 디렉토리로 이동합니다.

```bash
cd capstone_project
```

현재 위치와 파일 목록을 확인합니다.

```bash
pwd
ls
```

### 4. VS Code에서 프로젝트 열기

현재 디렉토리를 VS Code로 엽니다.

```bash
code .
```

이후 VS Code 터미널에서 아래 명령어를 실행합니다.

### 5. main 브랜치 최신 상태로 맞추기

작업을 시작하기 전에 항상 `main` 브랜치로 이동합니다.

```bash
git switch main
```

원격 저장소의 최신 내용을 내 컴퓨터로 가져옵니다.

```bash
git pull origin main
```

pull을 받은 뒤에는 다른 팀원이 새 의존성을 추가했는지 확인합니다.

```bash
git log --oneline -5
git status
```

최근 커밋 메시지나 변경된 파일을 보고 `package.xml`, `setup.py`, `requirements.txt` 같은 의존성 관련 파일이 바뀌었는지 확인합니다.

```bash
git diff --name-only HEAD~1 HEAD
```

의존성이 추가된 것 같다면 `colcon build` 전에 먼저 로컬 환경에 반영합니다. 보통은 팀원이 README, PR 본문, 커밋 메시지, 코드 주석 등에 적어둔 설치 방법을 확인하고 그대로 따라 하면 됩니다.

ROS 패키지 의존성은 아래 명령어로 확인하고 설치합니다.

```bash
rosdep install --rosdistro humble --from-paths src --ignore-src -r -y
```

그 외 `pip install`, `apt install` 같은 추가 작업이 필요해 보이면 다른 작업자가 남긴 설명을 먼저 확인합니다. 설명이 부족하거나 에러 메시지가 이해되지 않으면 AI에게 `package.xml`, `setup.py`, 에러 메시지를 함께 보여주고 어떤 의존성을 설치해야 하는지 도움을 받아도 됩니다.

### 6. 작업 브랜치 만들기

`main`에서 직접 작업하지 않고, 내 작업용 브랜치를 새로 만듭니다.

```bash
git checkout -b test/seunghyun
```

브랜치가 잘 바뀌었는지 확인합니다.

```bash
git branch
```

현재 브랜치 이름 앞에 `*` 표시가 있으면 그 브랜치에서 작업 중이라는 뜻입니다.

### 7. 테스트 파일 생성

Git 연결 확인용 파일을 하나 만듭니다.

```bash
touch src/seunghyun_git_test.txt
```

파일이 생겼는지 확인합니다.

```bash
ls src
```

### 8. 변경 사항 확인

Git이 어떤 파일 변경을 감지했는지 확인합니다.

```bash
git status
```

새로 만든 파일이 `Untracked files`에 보이면 아직 Git에 추가되지 않은 상태입니다.

### 9. colcon build 확인

push 하기 전에 프로젝트가 정상적으로 빌드되는지 확인합니다.

ROS 2 Humble 명령어를 사용할 수 있도록 현재 터미널에 ROS 환경을 불러옵니다.

```bash
source /opt/ros/humble/setup.bash
```

새 패키지를 추가했거나 의존성을 수정했다면 `colcon build` 전에 아래 명령어로 의존성을 먼저 확인합니다.

```bash
rosdep install --rosdistro humble --from-paths src --ignore-src -r -y
```

pull 이후 새 의존성이 들어온 경우에도 위 명령어와 팀원이 남긴 설치 안내를 먼저 적용한 다음 빌드합니다.

프로젝트를 빌드해서 코드와 패키지 설정에 문제가 없는지 확인합니다.

```bash
colcon build
```

`Summary`에 실패한 패키지가 없으면 빌드가 성공한 것입니다.

빌드 후 현재 터미널에서 새로 빌드된 환경을 사용하려면 아래 명령어를 실행합니다.

```bash
source install/setup.bash
```

`colcon build`는 최소한 push 전에 반드시 실행해야 합니다. 패키지 구조를 바꾸거나 `package.xml`, `setup.py`, launch 파일, 노드 코드를 수정한 경우에도 바로 실행해서 문제가 없는지 확인합니다.

### 10. 파일을 commit

변경한 파일을 Git이 관리하도록 추가합니다.

```bash
git add src/seunghyun_git_test.txt
```

커밋을 만들어 변경 내용을 기록합니다.

```bash
git commit -m "test: verify git connection"
```

### 11. GitHub에 push

내 작업 브랜치를 GitHub에 올립니다.

```bash
git push origin test/seunghyun
```

GitHub 저장소 페이지에서 `test/seunghyun` 브랜치가 보이면 push가 성공한 것입니다.

### 12. Pull Request 작성

GitHub 저장소 페이지로 이동합니다.

1. `Compare & pull request` 버튼을 누릅니다.
2. base 브랜치가 `main`인지 확인합니다.
3. compare 브랜치가 내가 push한 `test/seunghyun`인지 확인합니다.
4. PR 제목에는 무엇을 했는지 짧게 적습니다.
5. PR 본문에는 변경 내용, 빌드 확인 여부, 리뷰어가 봐야 할 내용을 적습니다.
6. `Create pull request`를 누릅니다.

PR 본문 예시는 아래처럼 작성합니다.

```markdown
## 변경 내용
- Git 연결 확인용 파일을 추가했습니다.

## 확인한 내용
- [x] colcon build 성공

## 참고 사항
- GitHub push와 PR 생성 연습용 PR입니다.
```

PR을 올린 뒤에는 팀원이 리뷰하고, 문제가 없을 때 `main`으로 merge합니다.

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
