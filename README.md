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

### 4. VS Code에서 프로젝트 열기

현재 디렉토리를 VS Code로 엽니다.

```bash
code .
```

이후 VS Code에서 터미널을 열고 아래 작업을 진행합니다.

### 5. main 최신화 후 작업 브랜치 만들기

작업을 시작하기 전에 항상 `main` 브랜치로 이동합니다.

```bash
git switch main
```

원격 저장소의 최신 내용을 내 컴퓨터로 가져옵니다.

```bash
git pull origin main
```

pull을 받은 뒤에는 다른 팀원이 새 의존성을 추가했는지 가볍게 확인합니다. `package.xml`, `setup.py`, `requirements.txt` 등이 바뀌었다면 이전 작업자의 README, PR 설명, 커밋 메시지를 참고해서 로컬 환경에도 반영합니다.

의존성 설치는 보통 세 종류가 있습니다. `rosdep`은 ROS 패키지 의존성을 맞출 때, `pip`는 Python 패키지를 설치할 때, `apt`는 Ubuntu 시스템 패키지를 설치할 때 사용합니다. 어떤 것을 써야 할지 애매하면 변경된 파일과 에러 메시지를 함께 보고 AI의 도움을 받아도 됩니다.

`main`에서 직접 작업하지 않고, 내 작업용 브랜치를 새로 만듭니다.

```bash
git checkout -b test/seunghyun
```

브랜치가 잘 바뀌었는지 확인합니다.

```bash
git branch
```

현재 브랜치 이름 앞에 `*` 표시가 있으면 그 브랜치에서 작업 중이라는 뜻입니다.

### 6. 파일 수정 또는 추가

VS Code에서 파일을 수정하거나 새 파일을 추가합니다.

Git 연결 확인 연습이라면 `src/seunghyun_git_test.txt`처럼 본인 이름이 들어간 테스트 파일을 하나 만들어도 됩니다.

Git이 어떤 파일 변경을 감지했는지 확인합니다.

```bash
git status
```

새로 만든 파일이 `Untracked files`에 보이면 아직 Git에 추가되지 않은 상태입니다.

### 7. colcon build 확인

push 하기 전에는 반드시 프로젝트가 정상적으로 빌드되는지 확인합니다.

Terminator에서 프로젝트 디렉토리로 이동한 뒤 아래 명령어를 실행합니다.

```bash
colcon build
source install/setup.bash
```

`colcon build`는 push 전에 필수로 확인합니다. 패키지 구조, `package.xml`, `setup.py`, launch 파일, 노드 코드를 수정했다면 특히 빌드 성공 여부를 꼭 확인해야 합니다.

### 8. commit

VS Code의 Source Control 탭에서 변경된 파일을 확인하고 commit합니다.

commit은 현재 변경 내용을 하나의 기록으로 남기는 작업입니다. 메시지는 `test: verify git connection`처럼 무엇을 했는지 짧게 적습니다.

### 9. GitHub에 push

내 작업 브랜치를 GitHub에 올립니다.

```bash
git push origin test/seunghyun
```

GitHub 저장소 페이지에서 `test/seunghyun` 브랜치가 보이면 push가 성공한 것입니다.

push는 VS Code Source Control의 `Sync Changes` 또는 `Push` 버튼으로 해도 됩니다. 터미널 명령어가 익숙하지 않다면 VS Code 도구를 활용해도 괜찮습니다.

### 10. Pull Request 작성

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
