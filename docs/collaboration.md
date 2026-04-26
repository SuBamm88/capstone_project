# Collaboration Guide

## 현재 저장소 상태

## Current Repository Status

- 로컬 저장소 경로: `~/capstone-project`
- 연결된 GitHub 저장소: `git@github.com:SuBamm88/capstone_project.git`
- 기본 브랜치: `main`
- 현재 로컬 추적 브랜치: `main -> origin/main`

- Local repository path: `~/capstone-project`
- Connected GitHub repository: `git@github.com:SuBamm88/capstone_project.git`
- Default branch: `main`
- Current local branch tracking: `main -> origin/main`

## 지금 가장 먼저 할 일: 팀원 초대

## First Thing To Do Now: Invite Teammates

팀 전체가 하나의 SSH key를 공유하면 안 됩니다. 각 팀원은 자신의 GitHub 계정과 자신의 SSH key를 사용해야 합니다.

Do not share one SSH key across the team. Each teammate should use their own GitHub account and their own SSH key.

GitHub에서 아래 경로로 이동하세요.

Open GitHub and go to:

`Repository -> Settings -> Collaborators -> Add people`

권장 절차는 다음과 같습니다.

Recommended approach:

1. 각 팀원을 GitHub username으로 초대합니다.
2. 각 팀원이 이메일 또는 GitHub에서 초대를 수락합니다.
3. 초대 수락 후 각자 자신의 SSH 설정으로 저장소를 clone합니다.

1. Add each teammate by their GitHub username.
2. Ask them to accept the invitation by email or on GitHub.
3. After acceptance, each teammate clones with their own SSH setup.

## SSH 연결 확인

## SSH Connection Check

각 팀원은 아래 명령으로 SSH 연결을 확인해야 합니다.

Each teammate should run the following command to verify SSH access:

```bash
ssh -T git@github.com
```

정상 결과는 다음과 같습니다.

Expected result:

- GitHub에서 인증 성공과 유사한 메시지가 표시됩니다.
- shell access is not provided 같은 문구가 보여도 정상입니다.

- GitHub shows a message similar to successful authentication.
- The shell may say shell access is not provided. That is normal.

아직 SSH key가 없는 팀원은 아래처럼 생성합니다.

If a teammate does not have an SSH key yet, generate one with:

```bash
ssh-keygen -t ed25519 -C "their_email@example.com"
cat ~/.ssh/id_ed25519.pub
```

출력된 공개 키는 아래 경로에 등록합니다.

Add the printed public key here:

`GitHub -> Settings -> SSH and GPG keys -> New SSH key`

## Remote origin 예시

## Remote Origin Example

현재 remote 주소는 아래 명령으로 확인할 수 있습니다.

Check the current remote with:

```bash
git remote -v
```

이 저장소에서 기대되는 origin은 아래와 같습니다.

Expected origin for this repository:

```bash
origin  git@github.com:SuBamm88/capstone_project.git (fetch)
origin  git@github.com:SuBamm88/capstone_project.git (push)
```

누군가 HTTPS로 clone했다면 SSH로 아래처럼 변경할 수 있습니다.

If someone cloned with HTTPS, switch to SSH with:

```bash
git remote set-url origin git@github.com:SuBamm88/capstone_project.git
```

## 브랜치 보호가 의미하는 것

## What Branch Protection Means

`main` 브랜치를 보호한다는 것은 GitHub가 `main`을 안정 브랜치로 취급하도록 설정하는 것입니다.

Protecting `main` means GitHub treats `main` as the stable branch.

- `main`에 대한 직접 push를 막을 수 있습니다.
- 코드는 pull request를 통해서만 `main`에 들어가게 할 수 있습니다.
- merge 전에 최소 1명 이상의 리뷰를 강제할 수 있습니다.
- merge 전에 CI 통과를 강제할 수 있습니다.

- Direct pushes to `main` are blocked.
- Code reaches `main` through pull requests.
- You can require at least one review before merge.
- You can require the CI build to pass before merge.

이 설정은 한 사람이 실수로 공용 환경을 깨뜨리는 일을 줄여줍니다.

This prevents one person from accidentally breaking the shared environment.

## 권장 작업 흐름

## Recommended Workflow

1. 저장소를 clone합니다.
2. `main`에서 새 브랜치를 만듭니다.
3. 자신의 작업 범위만 수정합니다.
4. 작업 브랜치를 push합니다.
5. `main` 대상으로 pull request를 엽니다.
6. 리뷰와 CI 통과 후 merge합니다.

1. Clone the repository.
2. Create a branch from `main`.
3. Modify only the files for your task.
4. Push your branch.
5. Open a pull request into `main`.
6. Merge after review and CI pass.

예시는 아래와 같습니다.

Example:

```bash
git checkout main
git pull origin main
git checkout -b feat/localization
git add .
git commit -m "Add localization node"
git push origin feat/localization
```

## 브랜치 보호 설정 클릭 경로

## Branch Protection Click Path

GitHub 저장소에서 아래 경로로 이동합니다.

Open the repository on GitHub and go to:

`Settings -> Branches -> Add branch protection rule`

`main`에 대한 권장 설정은 아래와 같습니다.

Recommended rule for `main`:

- Branch name pattern: `main`
- `Require a pull request before merging` 활성화
- `Require approvals` 활성화 후 `1`로 설정
- `Require status checks to pass before merging` 활성화
- `colcon-build` 워크플로 선택
- 필요 시 `Restrict who can push to matching branches` 활성화

- Branch name pattern: `main`
- Enable `Require a pull request before merging`
- Enable `Require approvals` and set it to `1`
- Enable `Require status checks to pass before merging`
- Select the `colcon-build` workflow
- Enable `Restrict who can push to matching branches` if needed

만약 GitHub가 예전 branch protection UI 대신 rulesets UI를 보여주면 아래 경로를 사용합니다.

If GitHub shows the newer rulesets UI instead of the old branch protection UI, use:

`Settings -> Rules -> Rulesets -> New ruleset -> Branch ruleset`

권장 값은 다음과 같습니다.

Recommended values:

1. Ruleset name: `Protect main`
2. Enforcement status: `Active`
3. Target branches: `Include default branch` 또는 `main` 직접 지정
4. Pull request requirement 활성화
5. Approval 수를 `1`로 설정
6. Status checks를 활성화하고 build workflow 선택
7. `main` 직접 push 차단

1. Ruleset name: `Protect main`
2. Enforcement status: `Active`
3. Target branches: `Include default branch` or explicitly `main`
4. Enable pull request requirement
5. Require `1` approval
6. Require status checks and choose the build workflow
7. Block direct pushes to `main`

## 역할 분담 제안

## Role Split Suggestion

- `src/perception_pkg`: perception 담당
- `src/planning_pkg`: planning 담당
- `src/control_pkg`: control 담당
- `docker/`와 CI: 공용 환경 담당 maintainer 1명 이상

- `src/perception_pkg`: perception owner
- `src/planning_pkg`: planning owner
- `src/control_pkg`: control owner
- `docker/` and CI: one maintainer reviews shared environment changes

`docker/`, workflow 파일, 루트 스크립트는 모두에게 영향을 주므로 특히 신중하게 리뷰해야 합니다.

Changes to `docker/`, workflow files, and root-level scripts should be reviewed carefully because they affect everyone.

## 팀 규칙 템플릿

## Team Rules Template

아래 규칙을 기본 협업 규칙으로 사용하는 것을 권장합니다.

Use this as the baseline team agreement:

1. `main` 브랜치에 직접 push하지 않습니다.
2. 하나의 작업은 하나의 브랜치에서 진행합니다.
3. 브랜치 이름은 `feat/...`, `fix/...`, `docs/...`, `chore/...` 형식을 사용합니다.
4. 작은 변경이라도 반드시 pull request를 엽니다.
5. 최소 1명의 팀원 리뷰 후 merge합니다.
6. CI가 실패한 상태에서는 merge하지 않습니다.
7. `docker/`, `.github/workflows/`, 루트 스크립트 변경은 더 신중하게 리뷰합니다.
8. 새 작업 시작 전 `main`을 pull합니다.
9. 생성 파일과 로컬 빌드 산출물은 Git에 올리지 않습니다.
10. commit message는 실제 변경 내용을 설명하도록 작성합니다.

1. Never push directly to `main`.
2. One task, one branch.
3. Branch names follow `feat/...`, `fix/...`, `docs/...`, or `chore/...`.
4. Open a pull request for every change, even small ones.
5. At least one teammate reviews before merge.
6. Do not merge while CI is failing.
7. Shared files such as `docker/`, `.github/workflows/`, and root scripts need extra review.
8. Pull `main` before starting new work.
9. Keep generated files and local build outputs out of Git.
10. Write commit messages that explain the actual change.

권장 브랜치 이름 예시는 아래와 같습니다.

Suggested branch examples:

- `feat/planning-node`
- `fix/docker-permission`
- `docs/setup-guide`

하루 작업 흐름 예시는 아래와 같습니다.

Suggested daily flow:

```bash
git checkout main
git pull origin main
git checkout -b feat/your-task
git add .
git commit -m "Implement your task"
git push origin feat/your-task
```
