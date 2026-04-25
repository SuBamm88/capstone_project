# Collaboration guide

## What branch protection means

Protecting `main` means GitHub treats `main` as the stable branch.

- Direct pushes to `main` are blocked.
- Code reaches `main` through pull requests.
- You can require at least one review before merge.
- You can require the CI build to pass before merge.

This prevents one person from accidentally breaking the shared environment.

## Recommended workflow

1. Clone the repository.
2. Create a branch from `main`.
3. Modify only the files for your task.
4. Push your branch.
5. Open a pull request into `main`.
6. Merge after review and CI pass.

Example:

```bash
git checkout main
git pull origin main
git checkout -b feat/localization
git add .
git commit -m "Add localization node"
git push origin feat/localization
```

## GitHub settings to apply

Open the repository on GitHub and go to:

`Settings -> Branches -> Add branch protection rule`

Recommended rule for `main`:

- Branch name pattern: `main`
- Enable `Require a pull request before merging`
- Enable `Require approvals` and set it to `1`
- Enable `Require status checks to pass before merging`
- Select the `colcon-build` workflow
- Enable `Restrict who can push to matching branches` if needed

## Role split suggestion

- `src/perception_pkg`: perception owner
- `src/planning_pkg`: planning owner
- `src/control_pkg`: control owner
- `docker/` and CI: one maintainer reviews shared environment changes

Changes to `docker/`, workflow files, and root-level scripts should be reviewed carefully because they affect everyone.
