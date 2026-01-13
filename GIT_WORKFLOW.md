## Git workflow (push to GitHub, pull on laptop/Pi)

This repo is a **ROS 2 workspace** + **Docker files**.

- Your code + Docker files are tracked in git.
- External upstream source dependencies are tracked in `src/ros2.repos` and fetched with `vcs import`.
- Colcon build outputs (`build/`, `install/`, `log/`) are not tracked.

---

## One-time setup (laptop)

### 1) Clone from GitHub

```bash
git clone git@github.com:ashwanth18/ws_rhapsodi-promtek.git
cd ws_rhapsodi-promtek
```

### 2) (Optional) fetch external source deps into `src/`

If you want to build *without Docker* on the host, or you want the sources present locally:

```bash
vcs import src < src/ros2.repos
```

Note: For Docker builds, the `Dockerfile` already runs `vcs import` automatically during image build.

---

## Day-to-day workflow (laptop) — edit → commit → push

### 1) Update your local branch

```bash
git pull --rebase
```

### 2) Make changes (code, Docker files, docs)

### 3) Review what will be committed

```bash
git status
git diff
```

### 4) Commit and push

```bash
git add -A
git commit -m "Describe what changed"
git push
```

---

## Pull/update on the Raspberry Pi 5 (runtime machine)

### 1) Clone once

```bash
git clone git@github.com:ashwanth18/ws_rhapsodi-promtek.git
cd ws_rhapsodi-promtek
```

### 2) Update later

```bash
cd ws_rhapsodi-promtek
git pull --rebase
```

### 3) Rebuild the ROS image (if code/Docker changed)

```bash
docker build --build-arg COLCON_PARALLEL_WORKERS=1 -t rhapsodi-promtek:jazzy .
```

---

## Updating `src/ros2.repos` (external git dependencies)

### If you changed which external repos/versions you want

Best practice is to pin exact commits:

```bash
vcs export --exact src > src/ros2.repos
git add src/ros2.repos
git commit -m "Update ros2.repos pins"
git push
```

### If you just want to fetch them on a machine

```bash
vcs import src < src/ros2.repos
```

---

## Common gotchas

- **“Folders are empty on GitHub”**: that happens when you accidentally commit nested git repos as gitlinks/submodules.
  This repo avoids that by using `src/ros2.repos` instead.
- **Docker build is slow on Pi**: keep `COLCON_PARALLEL_WORKERS=1`.


