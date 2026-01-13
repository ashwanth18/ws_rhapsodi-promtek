# Python Dependencies Installation Guide

This document explains how to install Python dependencies for this ROS 2 workspace, particularly when encountering the "externally-managed-environment" error.

## Overview

Some Python packages required by this workspace are not available via `apt` and must be installed using `pip`. Modern Python installations (Python 3.12+) protect the system environment, requiring special handling.

## Required Python Packages

The following packages are required:

- **pyyaml** - Usually already installed with ROS 2
- **roslibpy** - Required by `niryo_ned_ros2_driver` for ROS 1/ROS 2 bridge functionality

## Installation Methods

### Method 1: Using `--break-system-packages` (Recommended for ROS 2)

Since `roslibpy` is not available via `apt` and is required for ROS 2 nodes to function, you can install it using the `--break-system-packages` flag:

```bash
pip install --break-system-packages roslibpy
```

**Note**: This flag is sometimes necessary for ROS 2 development when packages aren't available through the system package manager. Use with caution, but it's generally safe for ROS 2 workspaces.

### Method 2: Check if Already Installed

First, verify if the packages are already installed:

```bash
python3 -c "import roslibpy; print('roslibpy: OK')" 2>&1 || echo "roslibpy: NOT INSTALLED"
python3 -c "import yaml; print('pyyaml: OK')" 2>&1 || echo "pyyaml: NOT INSTALLED"
```

### Method 3: Install from requirements.txt

If you prefer to install from the requirements file:

```bash
pip install --break-system-packages -r src/requirements.txt
```

This will install:
- `pyyaml` (if not already installed)
- `roslibpy`

## Safety of `--break-system-packages`

### Is it safe?

**Short answer**: It's **relatively safe for ROS 2 development**, but use it **selectively and carefully**.

### What are the risks?

1. **Version conflicts**: Installing packages globally can conflict with system packages managed by `apt`
2. **Breaking system tools**: Some system utilities depend on specific Python package versions
3. **Difficult to track**: Global installations make it harder to manage dependencies
4. **Update conflicts**: System updates via `apt` might conflict with pip-installed packages

### When is it acceptable?

✅ **Acceptable for:**
- ROS 2 development packages (like `roslibpy`) that aren't available via `apt`
- Packages that are clearly development dependencies, not system-critical
- Packages that don't conflict with system packages
- Single-user development machines (not production servers)

❌ **Avoid for:**
- System-critical packages (cryptography, SSL libraries, etc.)
- Packages that are available via `apt` (use `apt` instead)
- Production servers or shared systems
- Packages that might conflict with system tools

### Best Practices

1. **Install only what you need**: Don't install packages globally unless necessary
2. **Prefer `apt` when available**: Always check if a package is available via `apt` first
3. **Document installations**: Keep track of what you've installed globally
4. **Use `--user` flag when possible**: This installs to `~/.local/` instead of system-wide (though it may still trigger the error)
5. **Test after installation**: Verify system tools still work after installing packages

### For ROS 2 Specifically

For ROS 2 workspaces, using `--break-system-packages` is **acceptable** because:
1. ROS 2 manages its own Python environment and dependencies
2. The packages are needed for ROS 2 nodes to function
3. These are development dependencies, not system-critical packages
4. ROS 2 development typically happens on dedicated development machines

### Mitigation Strategies

If you're concerned about safety:

1. **Use a dedicated development machine**: Don't use your main production system
2. **Keep backups**: Regular system backups help if something breaks
3. **Monitor system updates**: Be careful when running `apt upgrade` after pip installations
4. **Use `pip list`**: Regularly check what's installed globally:
   ```bash
   pip list --break-system-packages
   ```
5. **Consider Docker**: For maximum isolation, use Docker containers for development

## Why `--break-system-packages` is Needed

Python 3.12+ implements PEP 668, which prevents installing packages directly into the system Python environment. This protects system stability, but ROS 2 development often requires packages that aren't available via `apt`.

## Package Dependencies

### niryo_ned_ros2_driver

The `niryo_ned_ros2_driver` package requires `roslibpy` for:
- ROS 1/ROS 2 bridge functionality
- Topic, service, and action bridging between ROS 1 and ROS 2

This dependency is declared in `src/niryo_ned_ros2_driver/setup.py`.

## Verification

After installation, verify the packages work:

```bash
# Test roslibpy
python3 -c "import roslibpy; print('roslibpy version:', roslibpy.__version__ if hasattr(roslibpy, '__version__') else 'installed')"

# Test pyyaml
python3 -c "import yaml; print('pyyaml version:', yaml.__version__)"
```

## Troubleshooting

### Error: "externally-managed-environment"

**Solution**: Use `--break-system-packages` flag as shown above.

### Error: "ModuleNotFoundError: No module named 'roslibpy'"

**Solution**: Install using:
```bash
pip install --break-system-packages roslibpy
```

### Error: "pip: command not found"

**Solution**: Install pip if not available:
```bash
sudo apt update
sudo apt install python3-pip
```

### ROS 2 Nodes Still Can't Find Module

**Solution**: Ensure you've sourced your ROS 2 workspace:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_rhapsodi-promtek/install/setup.bash
```

## Alternative: Virtual Environment (Not Recommended for ROS 2)

While virtual environments are generally recommended for Python projects, they are **not recommended** for ROS 2 development because:
- ROS 2 nodes need system-wide access to Python packages
- Launch files and ROS 2 tools expect packages in the system Python path
- It complicates the ROS 2 environment setup

If you must use a virtual environment, you would need to:
1. Activate it before sourcing ROS 2
2. Ensure ROS 2 can find packages in the venv
3. This is complex and error-prone

## References

- [PEP 668 - Marking Python base environments as "externally managed"](https://peps.python.org/pep-0668/)
- [ROS 2 Python Dependencies](https://docs.ros.org/en/jazzy/How-To-Guides/Installation-Troubleshooting.html)
- [roslibpy Documentation](https://roslibpy.readthedocs.io/)

---

**Last Updated**: November 2025  
**ROS 2 Distribution**: Jazzy Jalisco

