# Localization Package Usage

This package provides a dual EKF localization solution fusing wheel odometry, IMU, and GPS.

## Portability Update

This package has been refactored to use dynamic paths. Launch and config files do not rely on hardcoded paths (e.g., `/home/username`), making it easy to share.

## How to Use

1. **Build the package**:
   ```bash
   colcon build --packages-select localization
   ```

2. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

3. **Launch**:
   ```bash
   ros2 launch localization dual_ekf_localization.launch.py
   ```

The launch file will automatically locate the configuration files (`ekf_local.yaml`, `ekf_global.yaml`, etc.) in the package share directory.
