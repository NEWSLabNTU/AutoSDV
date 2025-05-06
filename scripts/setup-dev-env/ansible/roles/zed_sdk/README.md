# Role: zed_sdk

This role installs StereoLabs ZED SDK version 4.2.

## Inputs

| Variable               | Description                  | Default |
|------------------------|------------------------------|---------|
| `zed_sdk_version`      | ZED SDK version              | `4.2`   |
| `zed_sdk_cuda_version` | CUDA version for x86_64      | `12`    |
| `zed_sdk_l4t_version`  | L4T version for ARM64/Jetson | `36.3`  |

## Supported Platforms

- Ubuntu 22.04 (x86_64) with CUDA 12
- NVIDIA Jetson with L4T 36.3 (JetPack 6.0)

## Manual Installation

### For x86_64 (Ubuntu 22.04):

```bash
wget -O ZED_SDK.run https://download.stereolabs.com/zedsdk/4.2/cu12/ubuntu22
chmod +x ZED_SDK.run
sudo ./ZED_SDK.run -- --silent
```

### For ARM64 (Jetson with JetPack 6.0):

```bash
wget -O ZED_SDK.run https://download.stereolabs.com/zedsdk/4.2/l4t36.3/jetsons
chmod +x ZED_SDK.run
sudo ./ZED_SDK.run -- --silent
```

## Dependencies

- CUDA 12+ (for x86_64)
- NVIDIA Jetson L4T 36.3/JetPack 6.0 (for ARM64/Jetson)
