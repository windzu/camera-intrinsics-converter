# Quick Start Guide

## 快速开始

### 1. 安装

```bash
# 克隆仓库
git clone https://github.com/windzu/camera-intrinsics-converter.git
cd camera-intrinsics-converter

# 创建虚拟环境（推荐）
python3 -m venv venv
source venv/bin/activate  # Linux/Mac
# 或 venv\Scripts\activate  # Windows

# 安装
pip install -e .
```

### 2. 准备输入文件

确保你有一个 Sensing 格式的内参文件，格式如下：

```
SN码:H100F1A-H09150733
FX:1188.6918066682
FY:1188.8881856981
CX:958.1162131187
CY:770.3201938023
K1:0.3323618292
K2:-0.2032698843
P1:-0.0000441164
P2:0.0000196223
K3:-0.0168648964
K4:0.7584614814
K5:-0.1730226671
K6:-0.0831068238
RMS:0.0127
```

### 3. 转换

```bash
convert-intrinsics sensing data/sensing.txt output.yaml \
  --width 1920 \
  --height 1080 \
  --camera-name CAM_FRONT \
  --verbose
```

### 4. 查看结果

```bash
cat output.yaml
```

### 5. 在代码中使用

```python
import yaml

# 读取转换后的文件
with open('output.yaml', 'r') as f:
    calib = yaml.safe_load(f)

# 获取内参
fx = calib['intrinsics']['fx']
fy = calib['intrinsics']['fy']
cx = calib['intrinsics']['cx']
cy = calib['intrinsics']['cy']

# 获取畸变系数
k1 = calib['distortion_coefficients']['k1']
k2 = calib['distortion_coefficients']['k2']
p1 = calib['distortion_coefficients']['p1']
p2 = calib['distortion_coefficients']['p2']
k3 = calib['distortion_coefficients']['k3']

print(f"Camera: {calib['camera_name']}")
print(f"Resolution: {calib['image_width']}x{calib['image_height']}")
```

### 6. 运行示例

```bash
# 查看如何在 ROS 中使用
python examples/example_usage.py output.yaml
```

## 完整参数说明

```bash
convert-intrinsics sensing INPUT OUTPUT \
  --width WIDTH              # 必需：图像宽度
  --height HEIGHT            # 必需：图像高度
  --camera-name NAME         # 必需：相机名称
  --camera-model pinhole     # 可选：相机模型（默认 pinhole）
  --calibrated-at TIMESTAMP  # 可选：标定时间
  --num-samples 10000        # 可选：拟合采样点数（默认 10000）
  --verbose                  # 可选：显示详细信息
```

## 常用命令

### 基础转换

```bash
convert-intrinsics sensing input.txt output.yaml \
  --width 1920 --height 1080 --camera-name CAM_FRONT
```

### 带时间戳

```bash
convert-intrinsics sensing input.txt output.yaml \
  --width 1920 --height 1080 --camera-name CAM_FRONT \
  --calibrated-at "2025-09-25T00:00:00Z"
```

### 高精度转换（更多采样点）

```bash
convert-intrinsics sensing input.txt output.yaml \
  --width 1920 --height 1080 --camera-name CAM_FRONT \
  --num-samples 50000 \
  --verbose
```

### 查看详细信息

```bash
convert-intrinsics sensing input.txt output.yaml \
  --width 1920 --height 1080 --camera-name CAM_FRONT \
  --verbose
```

## 故障排除

### 问题：找不到 convert-intrinsics 命令

**解决**：确保已激活虚拟环境

```bash
source venv/bin/activate
```

### 问题：fitting_error_rms 太大

**解决**：增加采样点数

```bash
convert-intrinsics ... --num-samples 50000
```

### 问题：转换太慢

**解决**：减少采样点数

```bash
convert-intrinsics ... --num-samples 5000
```

## 更多帮助

- 查看完整文档：[README.md](README.md)
- 查看命令帮助：`convert-intrinsics --help`
- 提交问题：[GitHub Issues](https://github.com/windzu/camera-intrinsics-converter/issues)
