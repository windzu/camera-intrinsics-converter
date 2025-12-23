# 快速开始

> 详细文档请参阅 [README.md](README.md)

## 30 秒上手

```bash
# 1. 安装
git clone https://github.com/windzu/camera-intrinsics-converter.git
cd camera-intrinsics-converter
python3 -m venv venv && source venv/bin/activate
pip install -e .

# 2. 转换
convert-intrinsics sensing data/sensing.txt output.yaml \
  --width 1920 --height 1080 --camera-name CAM_FRONT

# 3. 查看结果
cat output.yaml
```

## 常用命令

```bash
# 基本转换
convert-intrinsics sensing input.txt output.yaml \
  --width 1920 --height 1080 --camera-name CAM_FRONT

# 带时间戳
convert-intrinsics sensing input.txt output.yaml \
  --width 1920 --height 1080 --camera-name CAM_FRONT \
  --calibrated-at "2025-09-25T00:00:00Z"

# 详细输出
convert-intrinsics sensing input.txt output.yaml \
  --width 1920 --height 1080 --camera-name CAM_FRONT \
  --verbose
```

## 常用命令

```bash
# 基本转换
convert-intrinsics sensing input.txt output.yaml \
  --width 1920 --height 1080 --camera-name CAM_FRONT

# 带时间戳
convert-intrinsics sensing input.txt output.yaml \
  --width 1920 --height 1080 --camera-name CAM_FRONT \
  --calibrated-at "2025-09-25T00:00:00Z"

# 详细输出
convert-intrinsics sensing input.txt output.yaml \
  --width 1920 --height 1080 --camera-name CAM_FRONT \
  --verbose
```

## 在代码中使用

```python
import yaml

with open('output.yaml', 'r') as f:
    calib = yaml.safe_load(f)

# 获取内参
fx = calib['intrinsics']['fx']
fy = calib['intrinsics']['fy']
cx = calib['intrinsics']['cx']
cy = calib['intrinsics']['cy']

# 获取畸变系数
dc = calib['distortion_coefficients']
k1 = dc['k1']
k2 = dc['k2']
k3 = dc['k3']
p1 = dc['p1']
p2 = dc['p2']

# 如果是 rational 模型，还有
if calib['distortion_model'] == 'rational':
    k4 = dc['k4']
    k5 = dc['k5']
    k6 = dc['k6']
```

## 输入格式

Sensing 格式 (纯文本)：

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
K4:0.7584614814     # 可选
K5:-0.1730226671
K6:-0.0831068238
RMS:0.0127
```

## 更多帮助

```bash
convert-intrinsics --help
```

详见 [README.md](README.md)

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
