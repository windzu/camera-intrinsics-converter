# Camera Intrinsics Converter

相机内参格式转换工具，将不同厂家提供的相机内参文件转换为标准的 YAML 格式，方便在 ROS 等系统中使用。

## 功能特性

- ✅ 支持 Sensing 相机内参格式  
- ✅ 自动检测畸变模型：有 K4/K5/K6 则输出 `rational` 模型，否则输出 `plumb_bob` 模型
- ✅ 原始参数保留：完整保存原始内参和畸变系数
- ✅ ROS 兼容：输出格式可直接用于 CameraInfo 消息
- 🔄 可扩展：架构支持添加更多厂家格式

## 快速开始

### 1. 安装

#### 方式 1：使用虚拟环境（推荐）

```bash
# 克隆仓库
git clone https://github.com/windzu/camera-intrinsics-converter.git
cd camera-intrinsics-converter

# 创建并激活虚拟环境
python3 -m venv venv
source venv/bin/activate  # Linux/Mac
# 或 venv\Scripts\activate  # Windows

# 安装依赖
pip install -r requirements.txt

# 开发模式安装
pip install -e .
```

#### 方式 2：直接安装

```bash
pip install git+https://github.com/windzu/camera-intrinsics-converter.git
```

#### 依赖项

- Python >= 3.7
- numpy >= 1.20.0
- scipy >= 1.7.0
- pyyaml >= 5.4.0

### 2. 基本用法

```bash
convert-intrinsics sensing input.txt output.yaml \
  --width 1920 \
  --height 1080 \
  --camera-name CAM_FRONT \
  --verbose
```

### 3. 完整示例

```bash
convert-intrinsics sensing data/sensing.txt output.yaml \
  --width 1920 \
  --height 1080 \
  --camera-name CAM_FRONT \
  --camera-model pinhole \
  --calibrated-at "2025-09-25T00:00:00Z"
```

### 4. 参数说明

| 参数 | 必需 | 说明 |
|------|------|------|
| `format` | ✓ | 输入格式类型：`sensing` |
| `input` | ✓ | 输入文件路径 |
| `output` | ✓ | 输出 YAML 文件路径 |
| `--width` | ✓ | 图像宽度（像素） |
| `--height` | ✓ | 图像高度（像素） |
| `--camera-name` | ✓ | 相机名称/标识符（如 CAM_FRONT） |
| `--camera-model` | | 相机模型：`pinhole`（默认）或 `fisheye` |
| `--calibrated-at` | | 标定时间（ISO 8601 格式） |
| `--verbose` | | 显示详细转换信息 |

## 格式说明

### Sensing 输入格式

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
K4:0.7584614814     # 可选：有则输出 rational 模型
K5:-0.1730226671
K6:-0.0831068238
RMS:0.0127
```

### YAML 输出格式

**含 K4/K5/K6 的输出（rational 模型）：**

```yaml
image_width: 1920
image_height: 1080
camera_name: CAM_FRONT

camera_model: pinhole
distortion_model: rational
distortion_coefficients:
  k1: 0.3323618292
  k2: -0.2032698843
  p1: -4.41164e-05
  p2: 1.96223e-05
  k3: -0.0168648964
  k4: 0.7584614814  # rational 模型参数
  k5: -0.1730226671
  k6: -0.0831068238

intrinsics:
  fx: 1188.6918066682
  fy: 1188.8881856981
  cx: 958.1162131187
  cy: 770.3201938023

meta:
  original_format: "sensing"
  serial_number: "H100F1A-H09150733"
  original_rms: 0.0127
  conversion_method: "format_only"
  converted_at: "2025-12-23T14:50:27.767307Z"
```

**无 K4/K5/K6 的输出（plumb_bob 模型）：**

```yaml
image_width: 1920
image_height: 1080
camera_name: CAM_FRONT

camera_model: pinhole
distortion_model: plumb_bob
distortion_coefficients:
  k1: 0.3323618292
  k2: -0.2032698843
  p1: -4.41164e-05
  p2: 1.96223e-05
  k3: -0.0168648964

intrinsics:
  fx: 1188.6918066682
  fy: 1188.8881856981
  cx: 958.1162131187
  cy: 770.3201938023

meta:
  original_format: "sensing"
  serial_number: "H100F1A-H09150733"
  original_rms: 0.0127
  conversion_method: "direct_copy"
  converted_at: "2025-12-23T14:51:17.918411Z"
```

## 在 ROS 中使用

生成的 YAML 文件可以直接用于 ROS CameraInfo：

```python
import yaml
import rospy
from sensor_msgs.msg import CameraInfo

# 读取转换后的内参文件
with open('output.yaml', 'r') as f:
    calib = yaml.safe_load(f)

# 创建 CameraInfo 消息
camera_info = CameraInfo()
camera_info.width = calib['image_width']
camera_info.height = calib['image_height']
camera_info.distortion_model = calib['distortion_model']

# 畸变系数
dc = calib['distortion_coefficients']
if calib['distortion_model'] == 'rational':
    # rational 模型：8 个系数
    camera_info.D = [dc['k1'], dc['k2'], dc['p1'], dc['p2'], dc['k3'], dc['k4'], dc['k5'], dc['k6']]
else:
    # plumb_bob 模型：5 个系数
    camera_info.D = [dc['k1'], dc['k2'], dc['p1'], dc['p2'], dc['k3']]

# 内参矩阵
intr = calib['intrinsics']
camera_info.K = [
    intr['fx'], 0, intr['cx'],
    0, intr['fy'], intr['cy'],
    0, 0, 1
]

# 发布
pub = rospy.Publisher('/camera_info', CameraInfo, queue_size=10)
pub.publish(camera_info)
```

## 项目结构

```
camera-intrinsics-converter/
├── src/
│   └── camera_converter/
│       ├── cli.py                  # 命令行接口
│       ├── parsers/
│       │   └── sensing.py          # Sensing 格式解析
│       ├── converters/
│       │   └── distortion.py       # 畸变模型处理
│       └── output/
│           └── yaml_generator.py   # YAML 输出生成
├── data/
│   └── sensing.txt                 # 示例数据
├── tests/
│   └── test_conversion.py          # 单元测试
├── requirements.txt
├── setup.py
└── README.md
```

## 扩展支持其他格式

要添加新的相机格式支持：

1. 在 `src/camera_converter/parsers/` 创建新的解析器文件
2. 实现 `parse()` 方法返回标准字典
3. 在 `cli.py` 中添加格式选择逻辑

## 常见问题

**Q: 如何判断输出的畸变模型？**

A: 工具会自动检测输入文件中是否包含 K4/K5/K6：

- 有 K4/K5/K6 → 输出 `rational` 模型（8 个参数）
- 无 K4/K5/K6 → 输出 `plumb_bob` 模型（5 个参数）

**Q: ROS 支持 rational 模型吗？**

A: ROS 的 CameraInfo 消息支持任意数量的畸变系数，可以通过 `D` 字段传递 8 个系数的 rational 模型。

**Q: 如何在代码中读取 YAML 文件？**

A: 使用 PyYAML：

```python
import yaml
with open('output.yaml', 'r') as f:
    calib = yaml.safe_load(f)
fx = calib['intrinsics']['fx']
```

## 许可证

MIT License

## 更新日志

### v0.1.0 (2025-12-23)

- ✨ 初始版本
- ✅ 支持 Sensing 格式
- ✅ 自动检测 rational/plumb_bob 畸变模型
- ✅ YAML 输出格式
- ✅ 命令行工具
