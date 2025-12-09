# Camera Intrinsics Converter

相机内参格式转换工具，将不同厂家提供的相机内参文件转换为标准的 YAML 格式，方便在 ROS 等系统中使用。

## 功能特性

- ✅ 支持 Sensing 相机内参格式
- ✅ 智能转换：自动将 Rational Polynomial 畸变模型拟合为 RadTan (plumb_bob) 模型
- ✅ 精度保证：使用最小二乘拟合，最大限度保留畸变校正精度
- ✅ 完整元信息：保留原始参数和转换信息
- ✅ ROS 兼容：输出格式可直接用于发布 CameraInfo 消息
- 🔄 可扩展：架构支持添加更多厂家格式

## 为什么需要模型转换？

### 畸变模型差异

**Sensing 使用的 Rational Polynomial 模型**（8 参数）:
```
x_distorted = x * (1 + k1*r² + k2*r⁴ + k3*r⁶) / (1 + k4*r² + k5*r⁴ + k6*r⁶) + [切向畸变]
```

**ROS 标准的 RadTan/plumb_bob 模型**（5 参数）:
```
x_distorted = x * (1 + k1*r² + k2*r⁴ + k3*r⁶) + [切向畸变]
```

### 精度损失问题

直接丢弃 K4, K5, K6 会导致**显著的精度损失**，特别是当这些系数较大时。

**本工具的解决方案：**
1. 在图像有效区域采样大量点（默认 10000 个）
2. 使用原始 Rational 模型计算这些点的畸变坐标
3. 用这些点对重新拟合 RadTan 5 参数模型
4. 这样在 RadTan 模型约束下最小化了误差

> ⚠️ **重要提示**: 拟合转换可以显著降低精度损失，但不能完全消除。如果原始 K4, K5, K6 系数很大（如示例中的 K4=0.758），转换后仍可能在图像边缘区域存在微小的残差。对于高精度应用，建议查看转换后的 `meta.fitting_error_rms` 值来评估精度。

## 安装

### 方式 1: 开发模式安装（推荐）

```bash
# 克隆仓库
git clone https://github.com/windzu/camera-intrinsics-converter.git
cd camera-intrinsics-converter

# 安装依赖
pip install -r requirements.txt

# 开发模式安装
pip install -e .
```

### 方式 2: 直接安装

```bash
pip install git+https://github.com/windzu/camera-intrinsics-converter.git
```

### 依赖项

- Python >= 3.7
- numpy >= 1.20.0
- scipy >= 1.7.0
- pyyaml >= 5.4.0

## 使用方法

### 基本用法

```bash
convert-intrinsics sensing input.txt output.yaml \
  --width 1920 \
  --height 1080 \
  --camera-name CAM_FRONT
```

### 完整示例

```bash
convert-intrinsics sensing data/sensing.txt output/cam_front.yaml \
  --width 1920 \
  --height 1080 \
  --camera-name CAM_FRONT \
  --camera-model pinhole \
  --calibrated-at "2025-09-25T00:00:00Z" \
  --num-samples 15000 \
  --verbose
```

### 参数说明

| 参数 | 必需 | 说明 |
|------|------|------|
| `format` | ✓ | 输入格式类型，当前支持: `sensing` |
| `input` | ✓ | 输入文件路径 |
| `output` | ✓ | 输出 YAML 文件路径 |
| `--width` | ✓ | 图像宽度（像素） |
| `--height` | ✓ | 图像高度（像素） |
| `--camera-name` | ✓ | 相机名称/标识符（如 CAM_FRONT） |
| `--camera-model` | | 相机模型类型，默认: `pinhole` |
| `--calibrated-at` | | 标定时间（ISO 格式） |
| `--num-samples` | | 拟合采样点数，默认: 10000 |
| `--verbose` | | 显示详细转换信息 |

## 输入格式示例

### Sensing 格式

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

## 输出格式示例

```yaml
image_width: 1920
image_height: 1080
camera_name: CAM_FRONT

camera_model: pinhole  # 成像模型 pinhole or fisheye
# 针孔：plumb_bob（= RadTan）
# 鱼眼：equidistant
distortion_model: plumb_bob
distortion_coefficients:
  k1: 0.3145892314
  k2: -0.1876543210
  p1: -0.0000441164  # t1
  p2: 0.0000196223  # t2
  k3: -0.0156789012  # 若无就写 0（例如fisheye就是0）

intrinsics:
  fx: 1188.6918066682
  fy: 1188.8881856981
  cx: 958.1162131187
  cy: 770.3201938023

meta:  # 可选元信息（不影响 CameraInfo 语义）
  calibrated_at: "2025-09-25T00:00:00Z"
  serial_number: "H100F1A-H09150733"
  original_format: "sensing"
  original_rms: 0.0127
  conversion_method: "fitted_from_rational_model"
  fitting_error_rms: 0.000123456
  original_rational_model:
    k1: 0.3323618292
    k2: -0.2032698843
    p1: -0.0000441164
    p2: 0.0000196223
    k3: -0.0168648964
    k4: 0.7584614814
    k5: -0.1730226671
    k6: -0.0831068238
  converted_at: "2025-12-09T10:30:00.000000Z"
```

## 转换质量评估

使用 `--verbose` 参数可以查看详细的转换信息：

```bash
convert-intrinsics sensing input.txt output.yaml \
  --width 1920 --height 1080 --camera-name CAM_FRONT \
  --verbose
```

输出示例：
```
Parsing sensing format file: data/sensing.txt
  Serial Number: H100F1A-H09150733
  Original intrinsics: fx=1188.6918, fy=1188.8882, cx=958.1162, cy=770.3202
  Has rational model: True

Rational model analysis:
  Rational model coefficients are significant (max=0.7585).
  Direct conversion may cause noticeable distortion errors.
  K4=0.758461, K5=-0.173023, K6=-0.083107

Fitting RadTan model with 10000 sample points...
  Fitting RMS error: 0.000234 (normalized coordinates)
  Fitted coefficients:
    k1=0.3145892314
    k2=-0.1876543210
    p1=-0.0000441164
    p2=0.0000196223
    k3=-0.0156789012

✓ Successfully converted to output.yaml
```

### 精度指标说明

- **fitting_error_rms**: 归一化坐标系下的均方根误差
  - < 0.001: 优秀，几乎无可察觉的差异
  - 0.001 - 0.005: 良好，大部分应用可接受
  - 0.005 - 0.01: 可接受，边缘区域可能有轻微差异
  - \> 0.01: 需要注意，建议增加 `--num-samples`

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

# 畸变系数 (k1, k2, p1, p2, k3)
dc = calib['distortion_coefficients']
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
│       ├── __init__.py
│       ├── cli.py                  # 命令行接口
│       ├── parsers/                # 输入格式解析器
│       │   ├── __init__.py
│       │   └── sensing.py          # Sensing 格式解析
│       ├── converters/             # 模型转换器
│       │   ├── __init__.py
│       │   └── distortion.py       # 畸变模型转换
│       └── output/                 # 输出生成器
│           ├── __init__.py
│           └── yaml_generator.py   # YAML 生成
├── data/                           # 示例数据
│   └── sensing.txt
├── requirements.txt
├── setup.py
└── README.md
```

## 扩展支持其他格式

要添加新的相机格式支持：

1. 在 `src/camera_converter/parsers/` 创建新的解析器
2. 继承基本接口并实现 `parse()` 方法
3. 在 `cli.py` 中注册新格式

示例：
```python
# src/camera_converter/parsers/new_format.py
class NewFormatParser:
    def parse(self) -> Dict[str, Any]:
        # 实现解析逻辑
        return {
            'fx': ..., 'fy': ..., 'cx': ..., 'cy': ...,
            'k1': ..., 'k2': ..., 'p1': ..., 'p2': ..., 'k3': ...,
        }
```

## 常见问题

### Q: 拟合需要多长时间？
A: 默认 10000 个采样点通常在 1-3 秒内完成。可以通过 `--num-samples` 调整。

### Q: 如何判断转换质量？
A: 使用 `--verbose` 查看 `fitting_error_rms`。对于 1920x1080 图像，< 0.001 的误差通常意味着像素级误差 < 1-2 像素。

### Q: 原始 K4, K5, K6 很小，还需要拟合吗？
A: 工具会自动检测。如果系数 < 0.01，可以考虑直接复制，但拟合方法总是更安全。

### Q: 支持鱼眼相机吗？
A: 当前版本支持针孔相机。鱼眼相机使用不同的畸变模型（equidistant），需要单独实现。

## 技术细节

### 拟合算法

1. **采样策略**: 
   - 70% 均匀网格采样（覆盖整个图像）
   - 30% 随机径向采样（加强不同半径的覆盖）

2. **优化方法**: 
   - Levenberg-Marquardt 算法
   - 最小化归一化坐标系下的重投影误差

3. **收敛条件**:
   - 自动收敛检测
   - 通常 < 100 次迭代

## 许可证

MIT License

## 贡献

欢迎提交 Issue 和 Pull Request！

如果需要添加新的相机格式支持，请：
1. 提供样例文件
2. 说明格式规范
3. 创建 Issue 讨论

## 联系方式

- GitHub: [@windzu](https://github.com/windzu)
- Issues: [GitHub Issues](https://github.com/windzu/camera-intrinsics-converter/issues)

## 更新日志

### v0.1.0 (2025-12-09)
- ✨ 初始版本
- ✅ 支持 Sensing 格式
- ✅ Rational to RadTan 拟合转换
- ✅ YAML 输出格式
- ✅ 命令行工具