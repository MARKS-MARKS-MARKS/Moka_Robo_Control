# 棋盘格倒茶 YOLO 数据集工具

这个目录用于先做“只看不动”的视觉阶段：采集桌面棋盘上的茶杯、茶叶罐、矿泉水瓶图片，标注后训练 YOLO，最后输出识别框。这里的脚本不会给机器人发送任何运动指令。

## 类别

```text
0 cup
1 tea_can
2 water_bottle
```

## 1. 拍照采集

确保网页服务没有打开相机预览，然后运行：

```bash
cd /home/seuauto/moka
python3 vision_yolo/capture_dataset.py --count 40 --interval 0.25
```

图片会保存到：

```text
vision_yolo/datasets/tea_objects/images/train
```

建议每拍一批，轻微改变光照、相机角度或物品摆放位置，让模型更稳。

## 2. 手动标注

```bash
cd /home/seuauto/moka
python3 vision_yolo/label_images.py
```

操作方式：

- 鼠标拖框。
- 按 `0` 保存为 cup。
- 按 `1` 保存为 tea_can。
- 按 `2` 保存为 water_bottle。
- 按 `u` 撤销上一框。
- 按 `n` 下一张。
- 按 `q` 退出。

标注会保存到：

```text
vision_yolo/datasets/tea_objects/labels/train
```

## 3. 安装训练依赖

当前电脑还没有 `ultralytics` 和 `torch`。需要训练时安装：

```bash
python3 -m pip install ultralytics
```

如果安装过程提示需要 PyTorch，请按提示安装 CPU 版或 CUDA 版。

## 4. 训练 YOLO

至少标注几十张后再训练：

```bash
cd /home/seuauto/moka
python3 vision_yolo/train_yolo.py --epochs 80 --imgsz 640
```

训练输出通常在：

```text
runs/detect/tea_objects
```

## 5. 只检测，不控制机械臂

```bash
cd /home/seuauto/moka
python3 vision_yolo/detect_once.py --model runs/detect/tea_objects/weights/best.pt
```

输出：

```text
vision_yolo/output/latest_detection.jpg
vision_yolo/output/latest_detection.json
```

