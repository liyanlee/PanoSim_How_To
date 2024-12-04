from ultralytics.utils.downloads import attempt_download_asset

from ultralytics import YOLO
import onnx

# 将模型导出为onnx模型

# model = YOLO("yolov8n.pt")
# model.export(format = "onnx")

# 加载模型
model_path = "yolov8n.onnx"
model = onnx.load(model_path)

# 检查当前的IR版本
print(f"Current IR version: {model.ir_version}")

# 设置新的IR版本
model.ir_version = 8

# 保存修改后的模型

# aa = model.predict(["C:/Users/Administrator/Pictures/11.png"])
# aa[0].show()
# 检查当前的操作集版本
print(f"Current opset version: {model.opset_import[0].version}")

# 设置新的操作集版本
for opset in model.opset_import:
    if opset.domain == '':
        opset.version = 15

# 保存修改后的模型
output_path = 'path_to_your_model_v8.onnx'
onnx.save(model, output_path)
print(f"Model saved to {output_path} with IR version 8")
print(f"Model saved to {output_path} with opset version 15")

