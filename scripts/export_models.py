from ultralytics import YOLO

# Stage 1: 全图检测，imgsz=1280
model1 = YOLO("weights/stage_one.pt")
model1.export(format="engine", imgsz=1280, half=True, device=0)

# Stage 2: 装甲板分类，imgsz=256
model2 = YOLO("weights/stage_two.pt")
model2.export(format="engine", imgsz=256, half=True, device=0)

# Stage 3: 灰色装甲板分类，imgsz=256
model3 = YOLO("weights/stage_three.pt")
model3.export(format="engine", imgsz=256, half=True, device=0)
