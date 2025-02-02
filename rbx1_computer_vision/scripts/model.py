# Temporary files for loading the model
from roboflow import Roboflow
import yaml
from ultralytics import YOLO

#Export dataset
rf = Roboflow(api_key="")
project = rf.workspace("oxarm-dnsiz").project("oxarm")
dataset = project.version(2).download("yolov8-obb")

# Allows us to rain dataset
with open(f'{dataset.location}/data.yaml', 'r') as file:
    data = yaml.safe_load(file)

data['path'] = dataset.location

with open(f'{dataset.location}/data.yaml', 'w') as file:
    yaml.dump(data, file, sort_keys=False)

model = YOLO("yolov8n.pt")

results = model.train(data=f'{dataset.location}/yolov8-obb.yaml', epochs=100, imgsz=640)
