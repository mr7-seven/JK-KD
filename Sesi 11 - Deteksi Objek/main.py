import cv2
import numpy as np
import onnxruntime as ort
import time
import paho.mqtt.client as mqtt

# ===============================
# CONFIG
# ===============================
model_path = "model/yolov5n.onnx"
input_size = 320
conf_threshold = 0.25
nms_threshold = 0.4

PERSON_CLASS_ID = 0

MQTT_BROKER = "broker.emqx.io"
MQTT_PORT = 1883
MQTT_TOPIC = "jkkd/nim/person_count"

# ===============================
# COCO LABELS
# ===============================
class_names = [
    "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat",
    "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
    "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack",
    "umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball",
    "kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket",
    "bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
    "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake",
    "chair","couch","potted plant","bed","dining table","toilet","tv","laptop",
    "mouse","remote","keyboard","cell phone","microwave","oven","toaster","sink",
    "refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"
]

# ===============================
# MQTT V2 INIT
# ===============================
def on_connect(client, userdata, flags, reason_code, properties):
    print("[MQTT] Connected:", reason_code)
    client.subscribe(MQTT_TOPIC)

def on_message(client, userdata, msg):
    print(f"[MQTT] {msg.topic}: {msg.payload.decode()}")

mqttc = mqtt.Client(
    mqtt.CallbackAPIVersion.VERSION2,
    client_id="yolo_v5"
)

mqttc.on_connect = on_connect
mqttc.on_message = on_message

mqttc.connect(MQTT_BROKER, MQTT_PORT, 60)
mqttc.loop_start()

last_sent = -1

def publish(count):
    global last_sent
    if count == last_sent:
        return
    last_sent = count
    mqttc.publish(MQTT_TOPIC, str(count), qos=0, retain=False)


# ===============================
# LETTERBOX
# ===============================
def letterbox(img, new_shape=320):
    h, w = img.shape[:2]
    r = min(new_shape / h, new_shape / w)

    nh, nw = int(h * r), int(w * r)
    resized = cv2.resize(img, (nw, nh))

    canvas = np.full((new_shape, new_shape, 3), 114, dtype=np.uint8)
    canvas[:nh, :nw] = resized

    return canvas, r


# ===============================
# MODEL
# ===============================
so = ort.SessionOptions()
so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
so.intra_op_num_threads = 2

session = ort.InferenceSession(
    model_path,
    sess_options=so,
    providers=["CPUExecutionProvider"]
)

input_name = session.get_inputs()[0].name

cap = cv2.VideoCapture(0)

fps_list = []

# ===============================
# LOOP
# ===============================
while True:
    ret, frame = cap.read()
    if not ret:
        break

    start = time.time()

    # ===============================
    # PREPROCESS
    # ===============================
    img, scale = letterbox(frame, input_size)

    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))
    img = np.expand_dims(img, axis=0)

    # ===============================
    # INFERENCE
    # ===============================
    pred = session.run(None, {input_name: img})[0][0]

    conf_mask = pred[:, 4] > conf_threshold
    pred = pred[conf_mask]

    person_count = 0

    if len(pred) > 0:

        scores = pred[:, 4] * pred[:, 5:].max(axis=1)
        mask = scores > conf_threshold

        pred = pred[mask]
        scores = scores[mask]

        class_ids = pred[:, 5:].argmax(axis=1)

        cx, cy, w, h = pred[:, 0], pred[:, 1], pred[:, 2], pred[:, 3]

        cx /= scale
        cy /= scale
        w /= scale
        h /= scale

        x1 = (cx - w / 2).astype(int)
        y1 = (cy - h / 2).astype(int)
        w = w.astype(int)
        h = h.astype(int)

        boxes = np.stack([x1, y1, w, h], axis=1)

        indices = cv2.dnn.NMSBoxes(
            boxes.tolist(),
            scores.tolist(),
            conf_threshold,
            nms_threshold
        )

        if len(indices) > 0:
            indices = indices.flatten()

            for i in indices:
                x, y, w, h = boxes[i]
                cls = int(class_ids[i])
                conf = float(scores[i])

                if cls == PERSON_CLASS_ID:
                    person_count += 1

                # ===============================
                # YOLO STYLE BOX
                # ===============================
                color = (0, 255, 0)

                # BOX
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)

                # LABEL TEXT
                name = class_names[cls] if cls < len(class_names) else f"class {cls}"
                label = f"{name} {conf:.2f}"

                # TEXT SIZE
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)

                # LABEL BACKGROUND (YOLO STYLE)
                cv2.rectangle(
                    frame,
                    (x, y - th - 6),
                    (x + tw + 4, y),
                    color,
                    -1
                )

                # TEXT
                cv2.putText(
                    frame,
                    label,
                    (x + 2, y - 4),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 0),
                    1
                )

    # ===============================
    # MQTT
    # ===============================
    publish(person_count)

    # ===============================
    # FPS
    # ===============================
    fps = 1 / (time.time() - start)
    fps_list.append(fps)

    cv2.putText(frame, f"Person: {person_count}", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.putText(frame, f"FPS: {fps:.2f}", (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    cv2.imshow("YOLOv5 ONNX YOLO-STYLE + MQTT", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
mqttc.loop_stop()
mqttc.disconnect()

print("Avg FPS:", sum(fps_list)/len(fps_list))