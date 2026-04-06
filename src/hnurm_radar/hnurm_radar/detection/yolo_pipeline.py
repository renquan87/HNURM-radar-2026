"""
yolo_pipeline.py — 三阶段 YOLO 推理引擎

从 detector_node.py 和 camera_detector.py 中提取的公共三阶段推理逻辑，
不包含任何 ROS2 依赖，可独立使用。

三阶段流程：
  Stage 1 — track_infer():     全图目标检测 + ByteTrack 追踪 (stage_one, imgsz=1280)
  Stage 2 — classify_infer():  ROI 装甲板颜色+编号分类      (stage_two, imgsz=256)
  Stage 3 — classify_infer():  ROI 灰色装甲板专用分类        (stage_three, imgsz=256)

投票机制：
  每个 track_id 维护一个长度为 class_num 的投票表 Track_value，
  每帧分类结果按 (0.5 + conf * 0.5) 累加投票，每 life_time 帧衰减为 1/10，
  最终取投票最高的类别作为该目标的标签。

高置信度纠正（默认开启）：
  当 conf > 0.85 且当前帧分类与投票最大值不一致时，
  给予 4 倍权重 (2.0 + conf * 2.0) 加速纠正错误累积。

用法::

    from detection.yolo_pipeline import YoloPipeline
    pipeline = YoloPipeline(det_cfg, logger=my_logger)
    annotated_frame, results = pipeline.infer(frame)
"""

import math
import cv2

from .label_mappings import (
    Gray2Blue, Gray2Red, gray2gray, Blue2Gray, Red2Gray,
)


class YoloPipeline:
    """三阶段 YOLO 推理引擎（ROS 无关）。

    Parameters
    ----------
    det_cfg : dict
        ``detector_config.yaml`` 解析后的字典，需包含：
        - ``path.stage_one_path``, ``path.stage_two_path``, ``path.stage_three_path``
        - ``path.tracker_path``
        - ``params.labels``, ``params.stage_one_conf``, ``params.stage_two_conf``,
          ``params.stage_three_conf``, ``params.life_time``
    resolve_fn : callable
        将相对路径解析为绝对路径的函数，签名 ``resolve_fn(rel_path) -> abs_path``。
    logger : object, optional
        日志对象，需提供 ``.info()`` / ``.warn()`` / ``.error()`` 方法。
        若为 ``None`` 则使用 ``print`` 输出。
    high_conf_correction : bool
        是否启用高置信度纠正逻辑（默认 True）。
    max_track_id : int
        Track_value 表的最大 track_id 容量（默认 10000）。
    """

    def __init__(
        self,
        det_cfg: dict,
        resolve_fn=None,
        logger=None,
        high_conf_correction: bool = True,
        max_track_id: int = 10000,
    ):
        self._log = logger or _PrintLogger()
        self._resolve = resolve_fn or (lambda x: x)

        # ── 加载 YOLO 模型 ──
        # 延迟到这里才 import，避免在非推理场景（如 label_mappings）中强制依赖 ultralytics
        from ultralytics import YOLO

        self._log.info('YoloPipeline: 正在加载 YOLO 模型 ...')
        self.model_stage1 = YOLO(
            self._resolve(det_cfg['path']['stage_one_path']), task="detect")
        self.model_stage1.overrides['imgsz'] = 1280

        self.model_stage2 = YOLO(
            self._resolve(det_cfg['path']['stage_two_path']))
        self.model_stage2.overrides['imgsz'] = 256

        self.model_stage3 = YOLO(
            self._resolve(det_cfg['path']['stage_three_path']))
        self.model_stage3.overrides['imgsz'] = 256
        self._log.info('YoloPipeline: YOLO 模型加载完毕。')

        # ── 推理参数 ──
        self.tracker_path = self._resolve(det_cfg['path']['tracker_path'])
        self.stage_one_conf = det_cfg['params']['stage_one_conf']
        self.stage_two_conf = det_cfg['params']['stage_two_conf']
        self.stage_three_conf = det_cfg['params']['stage_three_conf']
        self.life_time = det_cfg['params']['life_time']
        self.labels = det_cfg['params']['labels']
        self.class_num = len(self.labels)
        self.high_conf_correction = high_conf_correction

        # ── 灰色装甲板亮度阈值（从配置读取，兼容旧配置） ──
        gray_cfg = det_cfg.get('params', {}).get('gray_threshold', {})
        self.gray_thresh_blue = gray_cfg.get('blue', 35)
        self.gray_thresh_red = gray_cfg.get('red', 15)
        self.gray_thresh_stage3 = gray_cfg.get('stage3', 30)

        # ── 标签映射（使用模块级常量的引用） ──
        self.Gray2Blue = Gray2Blue
        self.Gray2Red = Gray2Red
        self.gray2gray = gray2gray
        self.Blue2Gray = Blue2Gray
        self.Red2Gray = Red2Gray

        # ── 投票状态 ──
        self._max_track_id = max_track_id
        self.Track_value = {}
        self.Status = [0] * max_track_id
        for i in range(max_track_id):
            self.Track_value[i] = [0] * self.class_num
        self.id_candidate = [0] * max_track_id
        self.loop_times = 0

    # ================================================================
    #  公共方法
    # ================================================================

    def is_results_empty(self, results) -> bool:
        """判断 Stage 1 追踪结果是否为空。"""
        if results is None:
            return True
        if results[0].boxes.id is None:
            return True
        return False

    def parse_results(self, results):
        """解析 Stage 1 追踪结果为 numpy 数组。

        Returns
        -------
        confidences : ndarray, shape (N,)
        boxes : ndarray, shape (N, 4)  — xywh 格式
        track_ids : list[int]
        """
        confidences = results[0].boxes.conf.cpu().numpy()
        boxes = results[0].boxes.xywh.cpu().numpy()
        track_ids = results[0].boxes.id.int().cpu().tolist()
        return confidences, boxes, track_ids

    def track_infer(self, frame):
        """Stage 1：全图目标检测 + ByteTrack 追踪。"""
        results = self.model_stage1.track(
            frame, persist=True, tracker=self.tracker_path, verbose=False)
        return results

    def classify_infer(self, roi_list):
        """Stage 2 + Stage 3：对 ROI 列表进行分类推理。

        对每个 ROI 同时运行 Stage 2（颜色/编号分类）和 Stage 3（灰色装甲板分类），
        根据灰度阈值判定是否使用灰色结果覆盖 Stage 2 结果。

        Parameters
        ----------
        roi_list : list[ndarray]
            BGR 格式的 ROI 图像列表。

        Returns
        -------
        (label_list, conf_list) : tuple[list[int], list[float]]
            每个 ROI 的分类标签和置信度。若无检测结果返回 ``-1``。
        """
        results = []
        gray_results = []
        for roi in roi_list:
            results.extend(self.model_stage2.predict(
                roi, conf=self.stage_two_conf, device=0, verbose=False))
            gray_results.extend(self.model_stage3.predict(
                roi, device=0, verbose=False))

        if len(results) == 0:
            return -1

        label_list = []
        conf_list = []
        for result, gray_result, roi in zip(results, gray_results, roi_list):
            roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            data = result.boxes.data
            gray_data = gray_result.boxes.data
            maxConf = 0
            maxGrayConf = 0
            label = -1
            gray_label = -1
            gray_tmp = []
            tmp = []

            for i in range(len(gray_data)):
                if gray_data[i][4] > maxGrayConf:
                    maxGrayConf = gray_data[i][4]
                    gray_label = gray_data[i][5]
                    gray_tmp = gray_data[i]

            for i in range(len(data)):
                if data[i][4] > maxConf:
                    maxConf = data[i][4]
                    label = data[i][5]
                    tmp = data[i]

            if len(tmp) != 0:
                x1, y1, x2, y2 = tmp[:4]
                gray_region = roi_gray[int(y1):int(y2), int(x1):int(x2)]
                # 边界检查：防止裁剪区域为空时 cv2.mean 崩溃
                if label < 6 and gray_region.size > 0 and cv2.mean(gray_region)[0] < self.gray_thresh_blue:
                    label = self.Blue2Gray[int(label)]
                elif label > 5 and gray_region.size > 0 and cv2.mean(gray_region)[0] < self.gray_thresh_red:
                    label = self.Red2Gray[int(label)]
            else:
                if len(gray_tmp) != 0:
                    x1, y1, x2, y2 = gray_tmp[:4]
                    gray_region = roi_gray[int(y1):int(y2), int(x1):int(x2)]
                    if gray_region.size > 0 and cv2.mean(gray_region)[0] < self.gray_thresh_stage3:
                        label = self.gray2gray[int(gray_label)]

            label_list.append(int(label))
            conf_list.append(float(maxConf))
        return label_list, conf_list

    def infer(self, frame):
        """完整三阶段推理管线。

        流程：Stage 1 追踪 → ROI 裁剪 → Stage 2/3 分类 → 投票 → 判重 → 标注

        Parameters
        ----------
        frame : ndarray
            BGR 格式的输入图像。

        Returns
        -------
        (annotated_frame, zip_results) : tuple
            - ``annotated_frame``: 带标注的图像（原地修改）
            - ``zip_results``: 结果列表，每个元素为
              ``[xyxy_box, xywh_box, track_id, label_str]``。
              若无检测结果则为 ``None``。
        """
        if frame is None:
            return None, None

        results = self.track_infer(frame)
        if self.is_results_empty(results):
            return frame, None

        # exist_armor[label] 记录当前帧中每个类别被哪个 track_id 占据（用于判重）
        exist_armor = [-1] * (self.class_num + 6)
        draw_candidate = []
        confidences, boxes, track_ids = self.parse_results(results)
        zip_results = []
        roi_list = []
        id_list = []
        box_list = []

        for box, track_id, conf in zip(boxes, track_ids, confidences):
            # 周期性衰减投票值
            if self.loop_times % self.life_time == 1:
                for i in range(self.class_num):
                    self.Track_value[int(track_id)][i] = math.floor(
                        self.Track_value[int(track_id)][i] / 10)

            x, y, w, h = box
            x_left = x - w / 2
            y_left = y - h / 2
            roi = frame[int(y_left):int(y_left + h), int(x_left):int(x_left + w)]
            # 边界检查：防止空 ROI
            if roi.size == 0:
                continue
            roi_list.append(roi)
            id_list.append(track_id)
            box_list.append(box)

        if len(roi_list) == 0:
            self.loop_times += 1
            return frame, None

        classify_result = self.classify_infer(roi_list)
        if classify_result == -1:
            self.loop_times += 1
            return frame, None

        label_list, conf_list = classify_result
        index = 0
        for i in range(len(roi_list)):
            classify_label = label_list[i]
            conf = conf_list[i]
            track_id = id_list[i]
            box = box_list[i]
            x, y, w, h = box
            status = 0

            if classify_label != -1:
                label = self.Track_value[int(track_id)].index(
                    max(self.Track_value[int(track_id)]))

                if classify_label > 11:  # 灰色装甲板
                    status = 1
                    if self.Status[track_id] < 6:
                        self.Status[track_id] += status
                    # 使用 .get() 防止未知灰甲板 label 导致 KeyError
                    if label < 6 and self.Gray2Blue.get(classify_label) == label:
                        self.Track_value[int(track_id)][int(float(
                            self.Gray2Blue[classify_label]))] += 0.5 + conf * 0.5
                    elif label > 5 and self.Gray2Red.get(classify_label) == label:
                        self.Track_value[int(track_id)][int(float(
                            self.Gray2Red[classify_label]))] += 0.5 + conf * 0.5
                    else:
                        classify_label = -1
                else:
                    if self.Status[int(track_id)] > 0:
                        self.Status[int(track_id)] -= 1
                    if self.Status[int(track_id)] < 4:
                        self.Status[int(track_id)] = 0

                    # 投票权重计算
                    vote_weight = 0.5 + conf * 0.5
                    # 高置信度纠正：当前帧分类与投票结果不一致时，加大权重加速纠正
                    if (self.high_conf_correction
                            and conf > 0.85
                            and label != int(float(classify_label))
                            and max(self.Track_value[int(track_id)]) > 0):
                        vote_weight = 2.0 + conf * 2.0
                    self.Track_value[int(track_id)][int(float(
                        classify_label))] += vote_weight

            label = self.Track_value[int(track_id)].index(
                max(self.Track_value[int(track_id)]))

            # ── 判重：同一类别只保留投票值最高的 track_id ──
            if label < len(exist_armor) and exist_armor[label] != -1:
                old_id = exist_armor[label]
                if self.Track_value[int(track_id)][label] < self.Track_value[int(old_id)][label]:
                    self.Track_value[int(track_id)][label] = 0
                    label = "NULL"
                else:
                    self.Track_value[int(old_id)][label] = 0
                    old_id_index = self.id_candidate[old_id]
                    if old_id_index < len(draw_candidate):
                        draw_candidate[old_id_index][5] = "NULL"
                    exist_armor[label] = track_id
            else:
                if label < len(exist_armor):
                    exist_armor[label] = track_id

            # 判断投票是否有效（所有类别计数相同 = 无有效投票）
            pd = self.Track_value[int(track_id)][0]
            same = True
            for j in range(self.class_num - 1):
                if pd != self.Track_value[int(track_id)][j + 1]:
                    same = False
                    break
            if not same and label != "NULL":
                label = str(self.labels[label])
            else:
                label = "NULL"

            x_left = int(x - w / 2)
            y_left = int(y - h / 2)
            x_right = int(x + w / 2)
            y_right = int(y + h / 2)
            xywh_box = [x, y, w, h]
            xyxy_box = [x_left, y_left, x_right, y_right]
            draw_candidate.append(
                [track_id, x_left, y_left, x_right, y_right, label])
            zip_results.append([xyxy_box, xywh_box, track_id, label])
            self.id_candidate[track_id] = index
            index += 1

        # 在图像上画出检测结果
        for box in draw_candidate:
            tid, x1, y1, x2, y2, lbl = box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 128, 0), 3)
            cv2.putText(frame, str(lbl), (x1 - 10, y2 + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 122), 2)
            cv2.putText(frame, str(tid), (x2 + 5, y2 + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 122), 2)

        self.loop_times += 1
        return frame, zip_results

    def reset_tracking_state(self):
        """重置投票表和状态（视频循环时调用，避免状态残留）。"""
        for i in range(self._max_track_id):
            self.Track_value[i] = [0] * self.class_num
            self.Status[i] = 0
        self.id_candidate = [0] * self._max_track_id

        # 重置 ByteTrack 跟踪器
        if (hasattr(self.model_stage1, 'predictor')
                and self.model_stage1.predictor is not None):
            if hasattr(self.model_stage1.predictor, 'trackers'):
                self.model_stage1.predictor.trackers = None


class _PrintLogger:
    """简易日志后端，当未提供 ROS logger 时使用 print 输出。"""

    @staticmethod
    def info(msg):
        print(f"[INFO] {msg}")

    @staticmethod
    def warn(msg):
        print(f"[WARN] {msg}")

    @staticmethod
    def error(msg):
        print(f"[ERROR] {msg}")
