#!/usr/bin/env python3
"""
lss_vision - Vision Node (ROS 1 / Intel NUC)
==============================================
Optimasi khusus Intel NUC menggunakan Intel OpenVINO via Ultralytics.
Menggunakan multi-threading agar inference tidak memblokir pengambilan frame kamera.
"""

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Bool, Int32

import cv2
import numpy as np
import threading
import time
import os
import sys
from collections import deque
from pathlib import Path

# --- Setup path agar utils bisa di-import ---
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from ultralytics import YOLO
from utils.visual import BBoxVisualization
from utils.kalman_tracker import SimpleKalmanTracker, xyxy_to_cxcywh, cxcywh_to_xyxy

# ============================================================
#  KONFIGURASI UTAMA
#  Ubah parameter di sini sesuai kondisi lapang Anda
# ============================================================
CONFIG = {
    # --- Kamera ---
    'camera_device'  : 0,        # Index kamera USB (0 = kamera pertama)
    'width'          : 640,       # Lebar resolusi kamera
    'height'         : 480,       # Tinggi resolusi kamera
    'fps'            : 30,        # Target FPS kamera
    'debug'          : True,     # True = tampilkan jendela kamera di layar (hanya di laptop)

    # --- Model YOLO ---
    # Untuk Intel NUC: Export dulu ke OpenVINO dengan:
    #   yolo export model=models/best.pt format=openvino
    # lalu ubah model_path ke: 'models/best_openvino_model'
    'model_path'     : 'models/lesgo_openvino_model',  # <-- Sudah pakai format super kencang OpenVINO

    # --- Parameter Deteksi ---
    'imgsz'          : 416,       # Ukuran gambar tensor wajib sama dengan ukuran model saat training
    'conf'           : 0.50,      # Ambang batas keyakinan (0.0-1.0)
    'iou'            : 0.50,      # Ambang batas IoU untuk NMS
    'agnostic_nms'   : False,
    'half'           : False,     # FP16 (hanya efektif jika ada NVIDIA GPU)
    'verbose'        : False,     # Tampilkan log inference di terminal

    # --- Parameter Misi ---
    'min_buoy_area'  : 1000,      # Area minimum piksel pelampung (trigger menghindar)
    'min_box_area'   : 480,       # Area minimum piksel kotak imaging

    # --- RC / Yaw Range ---
    'rc_min'         : [1000, 1300],  # [Floating Ball, Imaging/Docking]
    'rc_mid'         : 1500,
    'rc_max'         : [2000, 1700],

    # --- Metode Obstacle Avoidance ---
    # Kalman Filter aktif → gunakan Piksel Area (lebih stabil di atas air berombak)
    # Set ke True hanya jika Kalman TIDAK dipakai dan kondisi kamera sangat stabil
    'use_distance_based'   : False,
    'camera_height'        : 0.50,   # Tinggi kamera dari permukaan air (meter)
    'camera_tilt_deg'      : 45.0,   # Sudut kemiringan kamera (derajat)
    'focal_length_y'       : 320.0,  # Focal length sumbu Y (piksel)
    'principal_point_y'    : 240.0,  # Koordinat principal point Y (piksel)
    'trigger_distance'     : 1.0,    # Jarak trigger snapshot/menghindar (meter)

    # --- Kalman Filter ---
    'use_kalman'           : True,   # Aktifkan Kalman Filter untuk smoothing Bounding Box
    'kalman_max_missed'    : 15,     # Berapa frame objek boleh hilang sebelum tracker direset
}


def setup_openvino_env():
    """
    Mengkonfigurasi environment variables untuk akselerasi Intel OpenVINO.
    Memanfaatkan CPU threads secara maksimal di Intel NUC.
    """
    # Jumlah thread OpenMP untuk Intel CPU (sesuaikan dengan jumlah core NUC Anda)
    # Intel NUC 12 biasanya punya 16 threads (8P+8E cores), NUC 11 = 8 threads
    cpu_cores = os.cpu_count() or 8
    os.environ['OMP_NUM_THREADS']       = str(cpu_cores)
    os.environ['OPENBLAS_NUM_THREADS']  = str(cpu_cores)
    os.environ['MKL_NUM_THREADS']       = str(cpu_cores)

    rospy.loginfo(f"[Vision] Intel OpenVINO env diset, menggunakan {cpu_cores} CPU threads")


class VisionNode:
    def __init__(self):
        rospy.init_node('lss_vision_node', anonymous=False)
        rospy.loginfo("[Vision] Memulai lss_vision_node...")

        # --- Setup Optimasi Intel NUC ---
        setup_openvino_env()

        # --- Load Model YOLO / OpenVINO ---
        model_path = os.path.join(str(ROOT), CONFIG['model_path'])
        rospy.loginfo(f"[Vision] Memuat model dari: {model_path}")
        try:
            self.model = YOLO(model=model_path, task='detect')
            self.cls_dict = self.model.names
            rospy.loginfo(f"[Vision] Model berhasil dimuat! Kelas: {self.cls_dict}")
        except Exception as e:
            rospy.logerr(f"[Vision] GAGAL memuat model: {e}")
            rospy.signal_shutdown("Model gagal dimuat")
            return

        # --- Warm-up Model (mengurangi latensi frame pertama) ---
        self._warmup_model()

        # --- Inisialisasi Visualisasi BBox ---
        self.vis = BBoxVisualization(self.cls_dict)
        # Sinkronkan parameter jarak ke vis
        self.vis.use_distance_based    = CONFIG['use_distance_based']
        self.vis.camera_height         = CONFIG['camera_height']
        self.vis.camera_tilt_deg       = CONFIG['camera_tilt_deg']
        self.vis.focal_length_y        = CONFIG['focal_length_y']
        self.vis.principal_point_y     = CONFIG['principal_point_y']
        self.vis.trigger_distance      = CONFIG['trigger_distance']
        self.vis.v_horizon = (
            self.vis.principal_point_y
            - self.vis.focal_length_y
            * np.tan(np.deg2rad(self.vis.camera_tilt_deg))
        )

        # --- State Variabel Kontrol ---
        self.auto_mode       = True
        self.gps_control     = False
        self.compass_control = False
        self.vision_control  = False
        self.lidar           = False
        self.is_surface      = False

        # --- State Variabel Misi ---
        self.mission    = 'Floating Ball'
        self.docking    = 'green_dock'
        self.left_buoy  = 'green_buoy'
        self.right_buoy = 'red_buoy'

        # --- FPS Tracker ---
        self.fps             = 0.0
        self.frame_count     = 0
        self.fps_update_time = time.time()

        # --- Threading & Buffer ---
        self.frame_buffer   = deque(maxlen=3)   # Simpan maks 3 frame, buang yang lama
        self.latest_result  = None
        self.result_lock    = threading.Lock()
        self.running        = True
        self.stats          = {'processed': 0, 'dropped': 0}

        # --- Kalman Filter Trackers (1 tracker per kelas objek) ---
        self.trackers       = {}   # Dict {class_id: SimpleKalmanTracker}

        # --- Setup Kamera ---
        cam_idx = CONFIG['camera_device']
        
        # Paksa menggunakan backend murni Video4Linux (V4L2)
        self.cam = cv2.VideoCapture(cam_idx, cv2.CAP_V4L2)
        
        if not self.cam.isOpened():
            rospy.logerr(f"[Vision] Kamera /dev/video{cam_idx} tidak bisa dibuka!")
            rospy.signal_shutdown("Kamera tidak tersedia")
            return

        # Konfigurasi kamera untuk performa optimal
        self.cam.set(cv2.CAP_PROP_BUFFERSIZE,  1)                                    # Buffer minimum agar frame selalu fresh
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH,  CONFIG['width'])
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, CONFIG['height'])
        self.cam.set(cv2.CAP_PROP_FPS,          CONFIG['fps'])
        self.cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))    # Format MJPG = lebih cepat dari YUYV
        rospy.loginfo(f"[Vision] Kamera /dev/video{cam_idx} terbuka (MJPG {CONFIG['width']}x{CONFIG['height']} @ {CONFIG['fps']}fps)")

        # --- ROS Subscribers ---
        rospy.Subscriber('/robot/mode',            Bool,   self._cb_mode)
        rospy.Subscriber('/robot/control/gps',     Bool,   self._cb_gps_control)
        rospy.Subscriber('/robot/control/compass', Bool,   self._cb_compass_control)
        rospy.Subscriber('/robot/control/vision',  Bool,   self._cb_vision_control)
        rospy.Subscriber('/robot/lidar',           Bool,   self._cb_lidar)
        rospy.Subscriber('/robot/mission',         String, self._cb_mission)
        rospy.Subscriber('/detect/docking',        String, self._cb_docking)
        rospy.Subscriber('/detect/left_buoy',      String, self._cb_left_buoy)
        rospy.Subscriber('/detect/right_buoy',     String, self._cb_right_buoy)
        rospy.Subscriber('/mission/reset',         Bool,   self._cb_reset_mission)

        # --- ROS Publishers ---
        self.pub_nav_vision     = rospy.Publisher('/robot/vision/navigation',  CompressedImage, queue_size=1)
        self.pub_surface_image  = rospy.Publisher('/robot/image/surface',      CompressedImage, queue_size=1)
        self.pub_surface_mission= rospy.Publisher('/robot/mission/surface',    Bool,            queue_size=1)
        self.pub_obj_detected   = rospy.Publisher('/detect/object/bool',       Bool,            queue_size=1)
        self.pub_raw_yaw        = rospy.Publisher('/robot/vision/raw_yaw',     Int32,           queue_size=1)
        self.pub_snapshot       = rospy.Publisher('/robot/snapshot',           Bool,            queue_size=1)

        # --- Mulai Background Inference Thread ---
        self._inference_thread = threading.Thread(
            target=self._inference_worker,
            daemon=True,
            name='InferenceWorker'
        )
        self._inference_thread.start()
        rospy.loginfo("[Vision] Inference thread dimulai.")

        rospy.loginfo("[Vision] lss_vision_node siap berjalan!")

    # ================================================================
    #  MODEL WARM-UP
    # ================================================================
    def _warmup_model(self):
        rospy.loginfo("[Vision] Melakukan warm-up model...")
        dummy = np.zeros((CONFIG['height'], CONFIG['width'], 3), dtype=np.uint8)
        try:
            self.model(dummy, imgsz=CONFIG['imgsz'], verbose=False)
            rospy.loginfo("[Vision] Warm-up selesai.")
        except Exception as e:
            rospy.logwarn(f"[Vision] Warm-up gagal (tidak kritis): {e}")

    # ================================================================
    #  INFERENCE WORKER THREAD
    #  Berjalan di background — mengambil frame dari buffer dan predict
    # ================================================================
    def _inference_worker(self):
        """
        Thread terpisah untuk menjalankan model YOLO.
        Dengan begitu loop kamera utama tidak terhenti saat model sedang berpikir.
        """
        while self.running and not rospy.is_shutdown():
            if len(self.frame_buffer) == 0:
                time.sleep(0.001)   # Tidak ada frame, tunggu sebentar
                continue

            try:
                frame = self.frame_buffer.popleft()
                result = self.model(
                    frame,
                    imgsz       = CONFIG['imgsz'],
                    conf        = CONFIG['conf'],
                    iou         = CONFIG['iou'],
                    agnostic_nms= CONFIG['agnostic_nms'],
                    half        = CONFIG['half'],
                    verbose     = CONFIG['verbose']
                )
                with self.result_lock:
                    self.latest_result = result[0]
                    self.stats['processed'] += 1

            except Exception as e:
                rospy.logerr_throttle(5.0, f"[Vision] Error inference: {e}")

    # ================================================================
    #  SUBSCRIBER CALLBACKS
    # ================================================================
    def _cb_mode(self, msg):             self.auto_mode       = msg.data
    def _cb_gps_control(self, msg):      self.gps_control     = msg.data
    def _cb_compass_control(self, msg):  self.compass_control = msg.data
    def _cb_vision_control(self, msg):   self.vision_control  = msg.data
    def _cb_lidar(self, msg):            self.lidar           = msg.data
    def _cb_mission(self, msg):          self.mission         = msg.data
    def _cb_docking(self, msg):          self.docking         = msg.data
    def _cb_left_buoy(self, msg):        self.left_buoy       = msg.data
    def _cb_right_buoy(self, msg):       self.right_buoy      = msg.data

    def _cb_reset_mission(self, msg):
        if msg.data:
            self.mission    = ''
            self.is_surface = False
            rospy.loginfo("[Vision] Misi direset!")

    # ================================================================
    #  HELPER: PUBLISH COMPRESSED IMAGE (JPEG, hemat bandwidth)
    # ================================================================
    def _publish_compressed(self, frame, publisher, quality=80):
        """Mengubah frame OpenCV menjadi JPEG dan publish ke ROS topic."""
        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format       = 'jpeg'
        msg.data         = buf.tobytes()
        publisher.publish(msg)

    # ================================================================
    #  HANDLER: SURFACE IMAGING (ambil foto saat kapal mau mendekat)
    # ================================================================
    def _handle_surface_imaging(self, frame, snapshot):
        if not self.auto_mode:
            return
        if self.gps_control or self.compass_control or not self.vision_control:
            return
        if self.mission != 'Surface Imaging':
            return

        # Gunakan lidar sebagai trigger jika tersedia, jika tidak pakai snapshot dari AI
        proximity_trigger = self.lidar if CONFIG.get('use_lidar', False) else snapshot
        if proximity_trigger and not self.is_surface:
            self.is_surface = True
            self._publish_compressed(frame, self.pub_surface_image, quality=90)
            rospy.loginfo("[Vision] Surface Imaging triggered — foto terkirim!")

    # ================================================================
    #  PROCESSOR: Menggabungkan result AI + visual.py + publish ROS
    # ================================================================
    def _process_frame(self, frame):
        """
        1. Kirim frame ke buffer inference worker
        2. Ambil result terbaru dari worker
        3. Olah dengan BBoxVisualization (visual.py)
        4. Publish semua data ke ROS
        """
        # 1. Masukkan frame ke buffer worker (buang frame lama jika penuh)
        if len(self.frame_buffer) >= self.frame_buffer.maxlen:
            try:
                self.frame_buffer.popleft()
                self.stats['dropped'] += 1
            except IndexError:
                pass
        self.frame_buffer.append(frame.copy())

        # 2. Ambil result terbaru (non-blocking, bisa None jika worker belum selesai)
        with self.result_lock:
            result = self.latest_result

        # 3. Tentukan index RC berdasarkan misi
        rc_idx = 0 if self.mission == 'Floating Ball' else 1

        # 4. Ekstrak + Proses Bounding Boxes dengan atau tanpa Kalman Filter
        boxes, confs, clss = [], [], []

        if CONFIG['use_kalman']:
            # === PIPELINE KALMAN FILTER ===
            # Langkah A: Kumpulkan kandidat objek terbesar per kelas dari YOLO
            best_detections = {}
            if result is not None and hasattr(result, 'boxes') and len(result.boxes) > 0:
                raw_boxes = result.boxes.xyxy.cpu().numpy() if hasattr(result.boxes.xyxy, 'cpu') else result.boxes.xyxy.numpy()
                raw_confs = result.boxes.conf.cpu().numpy() if hasattr(result.boxes.conf, 'cpu') else result.boxes.conf.numpy()
                raw_clss  = result.boxes.cls.cpu().numpy()  if hasattr(result.boxes.cls,  'cpu') else result.boxes.cls.numpy()
                for b, c, cl in zip(raw_boxes, raw_confs, raw_clss):
                    cls_id = int(cl)
                    area   = (b[2] - b[0]) * (b[3] - b[1])
                    if cls_id not in best_detections or area > best_detections[cls_id]['area']:
                        best_detections[cls_id] = {'box': b, 'conf': c, 'area': area}

            # Langkah B: Update Kalman untuk objek yang TERDETEKSI
            active_classes = set()
            for cls_id, det in best_detections.items():
                if cls_id not in self.trackers:
                    trk = SimpleKalmanTracker()
                    trk.max_missed_frames = CONFIG['kalman_max_missed']
                    self.trackers[cls_id] = trk
                active_classes.add(cls_id)
                cxcywh         = xyxy_to_cxcywh(det['box'])
                smoothed       = self.trackers[cls_id].update(cxcywh)
                if smoothed is not None:
                    boxes.append(cxcywh_to_xyxy(smoothed))
                    confs.append(det['conf'])
                    clss.append(cls_id)

            # Langkah C: Update Kalman BUTA untuk objek yang HILANG sementara
            for cls_id, trk in list(self.trackers.items()):
                if cls_id not in active_classes:
                    simulated = trk.update(None)
                    if simulated is not None:
                        boxes.append(cxcywh_to_xyxy(simulated))
                        confs.append(0.50)  # Confidence palsu saat objek hilang
                        clss.append(cls_id)
                    else:
                        del self.trackers[cls_id]  # Tracker mati, bersihkan

        else:
            # === PIPELINE MENTAH (Tanpa Kalman) ===
            if result is not None and hasattr(result, 'boxes') and len(result.boxes) > 0:
                boxes = result.boxes.xyxy.tolist()
                confs = result.boxes.conf.tolist()
                clss  = result.boxes.cls.tolist()

        # 5. Jalankan logika visualisasi & misi dari visual.py
        display_frame, raw_yaw, snapshot, _, is_avoiding = self.vis.processor(
            frame          = frame.copy(),   # Jangan ubah frame asli
            fps            = self.fps,
            mission        = self.mission,
            docking        = self.docking,
            left_buoy      = self.left_buoy,
            right_buoy     = self.right_buoy,
            boxes          = boxes,
            confs          = confs,
            clss           = clss,
            width          = frame.shape[1],
            height         = frame.shape[0],
            batas_bawah    = CONFIG['rc_min'][rc_idx],
            batas_tengah   = CONFIG['rc_mid'],
            batas_atas     = CONFIG['rc_max'][rc_idx],
            min_buoy_area  = CONFIG['min_buoy_area'],
            min_box_area   = CONFIG['min_box_area']
        )

        # Karena user memilih Metode 1 (Area Bounding Box), indikator bahaya ROS akan hidup
        # HANYA jika visual.py memberikan status is_avoiding = True (ukuran area bahaya).
        objects_detected = is_avoiding

        # 6. Publish data numerik ke ROS
        self.pub_obj_detected.publish(Bool(data=objects_detected))
        self.pub_raw_yaw.publish(Int32(data=int(raw_yaw)))
        self.pub_snapshot.publish(Bool(data=snapshot))
        self.pub_surface_mission.publish(Bool(data=self.is_surface))

        # 7. Handle Surface Imaging
        self._handle_surface_imaging(display_frame, snapshot)

        # 8. Publish video streaming ke GCS (compressed JPEG)
        self._publish_compressed(display_frame, self.pub_nav_vision, quality=80)

        return display_frame

    # ================================================================
    #  FPS CALCULATOR
    # ================================================================
    def _update_fps(self):
        self.frame_count += 1
        now = time.time()
        elapsed = now - self.fps_update_time
        if elapsed >= 1.0:
            self.fps = self.frame_count / elapsed
            self.frame_count     = 0
            self.fps_update_time = now
            rospy.loginfo_throttle(
                3.0,
                f"[Vision] FPS: {self.fps:.1f} | "
                f"Processed: {self.stats['processed']} | "
                f"Dropped: {self.stats['dropped']}"
            )

    # ================================================================
    #  LOOP UTAMA
    # ================================================================
    def run(self):
        """Loop utama: baca kamera → proses → publish. Berjalan sampai ROS shutdown."""
        rate = rospy.Rate(CONFIG['fps'])

        while not rospy.is_shutdown() and self.running:
            success, frame = self.cam.read()
            if not success:
                rospy.logwarn_throttle(5.0, "[Vision] Gagal baca frame kamera!")
                rate.sleep()
                continue

            # Proses frame & publish ke ROS
            display_frame = self._process_frame(frame)

            # Mode DEBUG: tampilkan jendela kamera di layar laptop
            if CONFIG['debug']:
                cv2.imshow('lss_vision DEBUG', display_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    rospy.loginfo("[Vision] DEBUG window ditutup.")
                    break

            self._update_fps()
            rate.sleep()

        self._shutdown()

    # ================================================================
    #  SHUTDOWN GRACEFUL
    # ================================================================
    def _shutdown(self):
        rospy.loginfo("[Vision] Mematikan node...")
        self.running = False

        if hasattr(self, '_inference_thread') and self._inference_thread.is_alive():
            self._inference_thread.join(timeout=3.0)

        if hasattr(self, 'cam') and self.cam.isOpened():
            self.cam.release()

        cv2.destroyAllWindows()
        rospy.loginfo(
            f"[Vision] Selesai. Total processed: {self.stats['processed']}, "
            f"dropped: {self.stats['dropped']}"
        )


# ================================================================
#  ENTRY POINT
# ================================================================
if __name__ == '__main__':
    try:
        node = VisionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass