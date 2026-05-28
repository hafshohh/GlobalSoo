#!/usr/bin/env python3
"""
Standalone Test Script untuk Laptop (TIDAK Butuh ROS)
======================================================
Script ini memungkinkan Anda mengetes kualitas model best.pt dan 
garis-garis visual algoritma visual.py LANGSUNG dari webcam laptop 
Anda tanpa perlu menginstall/menjalankan ROS sama sekali.
"""

import cv2
import time
import sys
from pathlib import Path

# Setup path agar bisa mengambil module di luar folder models
FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]  # Naik 1 tingkat ke lss_vision/
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from ultralytics import YOLO
from utils.visual import BBoxVisualization

def main():
    # --- 1. Load Model ---
    model_path = str(ROOT / 'models/lesgo.pt')
    print(f"Loading model dari: {model_path}")
    try:
        model = YOLO(model_path)
        cls_dict = model.names
        print(f"Model berhasil dimuat! Kelas: {cls_dict}")
    except Exception as e:
        print(f"GAGAL memuat model: {e}")
        return

    # --- 2. Setup Visualisasi algoritma ASV ---
    vis = BBoxVisualization(cls_dict)
    
    # [OPSIONAL] Sesuaikan ketinggian pura-pura kamera jika pakai mode Distance Imaging
    vis.camera_height = 0.50   
    vis.camera_tilt_deg = 45.0

    # --- 3. Buka USB Kamera (Webcam Laptop) ---
    cam_index = 0
    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        print("Kesalahan: Tidak dapat membuka webcam laptop!")
        return

    # Paksa frame ke resolusi 640x480 agar ringan
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # --- Variabel FPS ---
    prev_time = time.time()
    
    # --- Variabel Simulasi Misi ---
    # Ubah string misi ini jika Anda ingin mengetes manuver yang berbeda
    # Pilihan: 'Floating Ball', 'Surface Imaging', 'Underwater Imaging', 'Docking'
    simulated_mission = 'Floating Ball'

    print("\n===============================")
    print("Mulai Merekam! Tekan tombol 'q' di keyboard untuk keluar.")
    print("===============================\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Gagal membaca frame kamera.")
            break

        # Hitung FPS real-time
        current_time = time.time()
        fps = 1 / (current_time - prev_time)
        prev_time = current_time

        # --- 4. Proses AI / YOLO Inference ---
        # Berjalan di memori laptop secara real-time
        results = model(
            frame, 
            imgsz=640, 
            conf=0.5, 
            verbose=True,
             # Set True jika ingin melihat teks terminal log ultralytics
        )[0]

        # Mengekstrak kotak hasil model
        if len(results.boxes) > 0:
            boxes = results.boxes.xyxy.tolist()
            confs = results.boxes.conf.tolist()
            clss = results.boxes.cls.tolist()
        else:
            boxes, confs, clss = [], [], []

        # --- 5. Terapkan Logika Misi (visual.py) ---
        display_frame, raw_yaw, snapshot, finish = vis.processor(
            frame          = frame, 
            fps            = fps, 
            mission        = simulated_mission, 
            docking        = 'green_dock', 
            left_buoy      = 'green_buoy', 
            right_buoy     = 'red_buoy', 
            boxes          = boxes, 
            confs          = confs, 
            clss           = clss, 
            width          = frame.shape[1], 
            height         = frame.shape[0], 
            batas_bawah    = 1000, 
            batas_tengah   = 1500, 
            batas_atas     = 2000, 
            min_buoy_area  = 480, 
            min_box_area   = 480
        )

        # Anda bisa print varibel snapshot & raw_yaw ini untuk melihat logikanya
        # print(f"YAW: {raw_yaw:.2f} | Snapshot: {snapshot} | Finish: {finish}")

        # --- 6. Tampilkan ke Layar ---
        cv2.imshow('ASV Vison Test (PURE PYTHON)', display_frame)

        # Keluar saat menekan tombol 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Bersihkan sisa program
    cap.release()
    cv2.destroyAllWindows()
    print("Program ditutup secara normal.")

if __name__ == '__main__':
    main()
