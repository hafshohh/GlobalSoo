#!/usr/bin/env python3
"""
ASV Vision Simulator (Tanpa Kamera & Tanpa AI)
======================================================
Script ini secara KHUSUS dirancang untuk mengetes mentah-mentah 
logika matematika dari visual.py Anda (nilai YAW, garis deadzone, 
dan status TURN LEFT / RIGHT).

TIDAK menggunakan webcam.
TIDAK mencari objek sungguhan.
Hanya menampilkan layar hitam dengan kotak simulasi/mainan yang bergerak otomatis!
"""

import cv2
import numpy as np
import time
import math
from visual import BBoxVisualization

def create_fake_box(cx, cy, size=40):
    """Fungsi bantuan membuat format bounding box [x1, y1, x2, y2]"""
    return [cx - size, cy - size, cx + size, cy + size]

def main():
    # --- 1. Persiapan Simulasi ---
    width, height = 640, 480
    
    # Daftarkan kamus kelas buatan (fake class)
    cls_dict = {
        0: 'blue_box', 1: 'green_box', 2: 'green_buoy', 
        3: 'green_dock', 4: 'red_buoy', 5: 'red_dock'
    }
    
    # Inisiasi visual.py
    vis = BBoxVisualization(cls_dict)
    
    print("\n==================================")
    print("SIMULATOR VISUAL.PY DIBUKA!")
    print("Misi: Floating Ball (Melewati 2 Pelampung)")
    print("Tekan tombol 'q' untuk keluar.")
    print("==================================\n")

    # Waktu untuk menggerakkan kotak otomatis
    t = 0.0

    while True:
        # Buat canvas latar belakang hitam
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        
        # --- 2. Simulasi Pergerakan Pelampung (Animasi Pakai Sinus Cosinus) ---
        t += 0.05
        
        # Gerakkan pelampung kiri (hijau) ke kanan-kiri
        left_cx = int(200 + 150 * math.sin(t))
        left_cy = 300
        
        # Gerakkan pelampung kanan (merah)
        right_cx = int(450 + 100 * math.cos(t * 0.8))
        right_cy = 320
        
        # --- 3. Membungkus datanya menyerupai tebakan YOLO ---
        boxes = [
            create_fake_box(left_cx, left_cy),
            create_fake_box(right_cx, right_cy)
        ]
        confs = [0.95, 0.99]      # Fake confidence 95% & 99%
        clss  = [2, 4]            # 2='green_buoy', 4='red_buoy' dari cls_dict di atas
        
        # Dapatkan nilai FPS fiktif
        fake_fps = 30.0

        # --- 4. KIRIM DATA PALSU KE VISUAL.PY -------------
        # Di sinilah kita menguji kemurnian "otak visual" Anda!
        display_frame, raw_yaw, snapshot, is_finish = vis.processor(
            frame          = frame, 
            fps            = fake_fps, 
            mission        = "Floating Ball",  
            docking        = "green_dock", 
            left_buoy      = "green_buoy", 
            right_buoy     = "red_buoy", 
            boxes          = boxes, 
            confs          = confs, 
            clss           = clss, 
            width          = width, 
            height         = height, 
            batas_bawah    = 1000, 
            batas_tengah   = 1500, 
            batas_atas     = 2000, 
            min_buoy_area  = 100,  # Dikecilkan biar kotaknya tidak ditolak
            min_box_area   = 100
        )

        # Anda bisa print hasil YAW buatan di terminal
        # print(f"Output Logic  --> YAW Tembakan: {int(raw_yaw)}")

        # --- 5. Tampilkan Hasil ---
        cv2.imshow('Pure Logic Simulator - visual.py', display_frame)

        # Keluar jika tekan 'q'
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    print("Simulator ditutup.")

if __name__ == '__main__':
    main()
