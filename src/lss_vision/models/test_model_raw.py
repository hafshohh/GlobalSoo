#!/usr/bin/env python3
"""
Raw YOLO Model Tester (Murni Bawaan Ultralytics)
=================================================
Script ini digunakan HANYA untuk mengetes ketajaman mata 
model 'best.pt' Anda dalam mendeteksi pelampung.
Tidak ada logika visual.py (garis deadzone/yaw) di sini.
Hanya menampilkan kotak (Bounding Box) asli bawaan YOLO.
"""

from ultralytics import YOLO
import cv2
import sys
from pathlib import Path

# Setup path agar relatif ke folder workspace
FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]  # lss_vision/

def main():
    model_path = str(ROOT / 'models/best.pt')
    print(f"\n[INFO] Memuat Model AI YOLO dari: {model_path}")
    
    try:
        model = YOLO(model_path)
        print(f"[INFO] Daftar Kelas Model Anda: {model.names}\n")
    except Exception as e:
        print(f"[ERROR] Gagal memuat model: {e}")
        return

    print("==============================================")
    print("Kamera Terbuka! Sedang Mencari Obyek...")
    print("Tekan tombol 'q' atau klik (x) pada jendela video untuk keluar.")
    print("==============================================\n")

    # Syntax bawaan ultralytics yang sangat praktis
    # source=0 -> Pakai kamera webcam laptop
    # show=True -> Buka jendela otomatis
    # stream=True -> Mode hemat memori untuk kamera langsung
    results = model.predict(source=0, show=True, stream=True, conf=0.5)

    # Loop penahan agar kamera tidak langsung tertutup
    for r in results:
        pass  # Jendela ditangani otomatis oleh parameter "show=True"

    print("\n[INFO] Uji coba model selesai.")

if __name__ == '__main__':
    main()
