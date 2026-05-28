#!/usr/bin/env python3
"""
utils/kalman_tracker.py
=======================
Kalman Filter Tracker untuk Bounding Box Objek pada Sistem Visi ASV.
Menghaluskan koordinat (cx, cy, w, h) yang dikeluarkan oleh YOLO
agar tidak bergetar akibat guncangan ombak / pergerakan kapal.

Model: Constant Velocity (8D State, 4D Measurement)
State  : [cx, cy, w, h, dx, dy, dw, dh]
Measure: [cx, cy, w, h]
"""

import cv2
import numpy as np


class SimpleKalmanTracker:
    """
    Pelacak Bounding Box per objek dengan Kalman Filter OpenCV.
    Satu instance tracker = satu kelas objek (misal: satu untuk red_buoy).
    """

    def __init__(self):
        # 8 State Variables: (cx, cy, w, h, dx, dy, dw, dh)
        # 4 Measurement Variables: (cx, cy, w, h)
        self.kf = cv2.KalmanFilter(8, 4)

        # Transition Matrix (F) — Model Gerak Kecepatan Konstan
        self.kf.transitionMatrix = np.array([
            [1, 0, 0, 0, 1, 0, 0, 0],
            [0, 1, 0, 0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0, 0, 1, 0],
            [0, 0, 0, 1, 0, 0, 0, 1],
            [0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 1]
        ], np.float32)

        # Measurement Matrix (H) — Kita hanya bisa mengamati posisi & dimensi
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0]
        ], np.float32)

        # Noise Covariance — Tuning kehalusan smoothing
        # processNoiseCov kecil   = KF sangat percaya model fisik (gerak konstan)
        # measurementNoiseCov lebih besar = KF "kurang percaya" sensor kamera mentah
        cv2.setIdentity(self.kf.processNoiseCov, 1e-2)
        cv2.setIdentity(self.kf.measurementNoiseCov, 1e-1)
        cv2.setIdentity(self.kf.errorCovPost, 1.0)

        self.is_initialized   = False
        self.missed_frames    = 0
        self.max_missed_frames = 15  # Jika hilang > 15 frame berturut, tracker direset

    def update(self, measurement=None):
        """
        Masukkan pengukuran YOLO, keluarkan koordinat yang sudah dihaluskan.

        Args:
            measurement: list [cx, cy, w, h] dari YOLO, atau None jika objek tidak terdeteksi.

        Returns:
            np.array [cx, cy, w, h] hasil prediksi/koreksi, atau None jika tracker mati.
        """
        # --- LANGKAH 1: Prediksi posisi berdasarkan kecepatan frame sebelumnya ---
        if self.is_initialized:
            predicted = self.kf.predict()
        else:
            predicted = np.zeros((8, 1), np.float32)

        if measurement is not None:
            # --- LANGKAH 2: Koreksi prediksi dengan pengukuran YOLO aktual ---
            self.missed_frames = 0
            meas = np.array(measurement, dtype=np.float32).reshape(4, 1)

            if not self.is_initialized:
                # Inisialisasi state dengan deteksi pertama
                self.kf.statePost          = np.zeros((8, 1), np.float32)
                self.kf.statePost[0:4]     = meas
                self.is_initialized        = True
                return meas.flatten()
            else:
                estimated = self.kf.correct(meas)
                return estimated[0:4].flatten()

        else:
            # --- LANGKAH 3: Tebakan buta saat objek hilang dari kamera ---
            self.missed_frames += 1
            if self.missed_frames > self.max_missed_frames:
                self.is_initialized = False   # Relakan, objek benar-benar hilang
                return None
            if self.is_initialized:
                return predicted[0:4].flatten()
            return None


# ============================================================
#  HELPER FUNCTIONS — Konversi Format Bounding Box
# ============================================================

def xyxy_to_cxcywh(box):
    """Konversi [x1, y1, x2, y2] → [cx, cy, w, h]"""
    x1, y1, x2, y2 = box
    return [(x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1]


def cxcywh_to_xyxy(box):
    """Konversi [cx, cy, w, h] → [x1, y1, x2, y2]"""
    cx, cy, w, h = box
    return [cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2]