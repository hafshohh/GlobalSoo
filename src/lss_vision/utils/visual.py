import numpy as np
import cv2

class BBoxVisualization():
    def __init__(self, cls_dict):
        self.cls_dict = cls_dict  # ['blue_box', 'green_box', 'green_buoy', 'green_dock', 'red_buoy', 'red_dock']

        # Constants
        self.FONT = cv2.FONT_HERSHEY_SIMPLEX
        self.FONT_SIZE = 0.75
        self.THICKNESS = 2
        self.CIRCLE_SIZE = 7
        self.BLUE_COLOR = (255, 155, 0)
        self.GREEN_COLOR = (0, 255, 0)
        self.RED_COLOR = (0, 0, 255)
        self.PURPLE_COLOR = (255, 55, 125)
        self.ORANGE_COLOR = (0, 125, 255)
        self.BLACK_COLOR = (0, 0, 0)
        self.WHITE_COLOR = (255, 255, 255)

        # Variables
        self.midpoint_deadzone = 50
        self.single_deadzone = int(self.midpoint_deadzone * 4.5)

        # Distance calculation parameters (Ground-Plane Projection)
        self.use_distance_based = False
        self.camera_height = 0.60       # H: camera height above water (meters)
        self.camera_tilt_deg = 45.0     # theta: camera tilt angle (degrees)
        self.focal_length_y = 320.0     # fy: focal length in y-axis (pixels)
        self.principal_point_y = 240.0  # cy: principal point y-coordinate (pixels)
        self.trigger_distance = 0.75    # Distance threshold for snapshot trigger (meters)

        # Precompute horizon line pixel
        self.v_horizon = self.principal_point_y - self.focal_length_y * np.tan(np.deg2rad(self.camera_tilt_deg))

    def calculate_distance_groundplane(self, bbox_bottom_y):
        """
        Calculate distance using Ground-Plane Projection (Simplified Formula 2)

        Formula: D ≈ (H * fy) / (v - v_horizon)

        Where:
        - H: camera height above water surface
        - fy: focal length in y-axis
        - v: bottom pixel of bbox (object touching water)
        - v_horizon: horizon line pixel = cy - fy * tan(theta)

        Args:
            bbox_bottom_y: Bottom y-coordinate of bounding box (pixels)

        Returns:
            distance: Distance to object on water surface (meters)
        """
        # Avoid division by zero
        denominator = bbox_bottom_y - self.v_horizon
        if denominator <= 0:
            return float('inf')

        # Ground-Plane Projection formula
        distance = (self.camera_height * self.focal_length_y) / denominator

        return distance

    def processor(self, frame, fps, mission, docking, left_buoy, right_buoy, boxes, confs, clss, width, height, batas_bawah, batas_tengah, batas_atas, min_buoy_area, min_box_area):
        connector_midpoint = None
        yaw = batas_tengah
        snapshot = False
        is_finish = False
        
        # Bikin dictionary dinamis dari semua obyek yang ada di AI dan yang diminta script
        all_classes = set(list(self.cls_dict.values()) + ['blue_box', 'green_box', 'green_buoy', 'green_dock', 'red_buoy', 'red_dock'])
        
        max_area = {c: 0 for c in all_classes}
        max_bbox = {c: None for c in all_classes}
        max_conf = {c: 0 for c in all_classes}
        midpoints = {c: None for c in all_classes}
        distances = {c: None for c in all_classes}

        # Detected object
        for bb, cf, cl in zip(boxes, confs, clss):
            cl = int(cl)
            x1, y1, x2, y2 = map(float, bb)
            cls_name = self.cls_dict.get(cl, 'CLS{}'.format(cl))
            midpoint = (int((x1 + x2) / 2), int((y1 + y2) / 2))
            area = (x2 - x1) * (y2 - y1)
            distance = None

            # Filter dicabut agar kotak bounding box tetap tergambar saat pelampung masih jauh (kecil), 
            # tapi nanti di bawah kita filter hanya "MENGHINDAR" kalau area > min_buoy_area

            # Calculate distance using trigonometry Ground-Plane for both imaging & avoidance
            if self.use_distance_based and cls_name in ['green_box', 'blue_box', 'green_buoy', 'red_buoy']:
                distance = self.calculate_distance_groundplane(y2)

            if cls_name in max_area and area > max_area[cls_name]:
                max_area[cls_name] = area
                max_bbox[cls_name] = (int(x1), int(y1), int(x2), int(y2))
                max_conf[cls_name] = cf
                midpoints[cls_name] = midpoint

                if distance is not None:
                    distances[cls_name] = distance

        # Snapshot logic for imaging missions
        if mission.endswith("Imaging"):
            target_box_detected = midpoints["green_box"] or midpoints["blue_box"]

            if target_box_detected is not None:
                if self.use_distance_based:
                    # Distance-based: check if any box is within trigger distance
                    for box_type in ['green_box', 'blue_box']:
                        if distances[box_type] is not None and distances[box_type] < self.trigger_distance:
                            snapshot = True
                            break
                else:
                    # Pixel-based: check if any box meets area threshold
                    for box_type in ['green_box', 'blue_box']:
                        if midpoints[box_type] is not None and max_area[box_type] >= min_box_area:
                            snapshot = True
                            break

        # Bounding boxes with the largest area
        for cls_name, bbox in max_bbox.items():
            if bbox is not None:
                x1, y1, x2, y2 = bbox
                conf = max_conf[cls_name]
                color = self.GREEN_COLOR if cls_name == 'green_buoy' else self.RED_COLOR if cls_name == 'red_buoy' else self.BLUE_COLOR
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, self.THICKNESS)
                cv2.circle(frame, midpoints[cls_name], self.CIRCLE_SIZE, self.RED_COLOR, -1)
                
                # Garis pelacak dari titik tengah menuju objek (tambahan fitur)
                cv2.line(frame, (int(width/2), int(height/2)), midpoints[cls_name], self.WHITE_COLOR, self.THICKNESS)

                # Hybrid Info Text (Jarak Meteran vs Area Piksel)
                if self.use_distance_based and distances[cls_name] is not None:
                    dist_val = distances[cls_name]
                    # Jarak makin pendek di bawah trigger = BAHAYA
                    if dist_val <= self.trigger_distance:
                        cv2.putText(frame, f"JARAK: {dist_val:.2f}m [BAHAYA]", (x1, y2+20), self.FONT, self.FONT_SIZE, self.RED_COLOR, self.THICKNESS)
                    else:
                        cv2.putText(frame, f"JARAK: {dist_val:.2f}m [AMAN]", (x1, y2+20), self.FONT, self.FONT_SIZE, self.GREEN_COLOR, self.THICKNESS)
                else:
                    # Fallback ke model klasik Area Piksel
                    area_val = int(max_area[cls_name])
                    if area_val >= min_buoy_area:
                        cv2.putText(frame, f"AREA: {area_val} [BAHAYA]", (x1, y2+20), self.FONT, self.FONT_SIZE, self.RED_COLOR, self.THICKNESS)
                    else:
                        cv2.putText(frame, f"AREA: {area_val} [AMAN]", (x1, y2+20), self.FONT, self.FONT_SIZE, self.GREEN_COLOR, self.THICKNESS)

                cv2.putText(frame, f"{cls_name} {conf:.2f}", (x1, y1-10), self.FONT, self.FONT_SIZE, color, self.THICKNESS)

        # Connection line and midpoint
        if midpoints[left_buoy] and midpoints[right_buoy]:
            connector_midpoint = (int((midpoints[left_buoy][0] + midpoints[right_buoy][0]) / 2),
                                  int((midpoints[left_buoy][1] + midpoints[right_buoy][1]) / 2))

        # Tanda saklar menghindar
        is_avoiding = False

        # Mission Handler
        if mission == "Floating Ball":
            # Hybrid Validation: Cek syarat darurat berdasarkan switch model
            if self.use_distance_based:
                kiri_valid = midpoints[left_buoy] is not None and distances[left_buoy] is not None and distances[left_buoy] <= self.trigger_distance
                kanan_valid = midpoints[right_buoy] is not None and distances[right_buoy] is not None and distances[right_buoy] <= self.trigger_distance
            else:
                kiri_valid = midpoints[left_buoy] is not None and max_area[left_buoy] >= min_buoy_area
                kanan_valid = midpoints[right_buoy] is not None and max_area[right_buoy] >= min_buoy_area

            # Gambar garis konektor jika dua-duanya valid
            if kiri_valid and kanan_valid and connector_midpoint:
                a, b = connector_midpoint
                yaw = batas_bawah + a * ((batas_atas - batas_bawah) * (1/640))
                is_avoiding = True

                cv2.line(frame, midpoints[left_buoy], midpoints[right_buoy], self.WHITE_COLOR, self.THICKNESS)
                cv2.line(frame, (int(width/2), int(height/2)), connector_midpoint, self.WHITE_COLOR, self.THICKNESS)
                cv2.circle(frame, connector_midpoint, self.CIRCLE_SIZE, self.RED_COLOR, -1)

                cv2.putText(frame, "MENGHINDAR!", (int(width/2)-80, height-50), self.FONT, 1.0, self.RED_COLOR, 3)

                if (a < int(width/2) - self.midpoint_deadzone):
                    cv2.putText(frame, "TURN LEFT", (450, 30), self.FONT, self.FONT_SIZE, self.RED_COLOR, self.THICKNESS)
                elif (a > int(width/2) + self.midpoint_deadzone):
                    cv2.putText(frame, "TURN RIGHT", (450, 30), self.FONT, self.FONT_SIZE, self.RED_COLOR, self.THICKNESS)

            elif kiri_valid or kanan_valid:
                # FREE OBSTACLE AVOIDANCE: Tidak peduli warna, menjauh ke arah yang berlawanan!
                target = left_buoy if kiri_valid else right_buoy
                a, b = midpoints[target]
                is_avoiding = True

                cv2.line(frame, (int(width/2), int(height/2)), midpoints[target], self.WHITE_COLOR, self.THICKNESS)
                cv2.circle(frame, midpoints[target], self.CIRCLE_SIZE, self.RED_COLOR, -1)
                cv2.putText(frame, "MENGHINDAR!", (int(width/2)-80, height-50), self.FONT, 1.0, self.RED_COLOR, 3)

                if a < int(width/2): 
                    # Objek di Kiri kamera -> Banting stir Kanan
                    # Semakin obj mendekati tengah, belok Kanan makin tajam
                    yaw = batas_tengah + (a / (width/2)) * (batas_atas - batas_tengah)
                    yaw = min(batas_atas, yaw)
                    cv2.putText(frame, "TURN RIGHT", (450, 30), self.FONT, self.FONT_SIZE, self.RED_COLOR, self.THICKNESS)
                else: 
                    # Objek di Kanan kamera -> Banting stir Kiri
                    # Semakin obj mendekati tengah, belok Kiri makin tajam
                    yaw = batas_tengah - ((width - a) / (width/2)) * (batas_tengah - batas_bawah)
                    yaw = max(batas_bawah, yaw)
                    cv2.putText(frame, "TURN LEFT", (450, 30), self.FONT, self.FONT_SIZE, self.RED_COLOR, self.THICKNESS)
            elif midpoints[left_buoy] is not None or midpoints[right_buoy] is not None:
                cv2.putText(frame, "JALUR AMAN (KECIL)", (int(width/2)-130, height-50), self.FONT, 1.0, self.GREEN_COLOR, 2)
            else:
                cv2.putText(frame, "JALUR AMAN (KOSONG)", (int(width/2)-140, height-50), self.FONT, 1.0, self.GREEN_COLOR, 2)

        # STANDALONE IMAGING LOGIC
        
        # elif mission == "Surface Imaging":
        #     if midpoints["green_box"] is not None:
        #         a, b = midpoints["green_box"]
        #         yaw = batas_bawah + a * ((batas_atas - batas_bawah) * (1/640))

        #         cv2.line(frame, (int(width/2), int(height/2)), midpoints["green_box"], self.WHITE_COLOR, self.THICKNESS)
        #         cv2.circle(frame, midpoints["green_box"], self.CIRCLE_SIZE, self.RED_COLOR, -1)

        #         if (a < int(width/2) - self.midpoint_deadzone):
        #             cv2.putText(frame, "TURN", (450, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
        #             cv2.putText(frame, "LEFT", (450, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
        #         elif (a > int(width/2) + self.midpoint_deadzone):
        #             cv2.putText(frame, "TURN", (450, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
        #             cv2.putText(frame, "RIGHT", (450, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)

        # elif mission == "Underwater Imaging":
        #     if midpoints["blue_box"] is not None:
        #         a, b = midpoints["blue_box"]
        #         yaw = batas_bawah + a * ((batas_atas - batas_bawah) * (1/640))

        #         cv2.line(frame, (int(width/2), int(height/2)), midpoints["blue_box"], self.WHITE_COLOR, self.THICKNESS)
        #         cv2.circle(frame, midpoints["blue_box"], self.CIRCLE_SIZE, self.RED_COLOR, -1)

        #         if (a < int(width/2) - self.midpoint_deadzone):
        #             cv2.putText(frame, "TURN", (450, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
        #             cv2.putText(frame, "LEFT", (450, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
        #         elif (a > int(width/2) + self.midpoint_deadzone):
        #             cv2.putText(frame, "TURN", (450, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
        #             cv2.putText(frame, "RIGHT", (450, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)

        # MERGED IMAGING LOGIC
        
        elif mission.endswith("Imaging"):
            target_box = midpoints["green_box"] or midpoints["blue_box"]
            if target_box is not None:
                a, b = target_box
                yaw = batas_bawah + a * ((batas_atas - batas_bawah) * (1/640))

                cv2.line(frame, (int(width/2), int(height/2)), target_box, self.WHITE_COLOR, self.THICKNESS)
                cv2.circle(frame, target_box, self.CIRCLE_SIZE, self.RED_COLOR, -1)

                if (a < int(width/2) - self.midpoint_deadzone):
                    cv2.putText(frame, "TURN", (450, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                    cv2.putText(frame, "LEFT", (450, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                elif (a > int(width/2) + self.midpoint_deadzone):
                    cv2.putText(frame, "TURN", (450, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                    cv2.putText(frame, "RIGHT", (450, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)

        elif mission == "Docking":
            is_finish = True
            target_dock = None

            if midpoints[docking] is not None:
                target_dock = midpoints[docking]
            elif docking == "green_dock":
                # Priority-based fallback: prefer green_dock > green_box > blue_box
                # Arena B detection can vary between these classes
                if midpoints["green_box"] is not None:
                    target_dock = midpoints["green_box"]
                elif midpoints["blue_box"] is not None:
                    target_dock = midpoints["blue_box"]

            if target_dock is not None:
                a, b = target_dock
                yaw = batas_bawah + a * ((batas_atas - batas_bawah) * (1/640))

                cv2.line(frame, (int(width/2), int(height/2)), target_dock, self.WHITE_COLOR, self.THICKNESS)
                cv2.circle(frame, target_dock, self.CIRCLE_SIZE, self.RED_COLOR, -1)

                if (a < int(width/2) - self.midpoint_deadzone):
                    cv2.putText(frame, "TURN", (450, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                    cv2.putText(frame, "LEFT", (450, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                elif (a > int(width/2) + self.midpoint_deadzone):
                    cv2.putText(frame, "TURN", (450, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
                    cv2.putText(frame, "RIGHT", (450, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)

        # Deadzone grid area
        cv2.circle(frame, (int(width/2), int(height/2)), self.CIRCLE_SIZE, self.RED_COLOR, -1)
        cv2.line(frame, (int(width/2)-self.single_deadzone, 0), (int(width/2)-self.single_deadzone, height), self.GREEN_COLOR, self.THICKNESS)
        cv2.line(frame, (int(width/2)+self.single_deadzone, 0), (int(width/2)+self.single_deadzone, height), self.GREEN_COLOR, self.THICKNESS)
        cv2.line(frame, (int(width/2)-self.midpoint_deadzone, 0), (int(width/2)-self.midpoint_deadzone, height), self.BLUE_COLOR, self.THICKNESS)
        cv2.line(frame, (int(width/2)+self.midpoint_deadzone, 0), (int(width/2)+self.midpoint_deadzone, height), self.BLUE_COLOR, self.THICKNESS)
        cv2.line(frame, (0,int(height/2)-self.midpoint_deadzone), (width, int(height/2)-self.midpoint_deadzone), self.RED_COLOR, self.THICKNESS)
        cv2.line(frame, (0,int(height/2)+self.midpoint_deadzone), (width, int(height/2)+self.midpoint_deadzone), self.RED_COLOR, self.THICKNESS)

        # FPS and YAW value
        cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)
        cv2.putText(frame, f"YAW: {int(yaw)}", (10, 60), self.FONT, self.FONT_SIZE, self.ORANGE_COLOR, self.THICKNESS)

        return frame, yaw, snapshot, is_finish, is_avoiding