#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
import serial
import time


class SteeringNode:
    def __init__(self):
        rospy.init_node('control', anonymous=False)

        rudder_max_deg      = rospy.get_param('~rudder_max_deg', 40.0)
        self.rudder_max_rad = np.radians(rudder_max_deg)
        rospy.loginfo("[Control] Rudder max: %.1f deg (%.3f rad)", rudder_max_deg, self.rudder_max_rad)

        port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
        baud = rospy.get_param('~baudrate', 115200)
        self.ser = None
        try:
            # Bug #2 Fix: set dtr=False SEBELUM open() agar Arduino/MCU tidak reset
            self.ser = serial.Serial()
            self.ser.port     = port
            self.ser.baudrate = baud
            self.ser.timeout  = 0.1
            self.ser.dtr      = False
            self.ser.rts      = False
            self.ser.open()
            rospy.loginfo("[Control] Serial terbuka: %s @ %d baud (DTR=False)", port, baud)

            rospy.loginfo("[Control] Menunggu MCU boot (2s)...")
            rospy.sleep(2.0)

            # Bug #1 Fix: kirim sinyal arming ESC sebelum menerima throttle
            self._send_arm_signal()

        except serial.SerialException as e:
            rospy.logerr("[Control] Gagal buka serial %s: %s", port, e)

        self.control_sub = rospy.Subscriber('/autonomy/pathfollowing', Twist,
                                            self.get_actuator_input, queue_size=1)
        rospy.on_shutdown(self._shutdown)
        rospy.loginfo("[Control] Siap menerima perintah dari /autonomy/pathfollowing")

    def _send_arm_signal(self):
        """Kirim sinyal neutral (1500,1500) selama 2 detik untuk arming ESC."""
        if self.ser is None or not self.ser.is_open:
            return
        neutral  = b'its'
        neutral += (1500).to_bytes(2, byteorder='little', signed=False)
        neutral += (1500).to_bytes(2, byteorder='little', signed=False)

        rospy.loginfo("[Control] ESC arming — kirim neutral selama 2 detik...")
        for i in range(40):          # 40 × 50ms = 2 detik sinyal stabil
            self.ser.write(neutral)
            time.sleep(0.05)
            if (i + 1) % 10 == 0:
                rospy.loginfo("[Control]   arming %d/%d ...", i + 1, 40)

        rospy.loginfo("[Control] ESC arming selesai — siap menerima throttle")

    def _shutdown(self):
        if self.ser and self.ser.is_open:
            neutral  = b'its'
            neutral += (1500).to_bytes(2, byteorder='little', signed=False)
            neutral += (1500).to_bytes(2, byteorder='little', signed=False)
            try:
                self.ser.write(neutral)
                self.ser.flush()
            except Exception:
                pass
            self.ser.close()
            rospy.loginfo("[Control] Serial ditutup")

    def percent_to_pwm(self, thrust_percentage):
        clamp = lambda x: max(min(x, 2000), 1000)
        if abs(thrust_percentage) < 0.001:
            return 1500
        if thrust_percentage > 0:
            return clamp(int(1528 + (1832-1528)*thrust_percentage))
        else:
            return clamp(int(1472 + (1472-1100)*thrust_percentage))

    def rad_to_pwm(self, rad):
        clamp = lambda x: max(min(x, 2000), 1000)
        return clamp(1500 + int(-rad * 500 / self.rudder_max_rad))

    def get_actuator_input(self, target):
        if self.ser is None or not self.ser.is_open:
            rospy.logwarn_throttle(5.0, "Serial tidak tersedia — perintah motor diabaikan")
            return
        speed = self.percent_to_pwm(target.linear.x)
        angle = self.rad_to_pwm(target.angular.z)
        rospy.loginfo_throttle(1.0, "Motor: %d, Rudder: %d", speed, angle)
        try:
            to_send_data = b'its'
            to_send_data += speed.to_bytes(2, byteorder='little', signed=False)
            to_send_data += angle.to_bytes(2, byteorder='little', signed=False)
            self.ser.write(to_send_data)
        except serial.SerialException as e:
            rospy.logerr("Serial exception: %s", e)

def main():
    try:
        node = SteeringNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
