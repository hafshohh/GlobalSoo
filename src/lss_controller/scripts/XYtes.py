#!/usr/bin/env python
import serial
import time
import sys

class MotorRudderTester:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
    
    def connect(self):
        try:
            self.ser = serial.Serial(self.port, baudrate=self.baudrate, timeout=1)
            time.sleep(0.5)
            print("Connected to %s" % self.port)
            return True
        except serial.SerialException as e:
            print("Failed to connect: %s" % e)
            return False
    
    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Disconnected")
    
    def send_command(self, motor_pwm, rudder_pwm):
        if not self.ser or not self.ser.is_open:
            print("Serial not connected")
            return False
        
        try:
            data = b'its'
            data += motor_pwm.to_bytes(2, byteorder='little', signed=False)
            data += rudder_pwm.to_bytes(2, byteorder='little', signed=False)
            self.ser.write(data)
            print("Sent: Motor=%d, Rudder=%d" % (motor_pwm, rudder_pwm))
            return True
        except Exception as e:
            print("Send failed: %s" % e)
            return False
    
    def test_motor(self, pwm=1500, duration=2):
        print("\n=== Testing Motor (PWM=%d) ===" % pwm)
        self.send_command(pwm, 1500)
        time.sleep(duration)
        self.send_command(1500, 1500)
        print("Motor test complete\n")
    
    def test_rudder(self, pwm=1500, duration=2):
        print("\n=== Testing Rudder (PWM=%d) ===" % pwm)
        self.send_command(1500, pwm)
        time.sleep(duration)
        self.send_command(1500, 1500)
        print("Rudder test complete\n")
    
    def test_sequence(self):
        print("\n=== Running Test Sequence ===")
        print("1. Motor Forward (50%)")
        self.send_command(1680, 1500)
        time.sleep(3)
        
        print("2. Motor Backward (50%)")
        self.send_command(1320, 1500)
        time.sleep(3)
        
        print("3. Rudder Left")
        self.send_command(1500, 1700)
        time.sleep(2)
        
        print("4. Rudder Right")
        self.send_command(1500, 1300)
        time.sleep(2)
        
        print("5. Stop All")
        self.send_command(1500, 1500)
        print("\nSequence complete\n")
    
    def interactive_mode(self):
        print("\n=== Interactive Mode ===")
        print("Commands:")
        print("  m <pwm>     - Test motor (1100-2000)")
        print("  r <pwm>     - Test rudder (1100-2000)")
        print("  s           - Stop (neutral)")
        print("  seq         - Run test sequence")
        print("  q           - Quit")
        print("\nPWM Range: 1100-2000 (1500=neutral)")
        
        while True:
            try:
                cmd = input("\n> ").strip().lower()
                
                if cmd == 'q':
                    break
                elif cmd == 's':
                    self.send_command(1500, 1500)
                elif cmd == 'seq':
                    self.test_sequence()
                elif cmd.startswith('m '):
                    pwm = int(cmd.split()[1])
                    self.test_motor(pwm, 2)
                elif cmd.startswith('r '):
                    pwm = int(cmd.split()[1])
                    self.test_rudder(pwm, 2)
                else:
                    print("Invalid command")
            except (ValueError, IndexError):
                print("Invalid input")
            except KeyboardInterrupt:
                break
        
        self.send_command(1500, 1500)

def main():
    tester = MotorRudderTester()
    
    if not tester.connect():
        sys.exit(1)
    
    try:
        if len(sys.argv) > 1:
            if sys.argv[1] == 'seq':
                tester.test_sequence()
            elif sys.argv[1] == 'm' and len(sys.argv) > 2:
                tester.test_motor(int(sys.argv[2]))
            elif sys.argv[1] == 'r' and len(sys.argv) > 2:
                tester.test_rudder(int(sys.argv[2]))
        else:
            tester.interactive_mode()
    except KeyboardInterrupt:
        print("\n\nInterrupted")
    finally:
        tester.send_command(1500, 1500)
        tester.disconnect()

if __name__ == '__main__':
    main()
