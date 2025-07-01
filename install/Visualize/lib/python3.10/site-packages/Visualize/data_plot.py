import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from collections import deque
import time
import math

class DataPlot(Node):
    def __init__(self):
        super().__init__('data_plot')
        
        # Khởi tạo subscriber
        self.subscription_position = self.create_subscription(
            Float32, 'motor/position', self.position_callback, 10)
        self.subscription_torque_setpoint = self.create_subscription(
            Float32, 'motor/torque_setpoint', self.torque_setpoint_callback, 10)
        self.subscription_torque_actual = self.create_subscription(
            Float32, 'motor/torque_actual', self.torque_actual_callback, 10)
        
        # Khởi tạo dữ liệu cho đồ thị
        self.time_window = 10  # Hiển thị 10 giây gần nhất
        self.start_time = time.time()
        
        self.time_data = deque(maxlen=10000)  # Tăng lên 10000 mẫu (~10 giây ở 1ms)
        self.position_data = deque(maxlen=10000)
        self.torque_setpoint_data = deque(maxlen=10000)
        self.torque_actual_data = deque(maxlen=10000)


        # Tạo figure
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(8, 6))
        
        # Khởi tạo timer để update đồ thị
        self.timer = self.create_timer(0.1, self.update_plot)
    
    def position_callback(self, msg):
        self.time_data.append(time.time() - self.start_time)
        self.position_data.append(msg.data * (math.pi / 4))  # Chuyển đổi sang rad
    
    def torque_setpoint_callback(self, msg):
        self.torque_setpoint_data.append(msg.data)
        #self.get_logger().info(f'Received torque setpoint: {msg.data}')
    
    def torque_actual_callback(self, msg):
        self.torque_actual_data.append(msg.data)
        # self.get_logger().info(f'Received torque actual: {msg.data}')
    
    def update_plot(self):
        if len(self.time_data) == 0:
            return
    
        min_len = min(len(self.time_data), len(self.position_data), len(self.torque_setpoint_data), len(self.torque_actual_data))
    
        time_trimmed = list(self.time_data)[:min_len]
        position_trimmed = list(self.position_data)[:min_len]
        torque_setpoint_trimmed = list(self.torque_setpoint_data)[:min_len]
        torque_actual_trimmed = list(self.torque_actual_data)[:min_len]

        self.ax1.clear()
        self.ax2.clear()
    
        self.ax1.plot(time_trimmed, position_trimmed, 'r-', label='Position')
        self.ax1.set_ylabel('Position (rad)')
        self.ax1.legend()
        self.ax1.set_xlim(max(0, time_trimmed[-1] - self.time_window), time_trimmed[-1])
    
        self.ax2.plot(time_trimmed, torque_setpoint_trimmed, 'b-', label='Set Torque')
        self.ax2.plot(time_trimmed, torque_actual_trimmed, 'r-', label='Measured Torque')
        self.ax2.set_ylabel('Torque (gf·cm)')
        self.ax2.legend()
        self.ax2.set_xlim(max(0, time_trimmed[-1] - self.time_window), time_trimmed[-1])
    
        plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = DataPlot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
