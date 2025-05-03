import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from detect_result.msg import Location, Locations
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
import threading
import time
import cv2

class DisplayPanel(Node):

    def __init__(self):
        super().__init__('display_panel')
        self.get_logger().info('Display Panel Node is running')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=3
        )
        self.locations = Locations()
        self.map = cv2.imread('std_map.png')
        
        self.sub_location = self.create_subscription(Locations, "/ekf_location_filtered", self.location_callback, qos_profile)
        # 创建一个锁
        self.lock = threading.Lock()
        
        cv2.namedWindow('map', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('map', 800, 471)
        
    def display_locations(self):
        show_map = self.map.copy()

        cur_locations = []
        with self.lock:
            cur_locations = self.locations

        for i in range(len(cur_locations.locs)):
            location = cur_locations.locs[i]
            x = round(location.x, 2)
            y = round(location.y, 2)
            z = round(location.z, 2)
            
            xx = int( x * 100)
            yy = int( 1500 - y * 100)
            # # 限定xx,yy范围
            # if xx < 0 or xx > 2800 or yy < 0 or yy > 1500:
            #     xx = max(0, min(xx, 2800))
            #     yy = max(0, min(yy, 1500))
                
            if location.label == 'Red':
                cv2.putText(show_map, str(location.id), (xx - 15, yy + 10), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 4)
                cv2.putText(show_map, str((x)) + ',' + str((y)) + ',' + str(z), (xx, yy - 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 4)
                cv2.circle(show_map, (xx, yy), 60, (0, 0, 255), 4)
            else:
                cv2.putText(show_map, str(location.id), (xx - 15, yy + 10), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 4)
                cv2.putText(show_map, str((x)) + ',' + str((y)) + ',' + str(z), (xx, yy - 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 4)
                cv2.circle(show_map, (xx, yy), 60, (255, 0, 0), 4)
        cv2.imshow('map', show_map)
        cv2.waitKey(16)
        
    def location_callback(self, msg):
        # 更新locations
        with self.lock:
            self.locations = msg

def spin_thread(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    disPanel = DisplayPanel()
    
    # 创建一个线程来读取节点内容
    thread = threading.Thread(target=spin_thread, args=(disPanel,))
    thread.start()
    
    try:
        while rclpy.ok():
            disPanel.display_locations()
            time.sleep(1/15)  # 控制显示频率
    except KeyboardInterrupt:
        pass
    
    disPanel.destroy_node()
    rclpy.shutdown()
    thread.join()

if __name__ == '__main__':
    main()