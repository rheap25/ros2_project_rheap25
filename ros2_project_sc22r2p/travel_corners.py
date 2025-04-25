import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from math import sin, cos, pi
import time
import signal
import threading

class NavigationGoalActionClient(Node):
    def __init__(self):
        super().__init__('navigation')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_done = False 

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription
        self.sensitivity = 10
    
    def callback(self, data):
    
        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        detection_image = image.copy()
            

        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
        hsv_red_lower = np.array([0, 100, 100])
        hsv_red_upper = np.array([10, 255, 255])

        hsv_green_lower = np.array([40, 100, 100])
        hsv_green_upper = np.array([80, 255, 255])
            
        hsv_blue_lower = np.array([100, 100, 100])
        hsv_blue_upper = np.array([140, 255, 255])
            
        red_mask = cv2.inRange(hsv_image, hsv_red_lower, hsv_red_upper)
            
        green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
        blue_mask = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)
            
        all_colors_mask = cv2.bitwise_or(cv2.bitwise_or(red_mask, green_mask), blue_mask)
        filtered_image = cv2.bitwise_and(image, image, mask=all_colors_mask)
            
            
        cv2.namedWindow('Filtered Image', cv2.WINDOW_NORMAL)
        cv2.imshow('Filtered Image', filtered_image)
        cv2.resizeWindow('Filtered Image', 320, 240)
            
        cv2.waitKey(3)
        

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2)
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self.goal_done = False

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.goal_done = True
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        self.goal_done = True 


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print(feedback)




def main(args=None):

    def signal_handler(sig, frame):
        cv2.destroyAllWindows()
        rclpy.shutdown()

    rclpy.init(args=args)
    navigation_client = NavigationGoalActionClient()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(navigation_client,), daemon=True)
    thread.start()

    
    corners = [
        (-10.0, 3.5, -0.001),
        (-9.0, -14.0, -0.005),
        (8.0, -13.0, -0.005),
        (7.0, -4.0, -0.001)
    ]

    
    for i, point in enumerate(corners):
            x, y, yaw = point
            
            navigation_client.get_logger().info(f"Navigating to corner {i+1}/{len(corners)}: ({x:.2f}, {y:.2f})")
            navigation_client.send_goal(x, y, yaw)

            timeout = 60.0  

            while not navigation_client.goal_done and rclpy.ok():
                
                rclpy.spin_once(navigation_client, timeout_sec=0.1)
                time.sleep(0.1)
            

            navigation_client.get_logger().info(f"Finished navigation to corner {i+1}")
            time.sleep(2.0)
    

    navigation_client.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
