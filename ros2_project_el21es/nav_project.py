import threading
import sys, time
import cv2
import numpy as np
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.exceptions import ROSInterruptException
import signal

# class GoToPose(Node):
#     def __init__(self):
#         super().__init__('navigation_goal_action_client')
#         self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')



class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Initialise a publisher to publish messages to the robot basegit clone git@github.com:COMP3631-2025/ros2_project <space> ros2_project_xxx
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription  # prevent unused variable warning
        
        self.sensitivity = 50
        self.green_found = False
        self.blue_found = False
        self.red_found = False
        self.skip_imaging = False

        self.turn_clockwise = False
        self.turn_anticlockwise = False
        self.move_forward = False
        self.move_backward = False
        self.stop_move = False
        self.pose_time = False
        self.isNavigating = False
        self.nav_reached = False

    def callback(self, data):

        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL)
        cv2.imshow('camera_Feed', image)
        cv2.resizeWindow('camera_Feed',320,240)
        cv2.waitKey(3)
        
        if self.skip_imaging == False:
            hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
            hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
            hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
            hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])
            hsv_red_lower1 = np.array([180 - self.sensitivity, 100, 100])
            hsv_red_upper1 = np.array([180, 255, 255])
            hsv_red_lower2 = np.array([0, 100, 100])
            hsv_red_upper2 = np.array([0 + self.sensitivity, 255, 255])
            
            Hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Filter out for specific colours
            green_mask = cv2.inRange(Hsv_image, hsv_green_lower, hsv_green_upper)
            blue_mask = cv2.inRange(Hsv_image, hsv_blue_lower, hsv_blue_upper)
            red_mask0 = cv2.inRange(Hsv_image, hsv_red_lower1, hsv_red_upper1)
            red_mask1 = cv2.inRange(Hsv_image, hsv_red_lower2, hsv_red_upper2)
            red_mask = red_mask0 +red_mask1
            
            green_contours, _ = cv2.findContours(green_mask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
            blue_contours, _ = cv2.findContours(blue_mask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
            red_contours, _ = cv2.findContours(red_mask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
            
            if len(green_contours)>0:
                c_g = max(green_contours, key=cv2.contourArea)
                if cv2.contourArea(c_g) > 500: 
                    self.green_found = True
                    print("Green Found")
            if len(blue_contours)>0:
                c_b = max(blue_contours, key=cv2.contourArea)
                if cv2.contourArea(c_b) > 500: 
                    self.blue_found = True
                    print("Blue Found")
                    M = cv2.moments(c_b)
                    cbx, cby = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                    print("Cbx: ", cbx, ", Cby: ", cby)
                    # Stop all previous movement commands
                    self.turn_clockwise = False
                    self.turn_anticlockwise = False
                    self.move_forward = False
                    self.move_backward = False
                    self.stop_move = False
                    self.isNavigating = False
                    self.pose_time = False
                    print("area: ", cv2.contourArea(c_b) )
                    if cbx <= 500 - self.sensitivity:
                        print("clockwise")
                        self.turn_clockwise = True
                    elif cbx >= 500 + self.sensitivity:
                        self.turn_anticlockwise = True
                        print("anticlockwise")
                    else:
                        if cv2.contourArea(c_b) > 300000:
                            # Too close to object, need to move backwards
                            # Set a flag to tell the robot to move backwards when in the main loop
                            self.move_backward = True
                            print("too close")
                            
                        elif cv2.contourArea(c_b) < 290000:
                            # Too far away from object, need to move forwards
                            # Set a flag to tell the robot to move forwards when in the main loop
                            self.move_forward = True
                            print("too far")  
                        else:
                            self.stop_move = True
                            print("close enough")
                            
            if len(red_contours)>0:
                c_r = max(red_contours, key=cv2.contourArea)
                if cv2.contourArea(c_r) > 500: 
                    self.red_found = True
                    print("Red Found")
        

    def walk_forward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.1  # Forward with 0.1 m/s

        for _ in range(5): 
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def walk_backward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = -0.1  # Backward with 0.1 m/s

        for _ in range(5):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()
            
    def rotate_clockwise(self):
        desired_velocity = Twist()
        desired_velocity.angular.z = (3.141/12) # 15 degree rotation
        
        for _ in range(5):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()
    
    def rotate_anticlockwise(self):
        desired_velocity = Twist()
        desired_velocity.angular.z = -(3.141/12)  
        
        for _ in range(5):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def stop(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.0  # Send zero velocity to stop the robot
        desired_velocity.angular.z = 0.0

        self.publisher.publish(desired_velocity)
    
    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Orientation
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2)

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)
        

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        self.nav_reached = True
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
    


# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main():
    def signal_handler(sig, frame):
        robot.stop()
        rclpy.shutdown()

    # Instantiate your class
    # And rclpy.init the entire node
    rclpy.init(args=None)
    robot = Robot()
    # List of sample points for navigation 
    coord_list = [(0.0,0.0,0.0), (2.5,7.5,0.0), (-5.0,0.0,0.0), (-6.0,-7.0,0.0)]
    coord_count = 0
    robot.pose_time = True
    turn_count = 0
    spin_time = False
    
    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            if robot.pose_time == True:
                robot.skip_imaging = True
                robot.stop_move = False
                robot.send_goal(coord_list[coord_count][0], coord_list[coord_count][1], coord_list[coord_count][2])
                robot.isNavigating = True
                robot.pose_time = False
                
            # Publish moves
            if robot.isNavigating == True:
                if robot.nav_reached == True:
                    robot.isNavigating = False
                    robot.nav_reached = False
                    spin_time = True
                    robot.skip_imaging = False
                    print("spin time")
                    robot.turn_clockwise = True
                    coord_count += 1
            elif robot.stop_move == True:
                robot.stop()
            elif robot.turn_clockwise == True:
                robot.rotate_clockwise()
                if spin_time == True:     
                    turn_count = turn_count + 1
                    if turn_count >= 48:
                        robot.turn_clockwise = False
                        print("spin done")
                        turn_count = 0
            elif robot.turn_anticlockwise == True:
                robot.rotate_anticlockwise()
                robot.turn_anticlockwise = False
            elif robot.move_forward == True:
                robot.walk_forward()
                robot.move_forward = False
            elif robot.move_backward == True:
                robot.walk_backward()
                robot.move_backward = False
            else:
                # If movement is not being done (or stop is not being deliberatly held) go to next preset position
                robot.pose_time = True
                print("next pose")
            if coord_count == len(coord_list):
                robot.pose_time = False
            pass

    except ROSInterruptException:
        pass

    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()