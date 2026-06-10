# #!/usr/bin/env python3
# import rospy
# from geometry_msgs.msg import Twist

# def move_turtle():
#     rospy.init_node('turtle_mover', anonymous=True)
#     pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
#     rate = rospy.Rate(10)  # 10 Hz

#     twist = Twist()
#     twist.linear.x = 2.0  # 向前速度
#     twist.angular.z = 1.0  # 旋转速度

#     while not rospy.is_shutdown():
#         pub.publish(twist)
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         move_turtle()
#     except rospy.ROSInterruptException:
#         pass
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleSquare(Node):
    def __init__(self):
        super().__init__('turtle_square')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_square)
        self.stage = 0
        self.start_time = time.time()
        self.speed = 1.5       # 线速度
        self.turn_speed = 1.57 # 角速度（约 90°/s）
        self.side_duration = 2 # 每条边前进 2 秒
        self.turn_duration = 1 # 转弯 90°耗时 1 秒
        self.state = 'forward' # 当前阶段 forward/turn

    def move_square(self):
        msg = Twist()
        t = time.time() - self.start_time

        if self.state == 'forward':
            msg.linear.x = self.speed
            msg.angular.z = 0.0
            if t >= self.side_duration:
                self.state = 'turn'
                self.start_time = time.time()
        elif self.state == 'turn':
            msg.linear.x = 0.0
            msg.angular.z = self.turn_speed
            if t >= self.turn_duration:
                self.state = 'forward'
                self.start_time = time.time()
                self.stage += 1
                if self.stage >= 4:
                    self.stage = 0  # 可以无限循环

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSquare()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()