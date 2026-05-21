#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
import threading

class MockSimulator(Node):
    def __init__(self):
        super().__init__('mock_simulator')
        
        # 视觉与导航信号发布通道
        self.vision_pub = self.create_publisher(String, '/vision/signals', 10)
        
        # ================= 核心修改 =================
        # 双末端状态码发送通道 (完全分离，互不干扰)
        self.gripper_pub = self.create_publisher(Float64, '/gripper/status', 10)
        self.suction_pub = self.create_publisher(Float64, '/suction/status', 10)
        
        # 初始状态设为 9.9 代表空闲无动作
        self.mock_gripper_status = 9.9 
        self.mock_suction_status = 9.9 
        # ============================================

        # 以 10Hz 的频率持续广播底层状态
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.show_menu()

    def show_menu(self):
        print("\n" + "="*20 + " R2 双臂状态模拟器 " + "="*20)
        print(" [夹爪状态码模拟 (发布至 /gripper/status)]")
        print("  2 : 模拟抓紧完成 (发送 2.0)")   
        print("  1 : 模拟运动中... (发送 1.0)")
        print("  0 : 模拟张开到位 (发送 0.0)")
        print("  E : 模拟卡死异常 (发送 -1.0)")
        print("\n [吸盘状态码模拟 (发布至 /suction/status)]")
        print("  S1: 模拟吸附成功 (发送 1.0)")
        print("  S0: 模拟释放成功 (发送 0.0)")
        print("  SE: 模拟气压异常 (发送 -1.0)")
        print("\n [视觉/导航信号模拟]")
        print("  N : 导航到位 (NAV_SUCCESS)")
        print("  V : 视觉对准 (VISION_SUCCESS)")
        print("  A : 组装完成 (ASSEMBLY_DONE)")
        print("  L : 离开资源岛 (LEAVE_GYM)")
        print("=" * 56)

    def timer_callback(self):
        # 持续发布夹爪状态
        msg_g = Float64()
        msg_g.data = float(self.mock_gripper_status)
        self.gripper_pub.publish(msg_g)

        # 持续发布吸盘状态
        msg_s = Float64()
        msg_s.data = float(self.mock_suction_status)
        self.suction_pub.publish(msg_s)

    def publish_event(self, signal_text, description):
        msg = String()
        msg.data = signal_text
        self.vision_pub.publish(msg)
        print(f"\n[EVENT] >>> 广播信号 : {description} ({signal_text})")
        self.show_menu()

    def keyboard_listener(self):
        while rclpy.ok():
            try:
                cmd = input("  > ").strip().upper()
                
                # --- 夹爪控制 ---
                if cmd == '2':                     
                    self.mock_gripper_status = 2.0
                    print("\n[STATUS] >>> 发送夹爪状态: 2.0 (已抓紧)")
                    self.show_menu()
                elif cmd == '1':
                    self.mock_gripper_status = 1.0
                    print("\n[STATUS] >>> 发送夹爪状态: 1.0 (运动中...)")
                    self.show_menu()
                elif cmd == '0':
                    self.mock_gripper_status = 0.0
                    print("\n[STATUS] >>> 发送夹爪状态: 0.0 (张开到位)")
                    self.show_menu()
                elif cmd == 'E':
                    self.mock_gripper_status = -1.0
                    print("\n[STATUS] >>> 发送夹爪异常 (-1.0)")
                    self.show_menu()
                    
                # --- 吸盘控制 ---
                elif cmd == 'S1':
                    self.mock_suction_status = 1.0
                    print("\n[STATUS] >>> 发送吸盘状态: 1.0 (吸附成功)")
                    self.show_menu()
                elif cmd == 'S0':
                    self.mock_suction_status = 0.0
                    print("\n[STATUS] >>> 发送吸盘状态: 0.0 (释放)")
                    self.show_menu()
                elif cmd == 'SE':
                    self.mock_suction_status = -1.0
                    print("\n[STATUS] >>> 发送吸盘异常 (-1.0)")
                    self.show_menu()
                    
                # --- 信号控制 ---
                elif cmd == 'N':
                    self.publish_event("NAV_SUCCESS", "导航到位")
                elif cmd == 'V':
                    self.publish_event("VISION_SUCCESS", "视觉识别成功")
                elif cmd == 'A':
                    self.publish_event("ASSEMBLY_DONE", "R1组装完成")
                elif cmd == 'L':
                    self.publish_event("LEAVE_GYM", "离开资源岛")
            except EOFError:
                break

def main(args=None):
    rclpy.init(args=args)
    node = MockSimulator()
    thread = threading.Thread(target=node.keyboard_listener, daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()