import datetime
import traceback
import sqlite3
import rclpy as rp
from rclpy.node import Node
from rclpy.qos import QoSProfile
from library.Constants import Service, Topic, Constants, DeviceCode, DeviceStatus, ResponseCode
from message.msg import DispenserStatus, ComponentStatus
from message.srv import RobotService, DispenseService
import time

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')
        self.robot_service_client = self.create_client(RobotService, Service.SERVICE_ROBOT)
        self.dispenser_service_client = self.create_client(DispenseService, Service.SERVICE_DISPENSER)

        # Wait for the service to be available
        while not self.robot_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('RobotService not available, waiting...')

        while not self.dispenser_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('DispenseService not available, waiting...')

        self.publisher = self.create_publisher(DispenserStatus, Topic.ROBOT_STATUS, QoSProfile(depth=10))
        # self.subscriber = self.create_subscription(DispenserStatus,  Topic.ROBOT_STATUS, QoSProfile(depth=10), self.get_status)
        
        # self.timer = self.create_timer(1.0, self.timer_callback)
        self.command_index = 0
        self.commands = self.load_commands_from_db('robot_commands.db')
        self.node_status = DeviceStatus.STANDBY
        self.send_robot_command(self.commands[self.command_index])

    def load_commands_from_db(self, db_path):
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        cursor.execute("SELECT * FROM robot_commands ORDER BY no")
        commands = cursor.fetchall()
        conn.close()
        return commands

    def send_robot_command(self, command):
        if command[1] in ['COFFEE_ON', 'WATER_TOGGLE']:
            request = DispenseService.Request()
            request.command = command[1]
            future = self.dispenser_service_client.call_async(request)
            future.add_done_callback(self.response_dis_callback)
        else:
            request = RobotService.Request()
            request.seq_no = str(datetime.datetime.now())
            request.cmd = command[1]
            request.par1 = command[2]
            request.par2 = command[3]
            request.par3 = command[4]
            request.par4 = command[5]
            request.par5 = command[6]
            future = self.robot_service_client.call_async(request)
            future.add_done_callback(self.response_callback)
            


    def response_dis_callback(self,future):
        response = future.result()
        time.sleep(0.1)
        seq_no = response.seq_no 
        response_cd = response.response_cd
        self.get_logger().info(f"seq_no : {seq_no} , response_cd = {response_cd}")
        if response_cd == ResponseCode.SUCCESS:
            if self.command_index < len(self.commands):
                self.command_index += 1
                command = self.commands[self.command_index]
                self.send_robot_command(command)

    def response_callback(self,future):
        time.sleep(0.1)
    
        response = future.result()
        seq_no = response.seq_no 
        component_cd = response.component_cd
        response_cd = response.response_cd
        status_cd = response.status_cd 
        result = response.result
        self.get_logger().info(f"seq_no : {seq_no} , component_cd = {component_cd} , response_cd = {response_cd}, status_cd = {status_cd}, result = {result} ")
        if response_cd == ResponseCode.SUCCESS:
            if self.command_index < len(self.commands):
                self.command_index += 1
                command = self.commands[self.command_index]
                self.send_robot_command(command)

    def get_status(self, msg):
        self.node_status = msg.node_status

def main(args=None):
    rp.init(args=args)
    node = RobotControllerNode()
    try:
        rp.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rp.shutdown()

if __name__ == '__main__':
    main()
