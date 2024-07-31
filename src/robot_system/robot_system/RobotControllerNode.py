import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel
from PyQt5.QtCore import QThread, pyqtSignal
import rclpy as rp
from rclpy.node import Node
from rclpy.qos import QoSProfile
from library.Constants import Service, Topic, Constants, DeviceCode, DeviceStatus, ResponseCode
from message.msg import DispenserStatus, ComponentStatus
from message.srv import RobotService, DispenseService
import sqlite3
import datetime
import time


class RobotControllerNode(Node):
    
    def __init__(self):
        super().__init__('robot_controller_node')
        self.robot_service_client = self.create_client(RobotService, Service.SERVICE_ROBOT)
        self.dispenser_service_client = self.create_client(DispenseService, Service.SERVICE_DISPENSER)

        while not self.robot_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('RobotService not available, waiting...')

        while not self.dispenser_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('DispenseService not available, waiting...')

        self.publisher = self.create_publisher(DispenserStatus, Topic.ROBOT_STATUS, QoSProfile(depth=10))
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
            
    def response_dis_callback(self, future):
        response = future.result()
        time.sleep(0.1)
        seq_no = response.seq_no 
        response_cd = response.response_cd
        self.get_logger().info(f"seq_no : {seq_no} , response_cd = {response_cd}")
        if response_cd == ResponseCode.SUCCESS:
            self.command_index += 1
            if self.command_index < len(self.commands):
                command = self.commands[self.command_index]
                self.send_robot_command(command)

    def response_callback(self, future):
        time.sleep(0.1)
        response = future.result()
        seq_no = response.seq_no
        component_cd = response.component_cd
        response_cd = response.response_cd
        status_cd = response.status_cd
        result = response.result
        self.get_logger().info(f"seq_no : {seq_no} , component_cd = {component_cd} , response_cd = {response_cd}, status_cd = {status_cd}, result = {result} ")

        gesture = self.extract_gesture_from_result(result)
        self.get_logger().info(f"Extracted Gesture: {gesture}")
        self.notify_ui(f"{gesture}")

        if response_cd == ResponseCode.SUCCESS:
            self.command_index += 1
            if self.command_index < len(self.commands):
                command = self.commands[self.command_index]
                self.send_robot_command(command)

    def extract_gesture_from_result(self, result):
        try:
            parts = result.split('//')
            if len(parts) > 0:
                return parts[0]  # assuming GESTURE is always the first element
            return "No robot_statusFound"
        except Exception as e:
            self.get_logger().error(f"Failed to extract gesture: {str(e)}")
            return "Error"

    def notify_ui(self, message):
        # This method will be overwritten in the RobotControllerThread to emit the signal
        pass

    def get_status(self, msg):
        self.node_status = msg.node_status


class RobotControllerThread(QThread):
    status_update = pyqtSignal(str)

    def __init__(self):
        super().__init__()

    def run(self):
        rp.init(args=None)
        self.node = RobotControllerNode()
        self.node.notify_ui = self.log_info
        try:
            rp.spin(self.node)
        except KeyboardInterrupt:
            pass
        finally:
            self.node.destroy_node()
            rp.shutdown()

    def log_info(self, msg):
        self.status_update.emit(msg)


class RobotControllerApp(QWidget):

    def __init__(self):
        super().__init__()
        self.initUI()
        self.robot_thread = RobotControllerThread()
        self.robot_thread.status_update.connect(self.update_status)

    def initUI(self):
        self.layout = QVBoxLayout()

        self.status_label = QLabel('Robot Status: STANDBY', self)
        self.layout.addWidget(self.status_label)

        self.start_button = QPushButton('Start Robot', self)
        self.start_button.clicked.connect(self.start_robot)
        self.layout.addWidget(self.start_button)

        self.setLayout(self.layout)
        self.setWindowTitle('Robot Controller')
        self.show()

    def start_robot(self):
        self.robot_thread.start()

    def update_status(self, msg):
        self.status_label.setText(f'Robot Status: {msg}')


def main():
    app = QApplication(sys.argv)
    ex = RobotControllerApp()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()