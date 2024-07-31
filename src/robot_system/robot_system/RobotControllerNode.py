import sys
import os
import glob
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel
from PyQt5.QtCore import QThread, pyqtSignal, QCoreApplication
import rclpy as rp
from rclpy.node import Node
from rclpy.qos import QoSProfile
from library.Constants import Service, Topic, Constants, DeviceCode, DeviceStatus, ResponseCode
from message.msg import DispenserStatus, ComponentStatus
from message.srv import RobotService, DispenseService
from library.Constants import Command, RobotParameter, Constants, RobotCommand
import sqlite3
import datetime
import time
from ament_index_python.packages import get_package_share_directory

os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = '/usr/local/opt/qt/plugins'

packageShareDirectory = get_package_share_directory('robot_system')
uiPath = os.path.join(packageShareDirectory, 'ui/')

Ui = glob.glob(os.path.join(uiPath, "baris.ui"))[0]
Robot_controller_app = uic.loadUiType(Ui)[0]

class RobotControllerNode(Node):
    
    def __init__(self):
        super().__init__('robot_controller_node')
        qos_profile = QoSProfile(depth=Constants.QOS_DEFAULT)
        self.robot_service_client = self.create_client(RobotService, Service.SERVICE_ROBOT)
        self.dispenser_service_client = self.create_client(DispenseService, Service.SERVICE_DISPENSER)
        self.robot_status_sub = self.create_subscription(DispenserStatus, Topic.ROBOT_STATUS, self.robot_status_callback, qos_profile=qos_profile)

        while not self.robot_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('RobotService not available, waiting...')

        while not self.dispenser_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('DispenseService not available, waiting...')

        self.publisher = self.create_publisher(DispenserStatus, Topic.ROBOT_STATUS, QoSProfile(depth=10))
        self.command_index = 0
        self.commands = self.load_commands_from_db('robot_commands.db')
        self.node_status = DeviceStatus.STANDBY
        self.done = False
        

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
        time.sleep(0.2)
        seq_no = response.seq_no 
        response_cd = response.response_cd
        self.get_logger().info("###############################################")
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
        result = result.split('//')[0]
        self.get_logger().info("###############################################")
        self.get_logger().info(f"seq_no : {seq_no} , component_cd = {component_cd} , response_cd = {response_cd}, status_cd = {status_cd}, result = {result} ")

        gesture = self.extract_gesture_from_result(result)
        self.get_logger().info(f"Extracted Gesture: {gesture}")
        # self.notify_ui(f"{gesture}")

        if not self.done:
            if response_cd == ResponseCode.SUCCESS:
                self.command_index += 1
                if self.command_index < len(self.commands):
                    command = self.commands[self.command_index]
                    self.send_robot_command(command)
        
        if result == "GESTURE":
            self.done = True
            self.get_logger().info("send reset")
            command = ['', '', '', '', '', '', '']
            command[1] = Command.RESET
            self.send_robot_command(command)
            self.command_index = 0
        else:
            self.done = False


    def robot_status_callback(self, topic_msg):
            seq_no = topic_msg.seq_no 
            component = topic_msg.component
            node_status = topic_msg.node_status
            # self.get_logger().info(f"component {component}")
            # self.get_logger().info(f"node_status {node_status}")
            self.notify_ui(f"{node_status}", component[0].status)

            

    def extract_gesture_from_result(self, result):
        try:
            parts = result.split('//')
            if len(parts) > 0:
                return parts[0]  # assuming GESTURE is always the first element
            return "No robot_statusFound"
        except Exception as e:
            self.get_logger().error(f"Failed to extract gesture: {str(e)}")
            return "Error"

    def notify_ui(self, node_status, robot_status):
        # This method will be overwritten in the RobotControllerThread to emit the signal
        pass

    def get_status(self, msg):
        self.node_status = msg.node_status


class RobotControllerThread(QThread):
    status_update = pyqtSignal(str, str)

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

    def log_info(self, node_status, robot_status):
        self.status_update.emit(node_status, robot_status)


class RobotControllerApp(QWidget, Robot_controller_app):

    def __init__(self):
        super().__init__()
        self.initUI()
        self.robot_thread = RobotControllerThread()
        self.robot_thread.start()
        self.robot_thread.status_update.connect(self.update_status)
        self.show()

    def initUI(self):
        self.setupUi(self)
        self.setWindowTitle('Robot Controller')

        self.order_btn.clicked.connect(self.start_robot)

    def start_robot(self):
        self.robot_thread.node.send_robot_command(self.robot_thread.node.commands[self.robot_thread.node.command_index])

    def update_status(self, node_status, robot_status):
        self.status_line.setText(f'{node_status}')
        self.robot_line.setText(f'{robot_status}')

    def closeEvent(self, event):
        self.robot_thread.quit()
        self.robot_thread.wait()
        QCoreApplication.instance().quit()
        event.accept()

def main():
    app = QApplication(sys.argv)
    ex = RobotControllerApp()
    exit_code = app.exec_()
    rp.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()