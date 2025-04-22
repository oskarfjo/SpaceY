import rclpy
from rclpy.node import Node
import subprocess
import serial
from sensor_msgs.msg import Imu, MagneticField, FluidPressure
from std_msgs.msg import Float64
import threading
import time
from ros_gz_interfaces.msg import EntityWrench
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseArray
import numpy as np
from scipy.spatial.transform import Rotation



class GazeboBridge(Node):
    def __init__(self):
        super().__init__('gazebo_bridge')
        self.get_logger().info("Starting Gazebo-ROS Bridge")
        


        # SIM INSTILLINGER
        self.usb_com = True
        self.ignition = True
        
        
        # GAZEBO BRIDGE SETUP
        time.sleep(3)
        self.bridge_process = subprocess.Popen([
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            'world/empty/model/RocketF2/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            'world/empty/model/RocketF2/link/base_link/sensor/magnetometer_sensor/magnetometer@sensor_msgs/msg/MagneticField@gz.msgs.Magnetometer',
            'world/empty/model/RocketF2/link/base_link/sensor/air_pressure/air_pressure@sensor_msgs/msg/FluidPressure@gz.msgs.FluidPressure',
            '/Gimbal_X@std_msgs/msg/Float64@gz.msgs.Double',
            '/Gimbal_Y@std_msgs/msg/Float64@gz.msgs.Double',
            '/Nose_Cone_Left@std_msgs/msg/Float64@gz.msgs.Double',
            '/Nose_Cone_Right@std_msgs/msg/Float64@gz.msgs.Double'
        ])
        self.get_logger().info("Started sensor bridge to gazebo")

        self.wrench_bridge_process = subprocess.Popen([
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            'world/empty/wrench@ros_gz_interfaces/msg/EntityWrench@gz.msgs.EntityWrench',
            '--ros-args', '--remap', '/world/empty/wrench:=world_wrench',
        ])
        self.get_logger().info("Started wrench bridge to gazebo")

        self.dynamic_pose_bridge_process = subprocess.Popen([
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/world/empty/dynamic_pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V'
        ])
        self.get_logger().info("Started dynamic pose bridge to gazebo")


        # SUBSCRIBERS
        self.create_subscription(Imu, 'world/empty/model/Rocket/link/base_link/sensor/imu_sensor/imu', self.imu_callback, 10)
        self.create_subscription(MagneticField, 'world/empty/model/Rocket/link/base_link/sensor/magnetometer_sensor/magnetometer', self.magnetometer_callback, 10)
        self.create_subscription(FluidPressure, 'world/empty/model/Rocket/link/base_link/sensor/air_pressure/air_pressure', self.air_pressure_callback, 10)
        self.create_subscription(PoseArray, '/world/empty/dynamic_pose/info', self.dynamic_pose_callback, 10)
        

        # PUBLISHER
        # GIMBAL (X and Y) Limits: -0.17 to 0.17 (-10 to 10 degree)
        # Nose cone left Limits: 0 to -1.57 (0 to -90 degree)
        # Nose cone right Limits: 0 to 1.57 (0 to 90 degree)

        self.x_gimbal_pub = self.create_publisher(Float64, '/Gimbal_X', 10)
        self.y_gimbal_pub = self.create_publisher(Float64, '/Gimbal_Y', 10)
        self.N_L_pub = self.create_publisher(Float64, '/Nose_Cone_Left', 10)
        self.N_R_pub = self.create_publisher(Float64, '/Nose_Cone_Right', 10)
        self.thrust_pub = self.create_publisher(EntityWrench, '/world_wrench', 10)

        if self.usb_com:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            time.sleep(2)
            self.serial_port.reset_input_buffer()
            self.serial_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.serial_thread.start()
        else:
            self.manual_servo_timer = self.create_timer(0.5, self.manual_servo)

        
        # SENSOR PARAMETER
        self.gx, self.gy, self.gz = 0.0, 0.0, 0.0 # Gyroscope
        self.ax, self.ay, self.az = 0.0, 0.0, 0.0 # Accelerometer
        self.mx, self.my, self.mz = 0.0, 0.0, 0.0 # Magnetometer
        self.pressure = 0.0 # Barometer

        self.imu_updated = False
        self.magnetometer_updated = False
        self.pressure_updated = False

        # ROCKET ENGINE PARAMETER
        self.engine_id = 17
        self.launch_done = False
        self.engine_orientation = Rotation.identity()
        self.rocket_orientation = Rotation.identity()
        self.engine_world_orientation = Rotation.identity()
        self.engine_orientation_valid = False 
        self.engine_position = [0.0, 0.0, 0.0]

        # TIMERS
        self.timer = self.create_timer(0.001, self.thrust_calculation)
        self.start_time = None
        self.start_timer_started = False

        # OTHERS
        self.teensy_ignition = False


    # FUNKSJONE FOR Å LESE ENGINE POSISJON
    def dynamic_pose_callback(self, msg):

        """#INDEX TESTER
        for idx, pose in enumerate(msg.poses):
            # Extract quaternion
            q = pose.orientation
            quat = [q.x, q.y, q.z, q.w]

            # Convert quaternion to Euler angles (in degrees)
            try:
                r = Rotation.from_quat(quat)
                euler_deg = r.as_euler('xyz', degrees=True)
                roll, pitch, yaw = euler_deg
            except ValueError:
                roll = pitch = yaw = float('nan')

            # Log the pose and the corresponding Euler angles
            self.get_logger().info(
                f"[Pose {idx}] Position: ({pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f}) | "
                f"Orientation (quat): ({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f}) | "
                f"Euler angles [deg]: Roll={roll:.2f}, Pitch={pitch:.2f}, Yaw={yaw:.2f}"
            )"""
            
        try:
            engine_pose = msg.poses[4]   # Motor_Mount
            rocket_pose = msg.poses[0]   # RocketF2

            # Konverter til scipy-rotation
            q_engine_rel = Rotation.from_quat([
                engine_pose.orientation.x,
                engine_pose.orientation.y,
                engine_pose.orientation.z,
                engine_pose.orientation.w
            ])
            q_rocket_world = Rotation.from_quat([
                rocket_pose.orientation.x,
                rocket_pose.orientation.y,
                rocket_pose.orientation.z,
                rocket_pose.orientation.w
            ])

            # Kombiner orientasjonar: motorens retning i verdensrommet
            q_world_engine = q_rocket_world * q_engine_rel

            # Lagre verdiane
            self.engine_world_orientation = q_world_engine
            self.engine_orientation_valid = True
            self.engine_world_position = [
                engine_pose.position.x,
                engine_pose.position.y,
                engine_pose.position.z
            ]
            rocket_height = rocket_pose.position.z
            self.get_logger().info(f"Rocket height (Z): {rocket_height:.2f} m")


            """
            self.get_logger().info(
                f"[Engine Pose] Position: ({engine_pose.position.x:.4f}, {engine_pose.position.y:.4f}, {engine_pose.position.z:.4f}) | "
                f"Orientation (quat): ({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})")
                """
            
            
        except IndexError:
            self.get_logger().warn("Engine pose not found in PoseArray")


    # IMU CALLBACK
    def imu_callback(self, msg):
        self.gx = round(msg.angular_velocity.x * 57.2958, 4)
        self.gy = round(msg.angular_velocity.y * 57.2958, 4)
        self.gz = round(msg.angular_velocity.z * 57.2958, 3)
        self.ax = round(msg.linear_acceleration.x, 4)
        self.ay = round(msg.linear_acceleration.y,4)
        self.az = round(msg.linear_acceleration.z,4)
        self.imu_updated = True
        self.check_and_send_data()

    
    # MAGNETOMETER CALLBACK
    def magnetometer_callback(self, msg):
        self.mx = round(msg.magnetic_field.x, 4)
        self.my = round(msg.magnetic_field.y, 4)
        self.mz = round(msg.magnetic_field.z, 4)
        self.magnetometer_updated = True
        self.check_and_send_data()
    

    # AIR PRESSURE CALLBACK
    def air_pressure_callback(self, msg):
        self.pressure = round(msg.fluid_pressure, 4)
        self.pressure_updated = True
        self.check_and_send_data()
    

    # FUNKSJON FOR Å OVERFØRE DATA TIL TEENSY
    def check_and_send_data(self):
        if self.imu_updated and self.magnetometer_updated and self.pressure_updated:
            command = f"DATA,{self.gx},{self.gy},{self.gz},{self.ax},{self.ay},{self.az},{self.mx},{self.my},{self.mz},{self.pressure}\n"
            if self.usb_com:
                self.serial_port.write(command.encode())
                #self.get_logger().info(f"Sent to Teensy: {command.strip()}")

            self.imu_updated = False
            self.magnetometer_updated = False
            self.pressure_updated = False
    

    # FUNKSJON FOR Å REKNE UT D3 KRAFT OVER TID
    def thrust_function(self, t):
        if t < 0.4:
            return (47.06 * (t)**4) + (0.79 * (t)**3) + (15.37 * (t)**2) + (0.71 * t)
        elif 0.4 <= t < 0.6:
            return -6 + (25 * t)
        elif 0.6 <= t < 0.7:
            return 33 - 40 * t
        elif 0.7 <= t <1.1:
            return (16.67 * t**2) - (35 * t) + 21.33
        elif 1.1 <= t < 5.5:
            return 3
        elif 5.5 <= t < 6:
            return 36 - (6 * t)
        elif 6 <= t:
            self.launch_done = True
            return 0
    
    # FUNKSJON FOR Å STARTE THRUST UTREKNING
    def activate_thrust(self):
        self.start_time = time.time()
        self.get_logger().info("Launching!")


    # FUNKSJON FOR Å OVERFØRE KRAFT TIL GAZEBO
    def thrust_calculation(self):

        if self.usb_com:
            if self.teensy_ignition and self.ignition and self.engine_orientation_valid and not self.start_timer_started:
                self.activate_thrust()
                self.start_timer_started = True
        else:
            if self.ignition and self.engine_orientation_valid and not self.start_timer_started:
                self.get_logger().info("10 Second countdown started!")
                self.create_timer(10.0, lambda: self.activate_thrust())
                self.start_timer_started = True


        if self.start_time is None or self.launch_done:
            return

        current_time = time.time() - self.start_time
        thrust_value = round((4* self.thrust_function(current_time)), 2)

        local_force = np.array([0.0, thrust_value, 0.0])
        world_force = self.engine_world_orientation.apply(local_force)

        wrench_msg = EntityWrench()
        wrench_msg.entity.id = self.engine_id 
        wrench_msg.wrench.force = Vector3(x=float(world_force[0]), y=float(world_force[1]), z=float(world_force[2]))
        wrench_msg.wrench.torque = Vector3(x=float(0.0), y=float(0.0), z=float(0.0))

        self.thrust_pub.publish(wrench_msg)
        #self.get_logger().info(f"Thrust vector world: {world_force}")
        #self.get_logger().info(f'Published thrust: {thrust_value:.2f} N at time: {current_time}')
        #self.get_logger().info(f"Thrust vector world: [{world_force[0]:.4f}, {world_force[1]:.4f}, {world_force[2]:.2f}]")
     
    
    # FUNKSJON FOR Å LESE DATA FRA TEENSY
    def read_serial(self):
        while rclpy.ok():
            try:
                line = self.serial_port.readline().decode().strip()
                if line:
                    #self.get_logger().info(f"Received from Teensy: {line}")
                    parts = line.split(',')

                    if len(parts) >= 3 and parts[0] in ["SERVO", "FULL"]:
                        x_angle = float(parts[1])
                        y_angle = float(parts[2])

                        if parts[3] == 1:
                            N_L_angle = float(-1.57)
                            N_R_angle = float(1.57)
                        else:
                            N_L_angle = float(0)
                            N_R_angle = float(0)

                        if parts[4] == 1:
                            self.teensy_ignition = True
                        else:
                            self.teensy_ignition = False

                        x_msg = Float64()
                        y_msg = Float64()
                        N_L_msg = Float64()
                        N_R_msg = Float64()

                        x_msg.data = x_angle
                        y_msg.data = y_angle
                        N_L_msg.data = N_L_angle
                        N_R_msg.data = N_R_angle

                        self.x_gimbal_pub.publish(x_msg)
                        self.y_gimbal_pub.publish(y_msg)
                        self.N_L_pub.publish(N_L_msg)
                        self.N_R_pub.publish(N_R_msg)

            except ValueError:
                self.get_logger().warn(f"Invalid number format in: {line}")



    # FUNKSJON FOR Å SETTE MANUEL SERVO POSISJON (UTEN TEENSY)
    def manual_servo(self):
        # GIMBAL (X and Y) Limits: -0.17 to 0.17 (-10 to 10 degree)
        # Nose cone left Limits: 0 to -1.57 (0 to -90 degree)
        # Nose cone right Limits: 0 to 1.57 (0 to 90 degree)

        x_angle = float(0)
        y_angle = float(0)

        N_L_angle = float(0)
        N_R_angle = float(0)

        x_msg = Float64()
        y_msg = Float64()

        N_L_msg = Float64()
        N_R_msg = Float64()

        x_msg.data = x_angle
        y_msg.data = y_angle

        N_L_msg.data = N_L_angle
        N_R_msg.data = N_R_angle

        self.x_gimbal_pub.publish(x_msg)
        self.y_gimbal_pub.publish(y_msg)
        self.N_L_pub.publish(N_L_msg)
        self.N_R_pub.publish(N_R_msg)

    
    # AVLSUTTING AV ROS
    def destroy(self):
        self.get_logger().info("Shutting down gazebo-ros bridge and serial bridge")
        self.bridge_process.terminate()
        self.wrench_bridge_process.terminate()
        self.dynamic_pose_bridge_process.terminate()
        if self.usb_com:
            self.serial_port.close()
        super().destroy_node()
        


def main(args=None):
    rclpy.init(args=args)
    node = GazeboBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()