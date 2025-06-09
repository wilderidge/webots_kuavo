# utilities.py (修改后的版本)

from controller import Robot
import math

class MyCustomRobot(Robot):
    def __init__(self, verbose=True):
        """ Initializes the robot and the required devices. """
        super().__init__()
        self.MAX_WHEEL_SPEED = 3.0  # PR2 constant from pr2_demo.c
        self.WHEELS_DISTANCE = 0.4492 # PR2 constant from pr2_demo.c
        self.SUB_WHEELS_DISTANCE = 0.098 # PR2 constant from pr2_demo.c
        self.WHEEL_RADIUS = 0.08    # PR2 constant from pr2_demo.c
        self.ANGLE_THRESHOLD = 0.05 # Using a small radian threshold for rotation
        self.DISTANCE_THRESHOLD = 0.025 # Using a small distance threshold for movement
        self.DISTANCE_PER_GRID_CELL = 1.0 # This value needs to be adjusted based on your Webots world scaling
        self.timestep = 16 # Use PR2's timestep
        self.verbose = verbose
        self.current_angle_rad = 0.0 # Keep track of robot's orientation in radians

        # PR2 specific wheel motors and sensors
        self.wheel_motors = {}
        self.wheel_sensors = {}
        self.rotation_motors = {}
        self.rotation_sensors = {}

        # 新增：PR2 机械臂、躯干和抓手电机字典
        self.arm_motors = {}
        self.arm_sensors = {}
        self.gripper_motors = {}
        self.gripper_sensors = {}
        self.torso_motor = None
        self.torso_sensor = None


    def initialize_devices(self):
        """ Initializes sensors and actuators for PR2. """
        self.iu = self.getDevice('imu_sensor') # Renamed based on pr2_demo.c
        if self.iu:
            self.iu.enable(self.timestep)
        self.gps = self.getDevice('gps') # Assuming a GPS is still available for position
        if self.gps:
            self.gps.enable(self.timestep)

        # Initialize PR2 wheel motors and sensors
        wheel_motor_names = [
            "fl_caster_l_wheel_joint", "fl_caster_r_wheel_joint",
            "fr_caster_l_wheel_joint", "fr_caster_r_wheel_joint",
            "bl_caster_l_wheel_joint", "bl_caster_r_wheel_joint",
            "br_caster_l_wheel_joint", "br_caster_r_wheel_joint"
        ]
        for name in wheel_motor_names:
            motor = self.getDevice(name)
            if motor:
                self.wheel_motors[name] = motor
                self.wheel_sensors[name] = motor.getPositionSensor()
                if self.wheel_sensors[name]:
                    self.wheel_sensors[name].enable(self.timestep)
                self.wheel_motors[name].setPosition(float('inf'))
                self.wheel_motors[name].setVelocity(0.0)
            else:
                print(f"WARNING: Wheel motor '{name}' not found.")

        # Initialize PR2 rotation motors and sensors
        rotation_motor_names = [
            "fl_caster_rotation_joint", "fr_caster_rotation_joint",
            "bl_caster_rotation_joint", "br_caster_rotation_joint"
        ]
        for name in rotation_motor_names:
            motor = self.getDevice(name)
            if motor:
                self.rotation_motors[name] = motor
                self.rotation_sensors[name] = motor.getPositionSensor()
                if self.rotation_sensors[name]:
                    self.rotation_sensors[name].enable(self.timestep)
            else:
                print(f"WARNING: Rotation motor '{name}' not found.")

        # 新增：初始化机械臂和躯干电机
        # 左臂关节
        left_arm_joint_names = [
            "l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint",
            "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"
        ]
        # 右臂关节
        right_arm_joint_names = [
            "r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint",
            "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"
        ]
        # 抓手关节
        gripper_joint_names = ["l_gripper_joint", "r_gripper_joint"]

        # 躯干关节
        torso_joint_name = "torso_lift_joint"

        all_arm_joint_names = left_arm_joint_names + right_arm_joint_names
        for name in all_arm_joint_names:
            motor = self.getDevice(name)
            if motor:
                self.arm_motors[name] = motor
                self.arm_sensors[name] = motor.getPositionSensor()
                if self.arm_sensors[name]:
                    self.arm_sensors[name].enable(self.timestep)
            else:
                print(f"WARNING: Arm motor '{name}' not found.")

        for name in gripper_joint_names:
            motor = self.getDevice(name)
            if motor:
                self.gripper_motors[name] = motor
                self.gripper_sensors[name] = motor.getPositionSensor()
                if self.gripper_sensors[name]:
                    self.gripper_sensors[name].enable(self.timestep)
            else:
                print(f"WARNING: Gripper motor '{name}' not found.")

        self.torso_motor = self.getDevice(torso_joint_name)
        if self.torso_motor:
            self.torso_sensor = self.torso_motor.getPositionSensor()
            if self.torso_sensor:
                self.torso_sensor.enable(self.timestep)
        else:
            print(f"WARNING: Torso motor '{torso_joint_name}' not found.")

        # 确保没有机械臂自动伸展的代码在这里，如果有，请注释掉或删除。
        # 例如，之前你可能删除了 set_initial_position() 类似的调用。

        # Some devices, such as the InertialUnit, need some time to "warm up"
        self.wait(100) # Give some time for sensors to stabilize


    def wait(self, ms):
        """ Waits for a specified number of milliseconds. """
        self.step(ms)

    def set_wheels_speeds(self, fll, flr, frl, frr, bll, blr, brl, brr):
        """ Set the speeds of the robot wheels. """
        # ... (此部分保持不变) ...
        if self.wheel_motors.get("fl_caster_l_wheel_joint"):
            self.wheel_motors["fl_caster_l_wheel_joint"].setVelocity(fll)
        if self.wheel_motors.get("fl_caster_r_wheel_joint"):
            self.wheel_motors["fl_caster_r_wheel_joint"].setVelocity(flr)
        if self.wheel_motors.get("fr_caster_l_wheel_joint"):
            self.wheel_motors["fr_caster_l_wheel_joint"].setVelocity(frl)
        if self.wheel_motors.get("fr_caster_r_wheel_joint"):
            self.wheel_motors["fr_caster_r_wheel_joint"].setVelocity(frr)
        if self.wheel_motors.get("bl_caster_l_wheel_joint"):
            self.wheel_motors["bl_caster_l_wheel_joint"].setVelocity(bll)
        if self.wheel_motors.get("bl_caster_r_wheel_joint"):
            self.wheel_motors["bl_caster_r_wheel_joint"].setVelocity(blr)
        if self.wheel_motors.get("br_caster_l_wheel_joint"):
            self.wheel_motors["br_caster_l_wheel_joint"].setVelocity(brl)
        if self.wheel_motors.get("br_caster_r_wheel_joint"):
            self.wheel_motors["br_caster_r_wheel_joint"].setVelocity(brr)

    def set_wheels_speed(self, speed):
        """ Set all wheels to the same speed. """
        self.set_wheels_speeds(speed, speed, speed, speed, speed, speed, speed, speed)

    def stop_wheels(self):
        """ Stop all wheels. """
        self.set_wheels_speed(0.0)

    def set_rotation_wheels_angles(self, fl, fr, bl, br, wait_on_feedback):
        """ Set the rotation wheels angles. """
        # ... (此部分保持不变) ...
        if wait_on_feedback:
            self.stop_wheels()

        if self.rotation_motors.get("fl_caster_rotation_joint"):
            self.rotation_motors["fl_caster_rotation_joint"].setPosition(fl)
        if self.rotation_motors.get("fr_caster_rotation_joint"):
            self.rotation_motors["fr_caster_rotation_joint"].setPosition(fr)
        if self.rotation_motors.get("bl_caster_rotation_joint"):
            self.rotation_motors["bl_caster_rotation_joint"].setPosition(bl)
        if self.rotation_motors.get("br_caster_rotation_joint"):
            self.rotation_motors["br_caster_rotation_joint"].setPosition(br)

        if wait_on_feedback:
            target = [fl, fr, bl, br]
            rotation_sensor_keys = [
                "fl_caster_rotation_joint", "fr_caster_rotation_joint",
                "bl_caster_rotation_joint", "br_caster_rotation_joint"
            ]
            while self.step(self.timestep) != -1:
                all_reached = True
                for i, key in enumerate(rotation_sensor_keys):
                    if self.rotation_sensors.get(key) and \
                       not self.almost_equal(self.rotation_sensors[key].getValue(), target[i]):
                        all_reached = False
                        break
                if all_reached:
                    break

    def almost_equal(self, a, b, tolerance=0.05): # Based on pr2_demo.c TOLERANCE
        """ Check if two double values are almost equal. """
        return (a < b + tolerance) and (a > b - tolerance)

    def rotate_angle(self, angle_radians):
        """ Rotates the robot around itself of a given angle [rad]. """
        # ... (此部分保持不变，包括 current_angle_rad 更新) ...
        if self.verbose:
            print(f"Rotating by {math.degrees(angle_radians):.2f} degrees")

        self.stop_wheels()
        self.set_rotation_wheels_angles(3.0 * math.pi / 4.0, math.pi / 4.0, -3.0 * math.pi / 4.0, -math.pi / 4.0, True)

        max_wheel_speed = self.MAX_WHEEL_SPEED if angle_radians > 0 else -self.MAX_WHEEL_SPEED
        self.set_wheels_speed(max_wheel_speed)

        initial_wheel0_position = self.wheel_sensors["fl_caster_l_wheel_joint"].getValue() if self.wheel_sensors.get("fl_caster_l_wheel_joint") else 0.0
        expected_travel_distance = abs(angle_radians * 0.5 * (self.WHEELS_DISTANCE + self.SUB_WHEELS_DISTANCE))

        while self.step(self.timestep) != -1:
            if not self.wheel_sensors.get("fl_caster_l_wheel_joint"):
                break
            wheel0_position = self.wheel_sensors["fl_caster_l_wheel_joint"].getValue()
            wheel0_travel_distance = abs(self.WHEEL_RADIUS * (wheel0_position - initial_wheel0_position))

            if wheel0_travel_distance >= expected_travel_distance - self.DISTANCE_THRESHOLD:
                break

            if expected_travel_distance - wheel0_travel_distance < 0.1 * expected_travel_distance and expected_travel_distance > 0.01:
                self.set_wheels_speed(0.2 * max_wheel_speed)

        self.set_rotation_wheels_angles(0.0, 0.0, 0.0, 0.0, True)
        self.stop_wheels()
        self.current_angle_rad = (self.current_angle_rad + angle_radians) % (2 * math.pi)
        if self.current_angle_rad < 0:
            self.current_angle_rad += 2 * math.pi
        if self.verbose:
            print(f"Robot current absolute angle: {math.degrees(self.current_angle_rad):.2f} degrees")


    def go_forward(self, distance):
        """ Moves the robot forward for a given distance [m]. """
        # ... (此部分保持不变) ...
        if self.verbose:
            print(f"Moving forward by {distance:.2f} meters")

        max_wheel_speed = self.MAX_WHEEL_SPEED if distance > 0 else -self.MAX_WHEEL_SPEED
        self.set_wheels_speed(max_wheel_speed)

        initial_wheel0_position = self.wheel_sensors["fl_caster_l_wheel_joint"].getValue() if self.wheel_sensors.get("fl_caster_l_wheel_joint") else 0.0

        while self.step(self.timestep) != -1:
            if not self.wheel_sensors.get("fl_caster_l_wheel_joint"):
                break
            wheel0_position = self.wheel_sensors["fl_caster_l_wheel_joint"].getValue()
            wheel0_travel_distance = abs(self.WHEEL_RADIUS * (wheel0_position - initial_wheel0_position))

            if wheel0_travel_distance >= abs(distance) - self.DISTANCE_THRESHOLD:
                break

            if abs(distance) - wheel0_travel_distance < 0.1 * abs(distance) and abs(distance) > 0.01:
                self.set_wheels_speed(0.1 * max_wheel_speed)

        self.stop_wheels()


    def turn_east(self):
        target_angle = 0.0
        self._calculate_and_rotate_to_target(target_angle)

    def turn_north(self):
        target_angle = math.pi / 2.0
        self._calculate_and_rotate_to_target(target_angle)

    def turn_west(self):
        target_angle = math.pi
        self._calculate_and_rotate_to_target(target_angle)

    def turn_south(self):
        target_angle = 3.0 * math.pi / 2.0
        self._calculate_and_rotate_to_target(target_angle)

    def _calculate_and_rotate_to_target(self, target_angle):
        """ Helper to calculate relative rotation needed to reach target_angle. """
        angle_diff = target_angle - self.current_angle_rad
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        self.rotate_angle(angle_diff)

    # 新增：设置单个机械臂关节位置的方法 (保持不变)
    def set_arm_joint_position(self, joint_name, position, wait_on_feedback=True):
        motor = self.arm_motors.get(joint_name)
        sensor = self.arm_sensors.get(joint_name)
        if motor:
            # 确保设置的速度足够快，能够到达目标，但又不会过冲
            # PR2机械臂通常是位置控制模式，所以速度是最大速度
            # 如果不设置速度，它会使用默认的最大速度
            motor.setVelocity(motor.getMaxVelocity()) # 显式设置最大速度，以便快速到达目标
            motor.setPosition(position)
            if self.verbose:
                print(f"Setting {joint_name} to {position:.2f}")
            if wait_on_feedback and sensor:
                start_time = self.getTime()
                timeout = 2.0 # 给关节设置一个超时时间，防止无限等待
                while self.step(self.timestep) != -1:
                    # 检查是否超时
                    if self.getTime() - start_time > timeout:
                        print(f"WARNING: Joint '{joint_name}' timed out reaching position {position:.2f}. Current: {sensor.getValue():.2f}")
                        break
                    if self.almost_equal(sensor.getValue(), position, 0.01): # 更严格的关节位置阈值
                        break
        else:
            print(f"WARNING: Joint '{joint_name}' motor not found for setting position.")

    # 新增：设置躯干高度的方法 (保持不变，但请注意其对碰撞的影响)
    def set_torso_height(self, height, wait_on_feedback=True):
        if self.torso_motor:
            self.torso_motor.setVelocity(self.torso_motor.getMaxVelocity()) # 显式设置最大速度
            self.torso_motor.setPosition(height)
            if self.verbose:
                print(f"Setting torso height to {height:.2f}")
            if wait_on_feedback and self.torso_sensor:
                start_time = self.getTime()
                timeout = 2.0
                while self.step(self.timestep) != -1:
                    if self.getTime() - start_time > timeout:
                        print(f"WARNING: Torso timed out reaching height {height:.2f}. Current: {self.torso_sensor.getValue():.2f}")
                        break
                    if self.almost_equal(self.torso_sensor.getValue(), height, 0.01):
                        break
        else:
            print(f"WARNING: Torso motor not found for setting height.")

    # 新增：设置抓手的方法 (保持不变)
    def set_gripper_position(self, gripper_name, position, wait_on_feedback=True):
        motor = self.gripper_motors.get(gripper_name)
        sensor = self.gripper_sensors.get(gripper_name)
        if motor:
            motor.setVelocity(motor.getMaxVelocity()) # 显式设置最大速度
            motor.setPosition(position)
            if self.verbose:
                print(f"Setting {gripper_name} to {position:.2f}")
            if wait_on_feedback and sensor:
                start_time = self.getTime()
                timeout = 1.0 # 抓手可能更快
                while self.step(self.timestep) != -1:
                    if self.getTime() - start_time > timeout:
                        print(f"WARNING: Gripper '{gripper_name}' timed out reaching position {position:.2f}. Current: {sensor.getValue():.2f}")
                        break
                    if self.almost_equal(sensor.getValue(), position, 0.005):
                        break
        else:
            print(f"WARNING: Gripper motor '{gripper_name}' not found for setting position.")

    def retract_arms(self):
        """ 将PR2的两个机械臂收缩到一个安全的姿态。
            这些关节位置值需要你根据Webots中的PR2模型和环境手动调试。
            目标是找到一个既收缩又不会与任何障碍物（包括机器人自身）发生碰撞的姿态。
        """
        if self.verbose:
            print("Retracting PR2 arms to a safe posture...")

        # 1. 尝试将躯干降到最低，但如果这导致手臂碰撞，可能需要略微抬高
        #    PR2的躯干最低通常是 0.0，但为了避免某些组件碰撞地面，可以设为 0.01 或略高
        self.set_torso_height(0.05, True) # 尝试一个略微抬高但仍然很低的值，避免地面碰撞

        # 2. 设置左臂收缩姿态
        #    这些值需要你根据Webots调试得到
        #    一般策略：肩部向内收，抬高肘部，腕部弯曲，使手臂靠近身体
        #    以下是示例值，请根据你的仿真调整：
        self.set_arm_joint_position("l_shoulder_pan_joint", 0.0, False)    # 肩部平移，稍微向内收
        self.set_arm_joint_position("l_shoulder_lift_joint", 1.35, False)   # 肩部抬高，避免下垂碰撞
        self.set_arm_joint_position("l_upper_arm_roll_joint", 0.0, False) # 上臂滚动，保持中立或略微调整
        self.set_arm_joint_position("l_elbow_flex_joint", -2.2, False)    # 肘部弯曲，将前臂收回
        self.set_arm_joint_position("l_forearm_roll_joint", 0.0, False)   # 前臂滚动
        self.set_arm_joint_position("l_wrist_flex_joint", 0.0, False)     # 腕部弯曲
        self.set_arm_joint_position("l_wrist_roll_joint", 0.0, False)     # 腕部滚动

        # 3. 设置右臂收缩姿态 (镜像左臂，注意pan和roll关节的符号)
        #    通常，对于对称的机器人，右臂的pan和upper_arm_roll可能是左臂的负值
        self.set_arm_joint_position("r_shoulder_pan_joint", 0.0, False)   # 肩部平移，稍微向内收 (与左臂方向相反)
        self.set_arm_joint_position("r_shoulder_lift_joint", 1.35, False)   # 肩部抬高
        self.set_arm_joint_position("r_upper_arm_roll_joint", 0.0, False) # 上臂滚动
        self.set_arm_joint_position("r_elbow_flex_joint", -2.2, False)    # 肘部弯曲
        self.set_arm_joint_position("r_forearm_roll_joint", 0.0, False)
        self.set_arm_joint_position("r_wrist_flex_joint", 0.0, False)
        self.set_arm_joint_position("r_wrist_roll_joint", 0.0, False)

        # 4. 关闭抓手 (通常让它们完全闭合，避免手指伸出)
        self.set_gripper_position("l_gripper_joint", 0.0, False) # 0.0通常是闭合位置
        self.set_gripper_position("r_gripper_joint", 0.0, True)  # 最后一个等待反馈，确保所有机械臂都已到达目标位置

        if self.verbose:
            print("Arms retracted to safe posture.")