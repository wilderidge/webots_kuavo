# BFS_exercise_1_webots_controller.py (修改后的版本)

# ... (文件顶部导入和配置保持不变) ...
from utilities import MyCustomRobot
from collections import deque
import json
import time
import requests
import math # 确保导入 math

# Configuration for Web communication
WEB_SERVER_URL = "http://127.0.0.1:5000/robot_command"

robot = MyCustomRobot(verbose=True) # 可以将 verbose 设置为 True，方便调试
robot.initialize_devices()
timestep = int(robot.getBasicTimeStep())

world_map = [
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, 0],
                [0, 1, 1, 0, 1, 1, 1, 0],
                [0, 1, 1, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, 0]
            ]

start = (7, 1)
goal = None

# 在机器人开始移动之前，调用收缩机械臂的方法
# 可以在这里立即调用，或者在收到第一个目标并计算出路径后调用
robot.retract_arms() # <<<<<<<<<<<<<<< 在这里调用，让机械臂在机器人开始规划和移动前收缩

def is_valid_position(position):
    # ... (此函数保持不变) ...
    map_size = 8 # 如果地图大小变了，这里也要改
    if 0 <= position[0] < map_size and 0 <= position[1] < map_size:
        if world_map[position[0]][position[1]] == 0:
            return True
    return False

def get_neighbors(map, position):
    # ... (此函数保持不变) ...
    north = (position[0]-1, position[1])
    east = (position[0], position[1]+1)
    south = (position[0]+1, position[1])
    west = (position[0], position[1]-1)
    output = []
    for direction in [north, east, south, west]:
        if is_valid_position(direction):
            output.append(direction)
    return output

def path_planning_BFS(map, start, goal):
    # ... (此函数保持不变) ...
    if not is_valid_position(start) or not is_valid_position(goal):
        print(f"Start {start} or Goal {goal} position is invalid!")
        return []

    frontier = deque()
    frontier.append(start)
    came_from = dict()
    came_from[start] = None
    while frontier:
        current = frontier.popleft()
        if current == goal:
            break
        for neighbor in get_neighbors(world_map, current):
            if neighbor not in came_from:
                frontier.append(neighbor)
                came_from[neighbor] = current

    if goal not in came_from:
        print(f"No path found from {start} to {goal}!")
        return []

    step = came_from[goal]
    path = [goal]
    while step is not None:
        path.append(step)
        step = came_from[step]
    path.reverse()
    return path

def move_from_to(current, next_pos):
    """ Decide in which cardinal direction to move, based on the current and
    next positions, and execute the movement using PR2-like actions. """
    # ... (此函数保持不变，使用 robot.turn_xxx() 和 robot.go_forward()) ...
    distance_per_grid_cell = robot.DISTANCE_PER_GRID_CELL

    if next_pos[0] < current[0]:
        print(f"Moving North from {current} to {next_pos}")
        robot.turn_north()
        robot.go_forward(distance_per_grid_cell)
    elif next_pos[0] > current[0]:
        print(f"Moving South from {current} to {next_pos}")
        robot.turn_south()
        robot.go_forward(distance_per_grid_cell)
    elif next_pos[1] < current[1]:
        print(f"Moving West from {current} to {next_pos}")
        robot.turn_west()
        robot.go_forward(distance_per_grid_cell)
    elif next_pos[1] > current[1]:
        print(f"Moving East from {current} to {next_pos}")
        robot.turn_east()
        robot.go_forward(distance_per_grid_cell)

def send_robot_status(current_pos, status_message):
    # ... (此函数保持不变) ...
    payload = {
        "robot_position": current_pos,
        "status": status_message
    }
    try:
        requests.post("http://127.0.0.1:5000/robot_status", json=payload)
    except requests.exceptions.ConnectionError:
        print("Could not connect to web server. Is it running?")
    except Exception as e:
        print(f"Error sending status: {e}")

# Main loop:
current_robot_position = start
path = []
goal_received = False

while robot.step(timestep) != -1:
    # ... (主循环逻辑保持不变) ...
    try:
        response = requests.get(WEB_SERVER_URL)
        if response.status_code == 200:
            command_data = response.json()
            new_goal = tuple(command_data.get("goal"))
            if new_goal and new_goal != goal:
                goal = new_goal
                print(f"Received new goal: {goal}")
                goal_received = True
                path = path_planning_BFS(world_map, current_robot_position, goal)
                if path:
                    print("Path recalculated:", path)
                    send_robot_status(current_robot_position, f"Path calculated to {goal}")
                else:
                    send_robot_status(current_robot_position, f"No path to {goal}")
                    goal = None
    except requests.exceptions.ConnectionError:
        pass
    except json.JSONDecodeError:
        print("Invalid JSON response from web server.")
    except Exception as e:
        print(f"Error fetching command: {e}")

    if goal_received and path:
        if current_robot_position == goal:
            print("Goal reached!")
            send_robot_status(current_robot_position, "Goal reached!")
            goal_received = False
            path = []
            # 可选：到达目标后可以再次调用 retract_arms，或者执行其他动作
            # robot.retract_arms()
        else:
            next_step_index = -1
            try:
                next_step_index = path.index(current_robot_position) + 1
            except ValueError:
                print(f"Robot moved off path or path no longer valid. Recalculating from {current_robot_position}...")
                path = path_planning_BFS(world_map, current_robot_position, goal)
                if not path:
                    print("Could not recalculate path.")
                    send_robot_status(current_robot_position, "Path blocked or no alternative.")
                    goal_received = False
                    continue
                next_step_index = path.index(current_robot_position) + 1

            if next_step_index < len(path):
                next_pos = path[next_step_index]
                print(f"Next step: moving from {current_robot_position} to {next_pos}")
                move_from_to(current_robot_position, next_pos)
                current_robot_position = next_pos
                send_robot_status(current_robot_position, "Moving along path.")
            else:
                print("Path completed, but goal not yet matched. Final check.")
                send_robot_status(current_robot_position, "Path traversal complete.")
                if current_robot_position == goal:
                     print("Goal reached after path traversal.")
                     send_robot_status(current_robot_position, "Goal reached!")
                     goal_received = False
                     path = []
    elif goal_received and not path:
        print(f"No path found to {goal}. Waiting for new goal.")
        send_robot_status(current_robot_position, f"No path to {goal}.")
        goal_received = False

    time.sleep(0.1)