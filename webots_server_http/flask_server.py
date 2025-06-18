# flask_server.py (修改后的完整文件)
from flask import Flask, request, jsonify, render_template, send_file
from flask_cors import CORS
from collections import deque
import threading
import cv2
import time
import logging
import json
import os
from datetime import datetime # 导入 datetime 模块
import io
from PIL import Image
import numpy as np
import sys
import os
import base64

# 添加控制器目录到系统路径
controller_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 
                             'Webots_PR2_Path_Planning', 'controllers', 'BFS_exercise_1')
sys.path.append(controller_path)

app = Flask(__name__)
CORS(app)

log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

robot_goal = None
robot_status = {"robot_position": (7, 1), "status": "Idle"}
command_queue = deque()

# 存储所有请求和响应数据 (不包括 supervisor 的世界状态更新)
request_response_log = []

# 专门存储来自 supervisor 的世界状态更新数据
supervisor_world_status_log = []

# 存储最新的相机图像
latest_camera_image = None

# 创建 logs 目录 (如果不存在)
log_dir = "logs"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)

# 辅助函数：将 Unix 时间戳转换为可读格式
def format_timestamp(unix_timestamp):
    return datetime.fromtimestamp(unix_timestamp).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3] # 截断毫秒到3位

# 新增路由：接收世界状态数据
@app.route('/world_status', methods=['POST'])
def update_world_status():
    data = request.get_json()
    if 'nodes' in data and 'timestamp' in data:
        # 将 supervisor 的世界状态数据添加到独立的列表中
        # 在这里，我们先将原始数据添加进去，然后在保存时进行格式转换
        # 或者，你也可以在接收时就转换 timestamp，看你的偏好
        supervisor_world_status_log.append({
            "timestamp_unix": data['timestamp'], # 原始 Unix 时间戳
            "timestamp_readable": format_timestamp(data['timestamp']), # 可读时间戳
            "event_type": "world_status_update",
            "data": data['nodes'], # nodes 字段中现在应该包含 'name' (DEF 名称)
            "message": "World status updated by supervisor."
        })
        # 可以在这里选择性地打印一些简要信息到终端，例如只打印移动的物体
        for node in data['nodes']:
            if node.get('is_moving'):
                # 打印到 Flask 服务器终端，方便实时监控移动的物体
                print(f"Flask INFO: 节点 '{node['name']}' (正在移动): 位置={node['position']}, 偏航角={node['rotation_degrees']['yaw']:.2f}°")
        return jsonify({"message": "World status updated"}), 200
    else:
        print(f"--- WARNING: Received invalid world status update: {data} ---")
        request_response_log.append({ # 错误信息可以记录到主日志
            "timestamp_unix": time.time(),
            "timestamp_readable": format_timestamp(time.time()),
            "event_type": "world_status_update_failure",
            "request_data": data,
            "message": "Invalid data for world status update.",
            "error": "Missing 'nodes' or 'timestamp'"
        })
        return jsonify({"error": "Invalid world status data"}), 400

# 以下是原有路由，保持不变或略作调整以更统一地记录日志
# Route to receive goal from web client
@app.route('/set_goal', methods=['POST'])
def set_goal():
    global robot_goal
    data = request.get_json()
    
    if 'goal' in data:
        robot_goal = tuple(data['goal'])
        print(f"--- INFO: Received new goal from web client: {robot_goal} ---")
        
        request_response_log.append({
            "timestamp_unix": time.time(),
            "timestamp_readable": format_timestamp(time.time()),
            "event_type": "set_goal_success",
            "request_data": data,
            "message": "Goal received and set."
        })
        return jsonify({"message": f"Goal set to {robot_goal}"}), 200
    else:
        print(f"--- WARNING: Received invalid goal request: {data} ---")
        request_response_log.append({
            "timestamp_unix": time.time(),
            "timestamp_readable": format_timestamp(time.time()),
            "event_type": "set_goal_failure",
            "request_data": data,
            "message": "No 'goal' provided in request.",
            "error": "Invalid request payload"
        })
        return jsonify({"error": "No 'goal' provided"}), 400

# Route for Webots controller to fetch commands (e.g., goal)
@app.route('/robot_command', methods=['GET'])
def get_robot_command():
    global robot_goal
    command_to_send = {}
    if robot_goal:
        command_to_send = {"goal": robot_goal}
    
    request_response_log.append({
        "timestamp_unix": time.time(),
        "timestamp_readable": format_timestamp(time.time()),
        "event_type": "get_robot_command",
        "response_data": command_to_send,
        "message": "Webots controller fetched command."
    })
    
    return jsonify(command_to_send), 200

# Route for Webots controller to send its status
@app.route('/robot_status', methods=['POST'])
def update_robot_status():
    global robot_status
    data = request.get_json()
    if 'robot_position' in data and 'status' in data:
        robot_status["robot_position"] = tuple(data['robot_position'])
        robot_status["status"] = data['status']
        print(f"--- INFO: Robot Status Update: 位置={robot_status['robot_position']}, 状态='{robot_status['status']}' ---")
        
        request_response_log.append({
            "timestamp_unix": time.time(),
            "timestamp_readable": format_timestamp(time.time()),
            "event_type": "robot_status_update_success",
            "request_data": data,
            "message": "Robot status updated."
        })

        return jsonify({"message": "Status updated"}), 200
    else:
        print(f"--- WARNING: Received invalid robot status update: {data} ---")
        request_response_log.append({
            "timestamp_unix": time.time(),
            "timestamp_readable": format_timestamp(time.time()),
            "event_type": "robot_status_update_failure",
            "request_data": data,
            "message": "Invalid data for robot status update.",
            "error": "Missing 'robot_position' or 'status'"
        })
        return jsonify({"error": "Invalid status data"}), 400

# Route for web client to fetch robot status
@app.route('/get_robot_status', methods=['GET'])
def get_robot_status():
    request_response_log.append({
        "timestamp_unix": time.time(),
        "timestamp_readable": format_timestamp(time.time()),
        "event_type": "get_robot_status",
        "response_data": robot_status,
        "message": "Web client fetched robot status."
    })
    return jsonify(robot_status), 200

@app.route('/')
def index():
    return render_template('index.html')

from flask_socketio import SocketIO, emit
socketio = SocketIO(app, cors_allowed_origins="*")

@socketio.on('connect')
def test_connect():
    print("Client connected")
    emit('status', robot_status)

@socketio.on('set_goal')
def handle_set_goal(data):
    global robot_goal
    robot_goal = tuple(data.get('goal'))
    print(f"WebSocket: Received goal {robot_goal}")
    emit('status', {"message": f"Goal set to {robot_goal}"}, broadcast=True)

# 新增：获取相机图像的路由
# 新增深度图存储和获取接口
@app.route('/camera_status', methods=['POST'])
def update_camera_status():
    global latest_camera_image, latest_depth_image
    try:
        data = request.get_json()
        if 'image' in data:
            latest_camera_image = data['image']
        if 'depth_image' in data:
            latest_depth_image = data['depth_image']
        return jsonify({"status": "success"}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/get_depth_image', methods=['GET'])
def get_depth_image():
    global latest_depth_image
    try:
        if latest_depth_image is None:
            return jsonify({"error": "No camera image available"}), 404
            
        # 解码base64图像
        image_data = base64.b64decode(latest_depth_image)
        
        # 创建一个字节流
        img_io = io.BytesIO(image_data)
        
        return send_file(img_io, mimetype='image/png')
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/get_camera_image', methods=['GET'])
def get_camera_image():
    global latest_camera_image
    try:
        if latest_camera_image is None:
            return jsonify({"error": "No camera image available"}), 404
            
        # 解码base64图像
        image_data = base64.b64decode(latest_camera_image)
        
        # 创建一个字节流
        img_io = io.BytesIO(image_data)
        
        return send_file(img_io, mimetype='image/png')
    except Exception as e:
        return jsonify({"error": str(e)}), 500

# --- 服务结束时保存主日志文件 ---
def save_main_log_on_exit():
    log_filename = os.path.join(log_dir, "flask_server_activity_log.json") # 主日志文件
    try:
        with open(log_filename, 'w', encoding='utf-8') as f:
            # 这里的 request_response_log 已经在路由中添加了可读时间戳，无需额外处理
            json.dump(request_response_log, f, indent=4, ensure_ascii=False)
        print(f"\n--- INFO: 主 Flask 服务器活动日志已保存至 {log_filename} ---")
    except Exception as e:
        print(f"\n--- ERROR: 保存主日志文件失败: {e} ---")

# --- 新增：服务结束时保存 Supervisor 世界状态日志文件 ---
def save_supervisor_log_on_exit():
    supervisor_log_filename = os.path.join(log_dir, "supervisor_world_status_log.json") # Supervisor 专用日志文件
    try:
        with open(supervisor_log_filename, 'w', encoding='utf-8') as f:
            # supervisor_world_status_log 已经在接收时处理了时间戳，无需额外处理
            json.dump(supervisor_world_status_log, f, indent=4, ensure_ascii=False)
        print(f"--- INFO: Supervisor 世界状态日志已保存至 {supervisor_log_filename} ---")
    except Exception as e:
        print(f"--- ERROR: 保存 Supervisor 日志文件失败: {e} ---")


if __name__ == '__main__':
    try:
        print("--- INFO: Flask 服务器正在启动... 按 CTRL+C 退出。 ---")
        app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)
    finally:
        # 在 finally 块中同时调用两个保存函数
        save_main_log_on_exit()
        save_supervisor_log_on_exit()