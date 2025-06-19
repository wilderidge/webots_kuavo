#!/usr/bin/env python3
# integrated_server.py - 整合的Flask服务器，包含Webots进程控制和机器人状态管理

from flask import Flask, request, jsonify, render_template, send_file
from flask_cors import CORS
from flask_socketio import SocketIO, emit
from collections import deque
import subprocess
import signal
import os
import threading
import time
import logging
import json
import cv2
import io
import base64
import numpy as np
import sys
from datetime import datetime
from PIL import Image

# 添加控制器目录到系统路径
controller_path = os.path.join(os.path.dirname(__file__), 
                             'Webots_PR2_Path_Planning', 'controllers', 'BFS_exercise_1')
sys.path.append(controller_path)

app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*")

# 设置日志级别
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

# ================= Webots进程控制相关变量 =================
webots_process = None
webots_process_lock = threading.Lock()
process_logs = []
current_loaded_scene = None  # 当前已加载的场景信息

# ================= 机器人状态管理相关变量 =================
robot_goal = None
robot_status = {"robot_position": (7, 1), "status": "Idle"}
command_queue = deque()

# 存储所有请求和响应数据
request_response_log = []
supervisor_world_status_log = []

# 存储最新的相机图像
latest_camera_image = None
latest_depth_image = None

# 机器人配置数据
robot_configs = {}

# 创建 logs 目录
log_dir = "logs"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)

def load_robot_configs():
    """加载机器人配置文件"""
    global robot_configs
    config_file = os.path.join(os.path.dirname(__file__), "robot_configs.json")
    try:
        with open(config_file, 'r', encoding='utf-8') as f:
            robot_configs = json.load(f)
        print(f"✅ 机器人配置文件加载成功: {config_file}")
    except FileNotFoundError:
        print(f"❌ 配置文件未找到: {config_file}")
        robot_configs = {"robot_types": {}, "node_name_mapping": {}}
    except json.JSONDecodeError as e:
        print(f"❌ 配置文件JSON格式错误: {e}")
        robot_configs = {"robot_types": {}, "node_name_mapping": {}}
    except Exception as e:
        print(f"❌ 加载配置文件时出错: {e}")
        robot_configs = {"robot_types": {}, "node_name_mapping": {}}

def get_robot_type_by_name(node_name):
    """根据节点名称获取机器人类型"""
    if not robot_configs or "node_name_mapping" not in robot_configs:
        return "unknown"
    
    node_name_lower = node_name.lower()
    
    for robot_type, keywords in robot_configs["node_name_mapping"].items():
        for keyword in keywords:
            if keyword.lower() in node_name_lower:
                return robot_type
    
    return "unknown"

def get_robot_config(robot_type):
    """获取指定类型的机器人配置"""
    if not robot_configs or "robot_types" not in robot_configs:
        return robot_configs.get("robot_types", {}).get("unknown", {})
    
    return robot_configs["robot_types"].get(robot_type, robot_configs["robot_types"].get("unknown", {}))

# 在启动时加载配置
load_robot_configs()

# 场景配置字典
SCENE_CONFIGS = {
    "kuavo": {
        "id": "kuavo",
        "name": "Kuavo Robot Scene",
        "version": "1.0.0",
        "world_file": "kuavo.wbt",
        "working_dir": "/home/rebot801/Wangwei/develop/webots_flask_server_http/Webots_PR2_Path_Planning/worlds",
        "description": "Kuavo机器人场景"
    },
    "path_planning": {
        "id": "path_planning",
        "name": "Path Planning Scene",
        "version": "2.1.0",
        "world_file": "path_planning.wbt", 
        "working_dir": "/home/rebot801/Wangwei/develop/webots_flask_server_http/Webots_PR2_Path_Planning/worlds",
        "description": "路径规划场景"
    },
    "pr2": {
        "id": "pr2",
        "name": "PR2 Robot Demo",
        "version": "1.5.2",
        "world_file": "pr2_demo.wbt",
        "working_dir": "/home/rebot801/Wangwei/develop/webots_flask_server_http/Webots_PR2_Path_Planning/worlds", 
        "description": "PR2机器人演示场景"
    }
}

# ================= 工具函数 =================
def log_event(event_type, message, **kwargs):
    """记录事件到日志"""
    log_entry = {
        "timestamp": datetime.now().isoformat(),
        "event_type": event_type,
        "message": message,
        **kwargs
    }
    process_logs.append(log_entry)
    print(f"[{log_entry['timestamp']}] {event_type}: {message}")

def format_timestamp(unix_timestamp):
    """将 Unix 时间戳转换为可读格式"""
    return datetime.fromtimestamp(unix_timestamp).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

def kill_all_webots_processes():
    """杀死所有Webots相关进程"""
    global current_loaded_scene
    try:
        result = subprocess.run(['pgrep', '-f', 'webots'], 
                              capture_output=True, text=True)
        
        if result.stdout.strip():
            pids = result.stdout.strip().split('\n')
            for pid in pids:
                try:
                    pid = int(pid.strip())
                    os.kill(pid, signal.SIGTERM)
                    log_event("kill_webots", f"发送SIGTERM到Webots进程 {pid}")
                    time.sleep(0.1)
                except (ValueError, ProcessLookupError):
                    continue
            
            time.sleep(1)
            result = subprocess.run(['pgrep', '-f', 'webots'], 
                                  capture_output=True, text=True)
            if result.stdout.strip():
                pids = result.stdout.strip().split('\n')
                for pid in pids:
                    try:
                        pid = int(pid.strip())
                        os.kill(pid, signal.SIGKILL)
                        log_event("force_kill_webots", f"强制杀死Webots进程 {pid}")
                    except (ValueError, ProcessLookupError):
                        continue
        
        # 清理当前场景信息
        current_loaded_scene = None
        log_event("scene_cleanup", "已清理当前场景信息")
                        
        return True
    except Exception as e:
        log_event("kill_all_error", f"杀死所有Webots进程时出错: {e}", error=str(e))
        return False

# ================= Webots进程控制路由 =================



@app.route('/webots/logs', methods=['GET'])
def get_webots_logs():
    """获取Webots进程日志"""
    global webots_process
    
    logs = {"process_logs": process_logs}
    
    with webots_process_lock:
        if webots_process and webots_process.poll() is None:
            try:
                stdout_data = ""
                stderr_data = ""
                
                if webots_process.stdout:
                    stdout_data = webots_process.stdout.read()
                if webots_process.stderr:
                    stderr_data = webots_process.stderr.read()
                
                logs["current_output"] = {
                    "stdout": stdout_data,
                    "stderr": stderr_data
                }
            except Exception as e:
                logs["output_error"] = str(e)
    
    return jsonify(logs), 200

# ================= 机器人状态管理路由 =================
@app.route('/world_status', methods=['POST'])
def update_world_status():
    """接收世界状态数据"""
    data = request.get_json()
    if 'nodes' in data and 'timestamp' in data:
        supervisor_world_status_log.append({
            "timestamp_unix": data['timestamp'],
            "timestamp_readable": format_timestamp(data['timestamp']),
            "event_type": "world_status_update",
            "data": data['nodes'],
            "message": "World status updated by supervisor."
        })
        
        for node in data['nodes']:
            if node.get('is_moving'):
                print(f"Flask INFO: 节点 '{node['name']}' (正在移动): 位置={node['position']}, 偏航角={node['rotation_degrees']['yaw']:.2f}°")
        
        return jsonify({"message": "World status updated"}), 200
    else:
        print(f"--- WARNING: Received invalid world status update: {data} ---")
        request_response_log.append({
            "timestamp_unix": time.time(),
            "timestamp_readable": format_timestamp(time.time()),
            "event_type": "world_status_update_failure",
            "request_data": data,
            "message": "Invalid data for world status update.",
            "error": "Missing 'nodes' or 'timestamp'"
        })
        return jsonify({"error": "Invalid world status data"}), 400

@app.route('/set_goal', methods=['POST'])
def set_goal():
    """设置机器人目标"""
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

@app.route('/robot_command', methods=['GET'])
def get_robot_command():
    """获取机器人命令"""
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

@app.route('/robot_status', methods=['POST'])
def update_robot_status():
    """更新机器人状态"""
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

@app.route('/get_robot_status', methods=['GET'])
def get_robot_status():
    """获取机器人状态"""
    request_response_log.append({
        "timestamp_unix": time.time(),
        "timestamp_readable": format_timestamp(time.time()),
        "event_type": "get_robot_status",
        "response_data": robot_status,
        "message": "Web client fetched robot status."
    })
    return jsonify(robot_status), 200

# ================= 相机图像管理路由 =================
@app.route('/camera_status', methods=['POST'])
def update_camera_status():
    """更新相机状态"""
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

@app.route('/get_camera_image', methods=['GET'])
def get_camera_image():
    """获取RGB相机图像"""
    global latest_camera_image
    try:
        if latest_camera_image is None:
            return jsonify({"error": "No camera image available"}), 404
            
        image_data = base64.b64decode(latest_camera_image)
        img_io = io.BytesIO(image_data)
        
        return send_file(img_io, mimetype='image/png')
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/get_depth_image', methods=['GET'])
def get_depth_image():
    """获取深度相机图像"""
    global latest_depth_image
    try:
        if latest_depth_image is None:
            return jsonify({"error": "No depth image available"}), 404
            
        image_data = base64.b64decode(latest_depth_image)
        img_io = io.BytesIO(image_data)
        
        return send_file(img_io, mimetype='image/png')
    except Exception as e:
        return jsonify({"error": str(e)}), 500



# ================= WebSocket 事件处理 =================
@socketio.on('connect')
def test_connect():
    """客户端连接事件"""
    print("WebSocket客户端已连接")
    emit('status', robot_status)

@socketio.on('set_goal')
def handle_set_goal(data):
    """处理WebSocket设置目标事件"""
    global robot_goal
    robot_goal = tuple(data.get('goal'))
    print(f"WebSocket: 收到目标 {robot_goal}")
    emit('status', {"message": f"目标设置为 {robot_goal}"}, broadcast=True)

# ================= 日志保存函数 =================
def save_main_log_on_exit():
    """保存主日志文件"""
    log_filename = os.path.join(log_dir, "flask_server_activity_log.json")
    try:
        with open(log_filename, 'w', encoding='utf-8') as f:
            json.dump(request_response_log, f, indent=4, ensure_ascii=False)
        print(f"\n--- INFO: 主 Flask 服务器活动日志已保存至 {log_filename} ---")
    except Exception as e:
        print(f"\n--- ERROR: 保存主日志文件失败: {e} ---")

def save_supervisor_log_on_exit():
    """保存Supervisor世界状态日志文件"""
    supervisor_log_filename = os.path.join(log_dir, "supervisor_world_status_log.json")
    try:
        with open(supervisor_log_filename, 'w', encoding='utf-8') as f:
            json.dump(supervisor_world_status_log, f, indent=4, ensure_ascii=False)
        print(f"--- INFO: Supervisor 世界状态日志已保存至 {supervisor_log_filename} ---")
    except Exception as e:
        print(f"--- ERROR: 保存 Supervisor 日志文件失败: {e} ---")

def save_process_logs_on_exit():
    """保存Webots进程日志"""
    process_log_filename = os.path.join(log_dir, "webots_process_log.json")
    try:
        with open(process_log_filename, 'w', encoding='utf-8') as f:
            json.dump(process_logs, f, indent=4, ensure_ascii=False)
        print(f"--- INFO: Webots 进程日志已保存至 {process_log_filename} ---")
    except Exception as e:
        print(f"--- ERROR: 保存进程日志文件失败: {e} ---")




@app.route('/api/v1/scenes', methods=['GET'])
def get_available_scenes():
    """获取可用的场景列表"""
    # 获取name参数用于过滤
    name_filter = request.args.get('name')
    
    scenes_data = []
    
    for i, (scene_id, config) in enumerate(SCENE_CONFIGS.items(), 1):
        # 如果提供了name参数，只返回匹配的场景
        if name_filter and config['name'] != name_filter:
            continue
            
        world_file_path = os.path.join(config['working_dir'], config['world_file'])
        size = "0B"
        if os.path.exists(world_file_path):
            try:
                file_size = os.path.getsize(world_file_path)
                if file_size < 1024:
                    size = f"{file_size}B"
                elif file_size < 1024 * 1024:
                    size = f"{file_size // 1024}KB"
                else:
                    size = f"{file_size // (1024 * 1024)}MB"
            except OSError:
                size = "Unknown"
        
        scene_data = {
            "id": i,
            "name": config['name'],
            "version": config['version'],
            "size": size,
            "description": config['description'],
            "scene_id": config['id']
        }
        scenes_data.append(scene_data)
    
    # 如果提供了name参数但没有找到匹配的场景
    if name_filter and not scenes_data:
        return jsonify({
            "code": 404,
            "message": f"未找到名称为 '{name_filter}' 的场景",
            "data": []
        }), 404
    
    return jsonify({
        "code": 200,
        "message": "success",
        "data": scenes_data
    }), 200

@app.route('/api/v1/scene/load', methods=['GET'])
def start_webots():
    """启动Webots进程"""
    global webots_process, current_loaded_scene
    
    with webots_process_lock:
        try:
            data = {}
            if request.is_json:
                data = request.get_json() or {}
            elif request.form:
                data = request.form.to_dict()
            elif request.args:
                data = request.args.to_dict()
            
            scene_id = data.get('id')
            
            if not scene_id:
                return jsonify({
                    "code": 400,
                    "message": "缺少必需参数 'id'"
                }), 400
            
            if scene_id not in SCENE_CONFIGS:
                available_ids = list(SCENE_CONFIGS.keys())
                return jsonify({
                    "code": 404,
                    "message": f"场景ID '{scene_id}' 不存在，可用场景: {', '.join(available_ids)}"
                }), 404
            
            scene_config = SCENE_CONFIGS[scene_id]
            
            # 如果当前已有相同场景在运行，直接返回成功
            if (current_loaded_scene and 
                current_loaded_scene.get('id') == scene_id and 
                webots_process and webots_process.poll() is None):
                return jsonify({
                    "code": 200,
                    "message": f"场景 '{scene_config['name']}' 已在运行中",
                    "data": {
                        "scene_id": scene_id,
                        "scene_name": scene_config['name'],
                        "pid": webots_process.pid,
                        "status": "running"
                    }
                }), 200
            
            # 清理之前的进程
            log_event("pre_start_cleanup", "启动前清理所有Webots进程...")
            kill_all_webots_processes()
            webots_process = None
            current_loaded_scene = None
            
            world_file = scene_config['world_file']
            working_dir = scene_config['working_dir']
            description = scene_config['description']
            scene_display_name = scene_config['name']
            scene_version = scene_config['version']
            
            log_event("scene_selection", f"选择场景: {scene_id} - {scene_display_name} v{scene_version}")
            
            cmd = [
                'xvfb-run',
                'webots',
                '--stdout',
                '--stderr',
                '--batch',
                '--mode=realtime',
                world_file,
                '--stream'
            ]
            
            webots_process = subprocess.Popen(
                cmd,
                cwd=working_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
                text=True
            )
            
            # 更新当前已加载的场景信息
            current_loaded_scene = {
                "id": scene_id,
                "name": scene_display_name,
                "version": scene_version,
                "world_file": world_file,
                "description": description,
                "pid": webots_process.pid,
                "loaded_at": datetime.now().isoformat()
            }
            
            log_event(
                "webots_start_success",
                f"Webots进程启动成功，PID: {webots_process.pid}",
                pid=webots_process.pid,
                command=" ".join(cmd),
                working_dir=working_dir,
                world_file=world_file,
                scene_id=scene_id,
                scene_name=scene_display_name,
                scene_version=scene_version,
                scene_description=description
            )
            
            return jsonify({
                "code": 200,
                "message": "success",
                "data": {
                    "scene_id": scene_id,
                    "scene_name": scene_display_name,
                    "pid": webots_process.pid,
                    "status": "started"
                }
            }), 200
            
        except Exception as e:
            error_msg = f"启动Webots进程失败: {str(e)}"
            log_event("webots_start_error", error_msg, error=str(e))
            
            return jsonify({
                "code": 500,
                "message": error_msg
            }), 500


@app.route('/api/v1/scene/current', methods=['GET'])
def get_current_scene():
    """获取当前已加载的场景信息"""
    global current_loaded_scene, webots_process
    
    if not current_loaded_scene:
        return jsonify({
            "code": 404,
            "message": "当前没有已加载的场景",
            "data": None
        }), 404
    
    # 检查进程是否还在运行
    process_status = "unknown"
    if webots_process:
        if webots_process.poll() is None:
            process_status = "running"
        else:
            process_status = "stopped"
            # 如果进程已停止，清理场景信息
            current_loaded_scene = None
            return jsonify({
                "code": 404,
                "message": "场景进程已停止",
                "data": None
            }), 404
    
    return jsonify({
        "code": 200,
        "message": "success",
        "data": {
            **current_loaded_scene,
            "process_status": process_status
        }
    }), 200


@app.route('/api/v1/scene/stop', methods=['POST'])
def stop_webots():
    """停止当前Webots进程"""
    global webots_process, current_loaded_scene
    
    with webots_process_lock:
        try:
            if not webots_process or webots_process.poll() is not None:
                current_loaded_scene = None
                return jsonify({
                    "code": 404,
                    "message": "没有正在运行的Webots进程"
                }), 404
            
            scene_info = current_loaded_scene.copy() if current_loaded_scene else {}
            
            log_event("webots_stop_request", "收到停止Webots进程请求")
            
            # 优雅地停止进程
            success = kill_all_webots_processes()
            webots_process = None
            current_loaded_scene = None
            
            if success:
                log_event("webots_stop_success", "Webots进程已成功停止")
                return jsonify({
                    "code": 200,
                    "message": "Webots进程已成功停止",
                    "data": {
                        "stopped_scene": scene_info
                    }
                }), 200
            else:
                log_event("webots_stop_error", "停止Webots进程时出现问题")
                return jsonify({
                    "code": 500,
                    "message": "停止Webots进程时出现问题"
                }), 500
                
        except Exception as e:
            error_msg = f"停止Webots进程失败: {str(e)}"
            log_event("webots_stop_error", error_msg, error=str(e))
            
            return jsonify({
                "code": 500,
                "message": error_msg
            }), 500


@app.route('/api/v1/scene/objects', methods=['GET'])
def get_world_status():
    """获取当前世界状态数据 - 所有对象列表"""
    global supervisor_world_status_log
    
    if not supervisor_world_status_log:
        return jsonify({
            "code": 404,
            "message": "暂无世界状态数据",
            "data": {
                "total_count": 0,
                "list": []
            }
        }), 404
    
    try:
        # 获取最新的世界状态数据
        latest_log_entry = supervisor_world_status_log[-1]
        nodes_data = latest_log_entry.get('data', [])
        
        # 转换为标准格式
        formatted_list = []
        
        for i, node in enumerate(nodes_data, 1):
            node_name = node.get('name', 'Unknown')
            position = node.get('position', [0, 0, 0])
            
            # 根据节点名称从配置文件获取机器人类型和属性
            robot_type = get_robot_type_by_name(node_name)
            robot_config = get_robot_config(robot_type)
            
            # 特殊处理障碍物名称
            display_name = robot_config.get('name', node_name)
            if robot_type == "obstacle" and "OBSTACLE" in node_name:
                box_num = node_name.split('_')[-1] if '_' in node_name else "1"
                display_name = f"障碍物 Box{box_num}"
            
            formatted_node = {
                "id": i,
                "name": display_name,
                "ability_name": robot_config.get('ability_name', ["未知"]),
                "ability_code": robot_config.get('ability_code', ["unknown"]),
                "describe": robot_config.get('describe', "未分类物体"),
                "position": {
                    "x": f"{position[0]:.2f}",
                    "y": f"{position[1]:.2f}",
                    "z": f"{position[2]:.2f}"
                }
            }
            
            formatted_list.append(formatted_node)
        
        return jsonify({
            "code": 200,
            "message": "success",
            "data": {
                "total_count": len(formatted_list),
                "list": formatted_list
            }
        }), 200
        
    except Exception as e:
        return jsonify({
            "code": 500,
            "message": f"获取世界状态数据失败: {str(e)}"
        }), 500


@app.route('/api/v1/scene/object', methods=['GET'])
def get_single_object():
    """获取单个物体的详细信息"""
    global supervisor_world_status_log, current_loaded_scene
    
    # 获取参数
    scene_id = request.args.get('id')
    object_id = request.args.get('object_id')
    
    # 参数验证
    if not scene_id:
        return jsonify({
            "code": 400,
            "message": "缺少必需参数 'id'"
        }), 400
    
    if not object_id:
        return jsonify({
            "code": 400,
            "message": "缺少必需参数 'object_id'"
        }), 400
    
    # 验证场景ID是否为当前已加载的场景
    if not current_loaded_scene or current_loaded_scene.get('id') != scene_id:
        current_scene = current_loaded_scene.get('id') if current_loaded_scene else "无"
        return jsonify({
            "code": 409,
            "message": f"场景ID不匹配。当前已加载场景: '{current_scene}'，请求查询场景: '{scene_id}'"
        }), 409
    
    # 检查是否有世界状态数据
    if not supervisor_world_status_log:
        return jsonify({
            "code": 404,
            "message": "暂无世界状态数据"
        }), 404
    
    try:
        # 转换object_id为整数
        try:
            target_object_id = int(object_id)
        except ValueError:
            return jsonify({
                "code": 400,
                "message": f"无效的object_id: '{object_id}'，必须是整数"
            }), 400
        
        # 获取最新的世界状态数据
        latest_log_entry = supervisor_world_status_log[-1]
        nodes_data = latest_log_entry.get('data', [])
        
        # 查找目标物体
        target_object = None
        for i, node in enumerate(nodes_data, 1):
            if i == target_object_id:
                target_object = node
                break
        
        if not target_object:
            return jsonify({
                "code": 404,
                "message": f"未找到ID为 '{object_id}' 的物体"
            }), 404
        
        # 构建物体详细信息
        node_name = target_object.get('name', 'Unknown')
        position = target_object.get('position', [0, 0, 0])
        
        # 根据节点名称从配置文件获取机器人类型和属性
        robot_type = get_robot_type_by_name(node_name)
        robot_config = get_robot_config(robot_type)
        
        # 特殊处理障碍物名称
        display_name = robot_config.get('name', node_name)
        if robot_type == "obstacle" and "OBSTACLE" in node_name:
            box_num = node_name.split('_')[-1] if '_' in node_name else "1"
            display_name = f"障碍物 Box{box_num}"
        
        object_info = {
            "id": target_object_id,
            "name": display_name,
            "ability_name": robot_config.get('ability_name', ["未知"]),
            "ability_code": robot_config.get('ability_code', ["unknown"]),
            "describe": robot_config.get('describe', "未分类物体"),
            "position": {
                "x": f"{position[0]:.2f}",
                "y": f"{position[1]:.2f}",
                "z": f"{position[2]:.2f}"
            }
        }
        
        return jsonify({
            "code": 200,
            "message": "success",
            "data": object_info
        }), 200
        
    except Exception as e:
        return jsonify({
            "code": 500,
            "message": f"获取物体信息失败: {str(e)}"
        }), 500




# ================= 启动服务器 =================
if __name__ == '__main__':
    try:
        print("=" * 80)
        print("🚀 整合的Webots控制服务器启动中...")
        print("📍 服务器地址: http://localhost:30114")
        print("📖 API文档: http://localhost:30114")
        print("🎮 Webots进程控制、机器人状态管理、相机图像服务已整合")
        print("🛑 按 CTRL+C 退出")
        print("=" * 80)
        
        log_event("server_start", "整合Flask服务器启动")
        
        # 使用 socketio.run 而不是 app.run 来支持WebSocket
        socketio.run(app, host='0.0.0.0', port=30114, debug=True, use_reloader=False)
        
    except KeyboardInterrupt:
        print("\n🛑 接收到停止信号，正在清理...")
        
    finally:
        # 清理Webots进程和场景信息
        kill_all_webots_processes()
        current_loaded_scene = None
        
        # 保存所有日志
        save_main_log_on_exit()
        save_supervisor_log_on_exit()
        save_process_logs_on_exit()
        
        log_event("server_stop", "整合Flask服务器停止")
        print("✅ 服务器已安全停止")
