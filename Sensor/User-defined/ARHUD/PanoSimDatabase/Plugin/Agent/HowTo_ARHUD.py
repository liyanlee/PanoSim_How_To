import math

import numpy as np
from DataInterfacePython import *
from PIL import Image,ImageDraw,ImageFont
from PIL.ImageOps import expand
from sympy import false
from sympy.physics.units import length
import time
from typing import Tuple

def ModelStart(userData):
    PPI = 72
    INCH_TO_CM = 2.54
    userData["width"] = math.ceil(57 * PPI / INCH_TO_CM)
    userData["height"] = math.ceil(19 * PPI / INCH_TO_CM)
    userData["arhud"] = BusAccessor(userData["busId"], "ARHUD.0", "time@i,%d@[,x@b,y@b,z@b,a@b" % (userData["width"] * userData["height"]))
    userData["time"] = 0
    # 构造主车状态总线读取器1
    userData['ego_state'] = BusAccessor(userData['busId'], 'ego', 'time@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d')
    # 交通参与物信息读取，type==1为行人,type==0为车辆
    userData['traffic'] = DoubleBusReader(userData['busId'], 'traffic',
                                          'time@i,100@[,id@i,type@b,shape@i,x@f,y@f,z@f,yaw@f,pitch@f,roll@f,speed@f')
    userData["objectPerception"] = BusAccessor(userData['busId'],'ObjectPerception.0','Timestamp@i,64@[,OBJ_ID@i,OBJ_Class@b,OBJ_X@d,OBJ_Y@d,OBJ_Z@d,OBJ_Velocity@d,OBJ_Length@d,OBJ_Width@d,OBJ_Height@d')
    userData["global.0"] = BusAccessor(userData["busId"], "global.0", GLOBAL_FORMAT)
    userData["global.1"] = BusAccessor(userData["busId"], "global.1", GLOBAL_FORMAT)
    userData["global.2"] = BusAccessor(userData["busId"], "global.2", GLOBAL_FORMAT)
    userData["global.3"] = BusAccessor(userData["busId"], "global.3", GLOBAL_FORMAT)
    userData["global.4"] = BusAccessor(userData["busId"], "global.4", GLOBAL_FORMAT)
    userData["global.5"] = BusAccessor(userData["busId"], "global.5", GLOBAL_FORMAT)
    userData["global.7"] = BusAccessor(userData["busId"], "global.7", GLOBAL_FORMAT)
    userData["global.8"] = BusAccessor(userData["busId"], "global.8", GLOBAL_FORMAT)
    userData["global.9"] = BusAccessor(userData["busId"], "global.9", GLOBAL_FORMAT)

def ModelOutput(userData):
    PPI = 72
    INCH_TO_CM = 2.54
    width = userData["width"]
    height = userData["height"]
    triangle_base = 50
    triangle_height = 50

    ego_time, ego_x, ego_y, ego_z, ego_yaw, ego_pitch, ego_roll, ego_speed = userData['ego_state'].readHeader()

    canvas_position = get_world_position([ego_x, ego_y, ego_z], ego_yaw, ego_pitch, ego_roll,[0.65,0.42,1.115])

    eyeposition = get_world_position([ego_x, ego_y, ego_z], ego_yaw, ego_pitch, ego_roll,[-1.45,0.42,1.135])

    ego_time, objectPerception_width = userData["objectPerception"].readHeader()

    traffic_list = []
    ego_Position = np.array([ego_x,ego_y,ego_z])
    sensorPosition = np.array([0,0,1]) + ego_Position
    for i in range(objectPerception_width):  
        OBJ_ID, OBJ_Class, OBJ_X, OBJ_Y, OBJ_Z, OBJ_Velocity, OBJ_Length, OBJ_Width, OBJ_Height = userData["objectPerception"].readBody(index=i)
        if OBJ_Class == 0 or OBJ_Class == 1:
            world_pos = transform_point(sensorPosition,ego_yaw,ego_pitch,ego_roll,[OBJ_X,OBJ_Y,OBJ_Z],false)
            traffic_list.append(world_pos + np.array([0,0,OBJ_Height]))

    # for frame in range(frames):
        # 清除图像并重新绘制三角形
    img = create_image(width, height)

    #绘制速度
    draw_speed(img, f"{math.ceil(ego_speed  * 3.6)}", (width//2,80),180, (0, 255, 255))
    draw_speed(img, f"km/h", (width//2,20),60, (0, 255, 255))

    for vehicle_position in traffic_list:
        # # 计算浮动偏移
        offset = math.sin(ego_time * 1000)

        # 调用函数计算交点
        intersection, is_inside, local_coords = compute_canvas_intersection(
            eyeposition, vehicle_position, canvas_position,
            ego_yaw, ego_pitch, ego_roll, [0.57,0.19]
        )

        # 设置三角形的顶点坐标
        x = math.ceil(width // 2 + local_coords[0] * 100 * PPI / INCH_TO_CM)
        y = math.ceil(height // 2 + local_coords[1] * 100 * PPI / INCH_TO_CM)

        if is_inside :
            #计算三角形的顶点坐标
            vertices = calculate_triangle_vertices(x, y, triangle_base, triangle_height, offset)

            #绘制三角形
            draw_triangle(img, vertices)

            # 在三角形上绘制文字
            text = f"{math.ceil(math.sqrt((ego_x - vehicle_position[0]) ** 2 + (ego_y - vehicle_position[1]) ** 2))} m"
            text_position = (x,y - triangle_height // 2 - 50)  # 放在三角形上方
            draw_text(img, text, text_position)

    img = img.transpose(Image.FLIP_LEFT_RIGHT)

    # 更新图像数据
    update_image_data(img, userData)

def ModelTerminate(userData):
    pass

def euler_to_rotation_matrix(yaw, pitch, roll, degrees=True):
    if degrees:
        yaw = np.deg2rad(yaw)
        pitch = np.deg2rad(pitch)
        roll = np.deg2rad(roll)

    # 绕Z轴（yaw）
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0, 0, 1]
    ])

    # 绕Y轴（pitch）
    Ry = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [ 0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    # 绕X轴（roll）
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])

    # 总旋转矩阵，注意顺序：Rz * Ry * Rx （Yaw → Pitch → Roll）
    R = Rz @ Ry @ Rx
    return R

def transform_point(sensor_pos, yaw, pitch, roll, local_pos, degrees=True):
    # 构造旋转矩阵
    R = euler_to_rotation_matrix(yaw, pitch, roll, degrees)

    # 构造 4x4 变换矩阵 T
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = sensor_pos

    # 构造局部坐标（齐次形式）
    local_h = np.array([*local_pos, 1.0])

    # 世界坐标
    world_h = T @ local_h
    return world_h[:3]

# 生成初始图像数据
def create_image(width, height, gray_value=40, radius=50):
    """创建一个灰色背景的 RGBA 图像"""
    """创建一个带圆角的灰色背景 RGBA 图像"""
    image_data = np.full((height, width, 4), (gray_value, gray_value, gray_value, 10), dtype=np.uint8)
    img = Image.fromarray(image_data, 'RGBA')

    # 创建一个带透明度的蒙版
    mask = Image.new('L', (width, height), 0)
    draw = ImageDraw.Draw(mask)
    draw.rounded_rectangle((0, 0, width, height), radius=radius, fill=100)

    # 应用蒙版，使背景变成圆角
    img.putalpha(mask)
    return img

#绘制文字
def draw_speed(img, text, position, font_size=32, color=(255, 0, 0)):
    try:
        font = ImageFont.truetype("calibri.ttf", font_size)
    except IOError:
        font = ImageFont.load_default()  # 备用字体

        # 创建一个透明背景的新图片，用来绘制文字
    text_layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(text_layer)

    # 计算文本大小
    text_width, text_height = draw.textsize(text, font=font)

    # 创建文字画布
    text_img = Image.new("RGBA", (text_width, text_height), (0, 0, 0, 0))
    text_draw = ImageDraw.Draw(text_img)

    # 绘制文字
    text_draw.text((0, 0), text, font=font, fill=color)

    # 旋转180度（翻转）
    rotated_text_img = text_img.rotate(180, expand=True)

    # 计算放置旋转文本的中心点
    text_x = position[0] - rotated_text_img.width // 2
    text_y = position[1]

    # 将旋转的文字粘贴到原图上
    img.paste(rotated_text_img, (text_x, text_y), rotated_text_img)

# 计算三角形的顶点
def calculate_triangle_vertices(x, y, triangle_base, triangle_height, offset):
    top = (x, y - triangle_height // 2 + int(offset))
    left = (x - triangle_base // 2, y + triangle_height // 2 + int(offset))
    right = (x + triangle_base // 2, y + triangle_height // 2 + int(offset))
    return [top, left, right]

# 绘制三角形
def draw_triangle(img, vertices):
    draw = ImageDraw.Draw(img)
    draw.polygon(vertices, fill=(255, 0, 0, 180))  # 红色填充

# 更新图像并返回字节数据
def update_image_data(img, userData):

    image_bytes = img.tobytes()
    bytes_time = userData["time"]
    bytes_size = userData["width"] * userData["height"] * 4
    b_time = struct.pack('i', bytes_time)
    b_size = struct.pack('i', bytes_size)
    userData["arhud"].getBus()[:] = b_time + b_size + image_bytes

# 在图像上绘制文字
def draw_text(img, text, position, font_size=60):
    try:
        font = ImageFont.truetype("arial.ttf", font_size)
    except IOError:
        font = ImageFont.load_default()  # 备用字体

        # 创建一个透明背景的新图片，用来绘制文字
    text_layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(text_layer)

    # 计算文本大小
    text_width, text_height = draw.textsize(text, font=font)

    # 创建文字画布
    text_img = Image.new("RGBA", (text_width, text_height), (0,0,0,0))
    text_draw = ImageDraw.Draw(text_img)

    # 绘制白色文本（带黑色边框）
    text_draw.text((0, 0), text, font=font, fill=(255, 0, 0, 255))  # 纯白文字

    # 旋转180度（翻转）
    rotated_text_img = text_img.rotate(180, expand=True)

    # 计算放置旋转文本的中心点
    text_x = position[0] - rotated_text_img.width // 2
    text_y = position[1] + int(rotated_text_img.height * 2.5)

    # 将旋转的文字粘贴到原图上
    img.paste(rotated_text_img, (text_x, text_y), rotated_text_img)


def get_world_position(ego_pos, ego_yaw, ego_pitch, ego_roll, local_offset):
    # 提取变量
    ego_x, ego_y, ego_z = ego_pos
    local_x, local_y, local_z = local_offset

    Rz = np.array([
        [np.cos(ego_yaw), -np.sin(ego_yaw), 0],
        [np.sin(ego_yaw), np.cos(ego_yaw), 0],
        [0, 0, 1]
    ])

    Ry = np.array([
        [np.cos(ego_pitch), 0, np.sin(ego_pitch)],
        [0, 1, 0],
        [-np.sin(ego_pitch), 0, np.cos(ego_pitch)]
    ])

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(ego_roll), -np.sin(ego_roll)],
        [0, np.sin(ego_roll), np.cos(ego_roll)]
    ])

    R = Rz @ Ry @ Rx

    local_offset_vec = np.array([local_x, local_y, local_z])
    world_offset = R @ local_offset_vec  # 旋转局部坐标
    world_pos = np.array([ego_x, ego_y, ego_z]) + world_offset  # 进行平移

    return world_pos

def get_canvas_normal(yaw, pitch, roll):
    # 绕 Z 轴 (yaw)
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    # 绕 Y 轴 (pitch)
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    # 绕 X 轴 (roll)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    # 综合旋转矩阵：旋转顺序为 yaw → pitch → roll
    R = Rz @ Ry @ Rx

    # 在车辆局部坐标系中，前进方向为 [1, 0, 0]
    local_forward = np.array([1, 0, 0])

    # 计算在世界坐标系中的前进方向
    canvas_normal = R @ local_forward

    # 归一化（通常已经是单位向量，但以防万一）
    canvas_normal = canvas_normal / np.linalg.norm(canvas_normal)

    return canvas_normal


def compute_canvas_intersection(eye_pos, traffic_pos, canvas_pos, canvas_yaw, canvas_pitch, canvas_roll, canvas_size):
    eye_pos = np.array(eye_pos, dtype=float)
    traffic_pos = np.array(traffic_pos, dtype=float)
    canvas_pos = np.array(canvas_pos, dtype=float)
    canvas_width, canvas_height = canvas_size

    # 构造旋转矩阵（yaw→pitch→roll，局部坐标系）
    Rz = np.array([
        [np.cos(canvas_yaw), -np.sin(canvas_yaw), 0],
        [np.sin(canvas_yaw), np.cos(canvas_yaw), 0],
        [0, 0, 1]
    ])
    Ry = np.array([
        [np.cos(canvas_pitch), 0, np.sin(canvas_pitch)],
        [0, 1, 0],
        [-np.sin(canvas_pitch), 0, np.cos(canvas_pitch)]
    ])
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(canvas_roll), -np.sin(canvas_roll)],
        [0, np.sin(canvas_roll), np.cos(canvas_roll)]
    ])
    R = Rz @ Ry @ Rx

    # 计算Canvas法向量（世界坐标系）
    canvas_normal = R[:, 0]  # 直接取第一列，避免矩阵乘法
    canvas_normal /= np.linalg.norm(canvas_normal)  # 确保单位化

    # 计算直线与平面交点
    d = traffic_pos - eye_pos
    denom = np.dot(canvas_normal, d)
    if abs(denom) < 1e-6:
        return None, False, np.array([0,0])
    direction =  canvas_pos - eye_pos
    direction /= np.linalg.norm(direction)
    t = np.dot(canvas_normal,direction) / denom
    if t < 0:  # 交点在视线反方向，无效
        return None, False, np.array([0,0])
    intersection = eye_pos + t * d

    # 转换到局部坐标系并判断范围
    local_coords = R.T @ (intersection - eye_pos)
    y_local, z_local = local_coords[1], local_coords[2]
    half_width, half_height = canvas_width / 2, canvas_height / 2
    is_inside = (-half_width <= y_local <= half_width) and (-half_height <= z_local <= half_height)

    return intersection, is_inside,np.array([y_local, z_local])