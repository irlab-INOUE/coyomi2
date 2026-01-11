"""
This script interfaces with the URG series of laser rangefinders to visualize 
the scanned data in real-time.

Attributes:
    XMIN (int): Minimum x-coordinate of the plot in millimeters.
    XMAX (int): Maximum x-coordinate of the plot in millimeters.
    YMIN (int): Minimum y-coordinate of the plot in millimeters.
    YMAX (int): Maximum y-coordinate of the plot in millimeters.

Note:
    This script requires a URG series laser rangefinder connected to the system,
    and the appropriate serial port must be specified when creating 
    an instance of the `Urg` class.

Example:
    To run the script, ensure that the URG device is connected and specify the correct serial port:
        $ python main.py
"""

import math
import sys
import matplotlib.pyplot as plt
import numpy as np
import time
import serial
from math import pi, cos, sin
import cv2
import copy
from PIL import Image, ImageDraw, ImageFont

import Lidar

# 図中のテキスト属性
font = cv2.FONT_HERSHEY_SIMPLEX                                                                                  
font_scale = 1                                                                                                   
font_color = (255, 0, 0)                                                                                     
thickness = 2                                                                                                    


DEBUG_MODE = False
SENSOR_DIRECTION_DEGREE = 90.0 # センサーの正面方向（度）
#SENSOR_DIRECTION_DEGREE = 180.0 # センサーの正面方向（度）

XMIN = -5000 #[mm]
XMAX =  5000 #[mm]
YMIN = -5000 #[mm]
YMAX =  5000 #[mm]

print(f"DEBUG MODE -> {DEBUG_MODE}")

urg = Lidar.Urg('/dev/cu.usbmodem1101', 115200)

# 色指定の形式変換
def hex_to_rgb(hex_color):
    hex_color = hex_color.lstrip('#')
    return tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))

try:
    count = 0
    count_max = 200
    center_range = []
    center_range2 = []
    time_stamp = []

    start_angle = -135.0
    end_angle = 135.0
    step_angle = 0.25
    echo_size = 3

    
    height = 2000
    width = 3000
    IMG_ORIGIN_X = width // 2
    IMG_ORIGIN_Y = height // 2

    csize = 0.0125
    img_org = np.zeros((height, width, 3), dtype=np.uint8)
    img_org[:] = hex_to_rgb('#e6e7ed')
    cv2.line(img_org, (0, IMG_ORIGIN_Y), (width, IMG_ORIGIN_Y), hex_to_rgb('#6c6e75'), 1)
    cv2.line(img_org, (IMG_ORIGIN_X, 0),  (IMG_ORIGIN_X, height), hex_to_rgb('#6c6e75'), 1)
    ticks = np.arange(-width/2*csize, width/2*csize + 0.5, 1)
    for i in ticks:
        if i == 0:
            continue
        tick_x = int(i / csize)
        cv2.line(img_org, (IMG_ORIGIN_X - tick_x, IMG_ORIGIN_Y-10), (IMG_ORIGIN_X - tick_x, IMG_ORIGIN_Y + 10), hex_to_rgb('#6c6e75'), 1)
        cv2.line(img_org, (IMG_ORIGIN_X -10, IMG_ORIGIN_Y - tick_x), (IMG_ORIGIN_X + 10, IMG_ORIGIN_Y- tick_x), hex_to_rgb('#6c6e75'), 1)

    # 指定した角度にセンサー正面の矢印を描画
    direction_rad = SENSOR_DIRECTION_DEGREE * math.pi / 180.0
    arrow_length = 300
    arrow_start = (IMG_ORIGIN_X, IMG_ORIGIN_Y)
    arrow_end_x = int(IMG_ORIGIN_X + arrow_length * math.cos(direction_rad))
    arrow_end_y = int(IMG_ORIGIN_Y - arrow_length * math.sin(direction_rad)) # 画面のY軸は下向きが正のため
    arrow_end = (arrow_end_x, arrow_end_y)
    arrow_color = (0, 0, 255) # BGRで赤色
    arrow_thickness = 5
    cv2.arrowedLine(img_org, arrow_start, arrow_end, arrow_color, arrow_thickness, tipLength=0.3)

    # 矢印の先端にテキストを描画
    # Pillowを使った描画処理
    pil_img = Image.fromarray(cv2.cvtColor(img_org, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil_img)

    # フォント設定
    font_path = '/System/Library/Fonts/ヒラギノ角ゴシック W3.ttc'
    font_size = 50 # ユーザーの変更を反映
    try:
        pil_font = ImageFont.truetype(font_path, font_size)
    except IOError:
        print(f"フォントファイルが見つかりません: {font_path}. デフォルトフォントを使用します.")
        pil_font = ImageFont.load_default()

    text = "センサー正面方向"
    bbox = draw.textbbox((0, 0), text, font=pil_font)
    text_width = bbox[2] - bbox[0]
    text_height = bbox[3] - bbox[1]

    # 矢印の象限に応じてテキスト位置を調整
    margin = 10
    # X方向の位置決め
    if math.cos(direction_rad) >= 0: # 右半分 (第1, 4象限)
        text_pos_x = arrow_end_x + margin
    else: # 左半分 (第2, 3象限)
        text_pos_x = arrow_end_x - text_width - margin

    # Y方向の位置決め
    if math.sin(direction_rad) >= 0: # 上半分 (第1, 2象限)
        text_pos_y = arrow_end_y - text_height - margin
    else: # 下半分 (第3, 4象限)
        text_pos_y = arrow_end_y + margin
    
    text_position = (int(text_pos_x), int(text_pos_y))

    draw.text(text_position, text, font=pil_font, fill=font_color)

    # PillowのRGB形式からOpenCVのBGR形式へ変換
    img_org = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)    
    img = copy.deepcopy(img_org)
    cv2.imshow("LiDAR", img)

    with open("urg_data.txt", "w") as file:
        while True:
            success, urg_data = urg.one_shot()
            #success, urg_data = urg.one_shot_intensity()
            data_size = 1081*3
            if success is True:
                #pass
                ts = int(time.time() * 1e3)
                file.write(f"LASERSCANRT {ts} {data_size} {start_angle} {end_angle} {step_angle} {echo_size} ")
                for d in urg_data:
                    file.write(f"{d[1]} 0 0 ")
                file.write(f"{ts}\n")
            else:
                print("False", file=sys.stderr)

            x = []
            y = []
            # SENSOR_DIRECTION_DEGREE をラジアンに変換
            direction_rad = SENSOR_DIRECTION_DEGREE * pi / 180.0
            cos_dir = cos(direction_rad)
            sin_dir = sin(direction_rad)

            for index, d in enumerate(urg_data):
                angle = (index * step_angle + start_angle) * pi/180
                
                # LiDARローカル座標
                xd_local = d[1] * cos(angle) / 1000
                yd_local = d[1] * sin(angle) / 1000

                # SENSOR_DIRECTION_DEGREE だけ回転
                xd = xd_local * cos_dir - yd_local * sin_dir
                yd = xd_local * sin_dir + yd_local * cos_dir

                x.append(xd)
                y.append(yd)

            img = copy.deepcopy(img_org)
            for px, py in zip(x, y):
                # px = -tmp_px
                ix =int( px / csize + IMG_ORIGIN_X)
                iy =int(-py / csize + IMG_ORIGIN_Y)
                if ix >= 0 and ix < width and iy >= 0 and iy < height:
                    cv2.rectangle(img, (ix-2, iy-2), (ix+2, iy+2), hex_to_rgb('#33635c'), -1)
            cv2.imshow("LiDAR", img)
            key = cv2.waitKey(5) & 0xFF
            if key == ord('q'):
                urg.close()
                print("Exiting program...")
                break

            #plt.clf()
            #plt.xlim(XMIN, XMAX)
            #plt.ylim(YMIN, YMAX)
            #plt.scatter(x, y, s=1)
            #plt.grid()
            #plt.draw()  # プロットを更新
            #plt.pause(0.001)  # 短時間待機
##print(count, "data recieved")
#count += 1
#center_range.append(urg_data[angle2index(0)][1])
#time_stamp.append(count)
#print(f"count:{count} range: {center_range[-1]}")
#if count > count_max:
#    break
#
except KeyboardInterrupt:
    # Ctrl-Cの後処理
    print("Pressed Ctrl-C")
    # シリアル接続を閉じる
    urg.close()
    print("Bye")
