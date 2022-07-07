#this code was written by Yohjustk in 2022

#import libraries /ライブラリを読み込み
import sensor, image, time,math,pyb
from pyb import UART
from pyb import Pin
from pyb import ADC

#sensor set up /カメラの設定
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA) #320*240px
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor. set_auto_exposure(False)

#global variables declaration /グローバル変数を宣言
wall_threshold = (10,45,20,50,5,50) #壁検知用閾値（RGB)
center_x = 160 #screen's x center cordinate /スクリーンのx軸中心座標
center_y = 120 #screen's y center cordinate /スクリーンのy軸中心座標
wall_cord = [(100,0),(0,100),(-100,0),(0,-100)] #壁検知スキャン座標用リスト4方向ver
wall_result = 0
target_ang = 0 #robot moving angle /ロボットの移動する角度
kick_ang = 0 #robot facing angle /ロボットが向く角度
sent_data = [0x66, 0, 0x54, 0, 0] #sending data list /送信するデータリスト

def wall_checker():
    scan_result = 0
    for k in range(4):
        wall_result = 0
        cord_x = wall_cord[k][0]
        cord_y = wall_cord[k][1]
        x_list = (cord_x-15,cord_x-14,cord_x-13,cord_x-12,cord_x-11,cord_x-10,cord_x-9,cord_x-8,cord_x-7,cord_x-6,cord_x-5,cord_x-4,cord_x-3,cord_x-2,cord_x-1,cord_x,cord_x+1,cord_x+2,cord_x+3,cord_x+4,cord_x+5,cord_x+6,cord_x+7,cord_x+8,cord_x+9,cord_x+10,cord_x+11,cord_x+12,cord_x+13,cord_x+14,cord_x+15)
        y_list = (cord_y-15,cord_y-14,cord_y-13,cord_y-12,cord_y-11,cord_y-10,cord_y-9,cord_y-8,cord_y-7,cord_y-6,cord_y-5,cord_y-4,cord_y-3,cord_y-2,cord_y-1,cord_y,cord_y+1,cord_y+2,cord_y+3,cord_y+4,cord_y+5,cord_y+6,cord_y+7,cord_y+8,cord_y+9,cord_y+10,cord_y+11,cord_y+12,cord_y+13,cord_y+14,cord_y+15)
        for x in x_list:
            for y in y_list:
                #31*31ピクセルの四角形内の色をスキャン
                Wx = x + center_x
                Wy = y + center_y
                colorRGB = img.get_pixel(Wx,Wy) #get RGB color
                #print(colorRGB)
                if colorRGB[0] >= wall_threshold[0] and colorRGB[0] <= wall_threshold[1] and colorRGB[1] >= wall_threshold[2] and colorRGB[1] <= wall_threshold[3] and colorRGB[2] >= wall_threshold[4] and colorRGB[2] <= wall_threshold[5]:
                    wall_result+= 1
                    #image.set_pixel(cord_x, cord_y, 0,255,0)
        if wall_result >= 950: #壁があると判定(961ピクセル内950が黒）
            img.draw_circle(cord_x+center_x, cord_y+center_y,1,color=(0,255,0))
            if k == 0:
                scan_result += 1
            elif k == 1:
                scan_result += 3
            elif k == 2:
                scan_result += 5
            elif k == 3:
                scan_result += 13
    if scan_result == 0:#free
        return 2
    elif scan_result == 1 or scan_result == 17:#後ろに壁がある
        sent_data[1] = 0
        sent_data[3] = 0
        return 0
    elif scan_result == 3 or scan_result == 9:#
        sent_data[1] = int((angle_mirror(270)/360)*255)
        sent_data[3] = 0
        return 0
    elif scan_result == 5 or scan_result == 21:#
        sent_data[1] = int((angle_mirror(0)/360)*255)
        sent_data[3] = 0
        return 0
    elif scan_result == 13 or scan_result == 19:#
        sent_data[1] = int((angle_mirror(90)/360)*255)
        sent_data[3] = 0
        return 0
    elif scan_result == 4:#1+3
        sent_data[1] = int((angle_mirror(225)/360)*255)
        sent_data[3] = 0
        return 0
    elif scan_result ==8 :#3+5
        sent_data[1] = int((angle_mirror(315)/360)*255)
        sent_data[3] = 0
        return 0
    elif scan_result == 18:#5+13
        sent_data[1] = int((angle_mirror(45)/360)*255)
        sent_data[3] = 0
        return 0
    elif scan_result == 14:#13+1
        sent_data[1] = int((angle_mirror(135)/360)*255)
        sent_data[3] = 0
        return 0
    elif scan_result == 22 or scan_result == 6 or scan_result == 16:#stop
        return 1


def angle_mirror(angle): #角度補正用関数
   real_ang = 360 - angle #正面となる角度（１８０度）を０度となるように回転させ、ミラーを補正するために反転
   if real_ang >= 180:
       real_ang -= 180
   else:
       real_ang += 180
   return real_ang

def uart_sender(): #データ送信用関数
    #sent_data[1] = target_ang
    #sent_data[3] = kick_ang
    uart_sum = sum(sent_data[0:3])&0xFF
    sent_data[4] = uart_sum
    print("target_ang=",sent_data[1],"kick_ang=",sent_data[3],"SUM=",sent_data[4]) #P1
    for i in range(4):
        uart.writechar(sent_data[i])

clock = time.clock()
uart = UART(3, 115200, timeout_char = 10) # uart start
uart.init(115200, bits=8, parity=None, stop=1, timeout_char=1000)

while(True):
    clock.tick()
    img = sensor.snapshot()
    print(clock.fps())
    wall = wall_checker() #
    if wall == 0:
        uart_sender()
        continue
    elif wall == 1:
        continue
    #elif wall == 2
        #メインに続く
    #uart_sender()