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
center_x = 160 #screen's x center cordinate /スクリーンのx軸中心座標
center_y = 120 #screen's y center cordinate /スクリーンのy軸中心座標
goal_threshold = (120, 255, 120, 190, 30, 135)#黄色ゴール閾値
#goal_threshold = (8, 66, 30, 81, 68, 135) #青色ゴール閾値(RGB)
my_goal_threshold = (135, 220, 120, 180, 30, 120)#黄色ゴール閾値　自ゴール検知用
#my_goal_threshold = (8, 66, 30, 81, 68, 135) #青色ゴール閾値(RGB)
radius_1= 80 # inner circle's radius /内円半径
radius_2 = 110 #outer ciecle's radius /外円半径
radius_3 = 95 #middle circle7s radius /中間円半径
radius_1= 80 # inner circle's radius /内円半径
radius_2 = 110 #outer ciecle's radius /外円半径
radius_3 = 95 #middle circle7s radius /中間円半径
angle_st = 45 #circular checking start angle /円形スキャン開始角度
angle_st_const = angle_st
angle_end = 315 #circular checking end angle /円形スキャン終了角度
angle_end_const = angle_end
angle_param = 80 #次回円形スキャンの範囲＋ー
ang_margin = 5 #誤検知防止スキャン用角度
split = 11 #敵検知用にゴール角度を分割する個数
enemy_skip = 4 #敵スキャン時スキャンしていくピクセルの間隔

#circle cordinate tables /円座標テーブル
#inner circle's tabele /内円テーブル
X_table1 = (240, 239, 239, 239, 239, 239, 239, 239, 239, 239, 238, 238, 238, 237, 237, 237, 236, 236, 236, 235, 235, 234, 234, 233, 233, 232, 231, 231, 230, 229, 229, 228, 227, 227, 226, 225, 224, 223, 223, 222, 221, 220, 219, 218, 217, 216, 215, 214, 213, 212, 211, 210, 209, 208, 207, 205, 204, 203, 202, 201, 200, 198, 197, 196, 195, 193, 192, 191, 189, 188, 187, 186, 184, 183, 182, 180, 179, 177, 176, 175, 173, 172, 171, 169, 168, 166, 165, 164, 162, 161, 160, 159, 158, 156, 155, 154, 152, 151, 149, 148, 147, 145, 144, 143, 141, 140, 138, 137, 136, 134, 133, 132, 131, 129, 128, 127, 125, 124, 123, 122, 121, 119, 118, 117, 116, 115, 113, 112, 111, 110, 109, 108, 107, 106, 105, 104, 103, 102, 101, 100, 99, 98, 97, 97, 96, 95, 94, 93, 93, 92, 91, 91, 90, 89, 89, 88, 87, 87, 86, 86, 85, 85, 84, 84, 84, 83, 83, 83, 82, 82, 82, 81, 81, 81, 81, 81, 81, 81, 81, 81, 80, 81, 81, 81, 81, 81, 81, 81, 81, 81, 82, 82, 82, 83, 83, 83, 84, 84, 84, 85, 85, 86, 86, 87, 87, 88, 89, 89, 90, 91, 91, 92, 93, 93, 94, 95, 96, 97, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 115, 116, 117, 118, 119, 120, 122, 123, 124, 125, 127, 128, 129, 131, 132, 133, 134, 136, 137, 138, 140, 141, 143, 144, 145, 147, 148, 149, 151, 152, 154, 155, 156, 158, 159, 160, 161, 162, 164, 165, 166, 168, 169, 171, 172, 173, 175, 176, 177, 179, 180, 182, 183, 184, 186, 187, 188, 189, 191, 192, 193, 195, 196, 197, 198, 199, 201, 202, 203, 204, 205, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 223, 224, 225, 226, 227, 227, 228, 229, 229, 230, 231, 231, 232, 233, 233, 234, 234, 235, 235, 236, 236, 236, 237, 237, 237, 238, 238, 238, 239, 239, 239, 239, 239, 239, 239, 239, 239, 240)
Y_table1 = (120, 121, 122, 124, 125, 126, 128, 129, 131, 132, 133, 135, 136, 137, 139, 140, 142, 143, 144, 146, 147, 148, 149, 151, 152, 153, 155, 156, 157, 158, 159, 161, 162, 163, 164, 165, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 183, 184, 185, 186, 187, 187, 188, 189, 189, 190, 191, 191, 192, 193, 193, 194, 194, 195, 195, 196, 196, 196, 197, 197, 197, 198, 198, 198, 199, 199, 199, 199, 199, 199, 199, 199, 199, 200, 199, 199, 199, 199, 199, 199, 199, 199, 199, 198, 198, 198, 197, 197, 197, 196, 196, 196, 195, 195, 194, 194, 193, 193, 192, 191, 191, 190, 189, 189, 188, 187, 187, 186, 185, 184, 183, 183, 182, 181, 180, 179, 178, 177, 176, 175, 174, 173, 172, 171, 170, 169, 168, 167, 165, 164, 163, 162, 161, 160, 158, 157, 156, 155, 153, 152, 151, 149, 148, 147, 146, 144, 143, 142, 140, 139, 137, 136, 135, 133, 132, 131, 129, 128, 126, 125, 124, 122, 121, 120, 119, 118, 116, 115, 114, 112, 111, 109, 108, 107, 105, 104, 103, 101, 100, 98, 97, 96, 94, 93, 92, 91, 89, 88, 87, 85, 84, 83, 82, 81, 79, 78, 77, 76, 75, 73, 72, 71, 70, 69, 68, 67, 66, 65, 64, 63, 62, 61, 60, 59, 58, 57, 57, 56, 55, 54, 53, 53, 52, 51, 51, 50, 49, 49, 48, 47, 47, 46, 46, 45, 45, 44, 44, 44, 43, 43, 43, 42, 42, 42, 41, 41, 41, 41, 41, 41, 41, 41, 41, 40, 41, 41, 41, 41, 41, 41, 41, 41, 41, 42, 42, 42, 43, 43, 43, 44, 44, 44, 45, 45, 46, 46, 47, 47, 48, 49, 49, 50, 51, 51, 52, 53, 53, 54, 55, 56, 57, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 75, 76, 77, 78, 79, 80, 82, 83, 84, 85, 87, 88, 89, 91, 92, 93, 94, 96, 97, 98, 100, 101, 103, 104, 105, 107, 108, 109, 111, 112, 114, 115, 116, 118, 119, 120)

target_ang = 0 #robot moving angle /ロボットの移動する角度
kick_ang = 0 #robot facing angle /ロボットが向く角度
sent_data = [0x66, 0, 0x54, 0, 0] #sending data list /送信するデータリスト


def radial_checker(max_ang,min_ang,cent_ang,split,start_radius,end_radius,appli_threshold):
    appli_distance =[0]*split
    appli_id1 = []
    appli_cord = [(0,0)]*split
    split_ang = (max_ang - min_ang)/(split-1) #ゴールを検知した角度を分割
    appli_ang = []
    appli_ang = [min_ang + 2, cent_ang - split_ang*4, cent_ang - split_ang*3, cent_ang - split_ang*2, cent_ang - split_ang, cent_ang, cent_ang + split_ang, cent_ang + split_ang*2,  cent_ang + split_ang*3, cent_ang + split_ang*4, max_ang-2]
    shoot_ang = cent_ang
    for i in range(split):
       for dist in range(start_radius,end_radius,enemy_skip): #分割した角度でenemy_skipとばしでゴールにあたるまでスキャン
            Rx = dist*math.cos(math.radians(appli_ang[i])) #R cos θ
            Ry = dist*math.sin(math.radians(appli_ang[i])) #R sin θ
            Rxin = center_x + int(Rx)
            Ryin = center_y + int(Ry)
            colorRGB = img.get_pixel(Rxin,Ryin) #get RGB color
            #colorLAB = image.rgb_to_lab(colorRGB) #change RGB to LAB
            appli_cord[i] = (Rxin,Ryin)#//
            #img.draw_circle(Rxin,Ryin,1) #//
            if colorRGB[0] >= appli_threshold[0] and colorRGB[0] <= appli_threshold[1] and colorRGB[1] >= appli_threshold[2] and colorRGB[1] <= appli_threshold[3] and colorRGB[2] >= appli_threshold[4] and colorRGB[2] <= appli_threshold[5]:
            #if colorLAB[0] >= goal_threshold[0] and colorLAB[0] <= goal_threshold[1] and colorLAB[1] >= goal_threshold[2] and colorLAB[1] <= goal_threshold[3] and colorLAB[2] >= goal_threshold[4] and colorLAB[2] <= goal_threshold[5]:
               img.draw_circle(Rxin,Ryin,1,color=(0,255,0)) #//
               appli_cord[i] = (Rxin,Ryin)#//
               appli_distance[i] = dist #ゴールにあたったらその距離をリストに格納
               break
            appli_distance2 = [a for a in appli_distance if a !=0]
    
    if len(appli_distance2) == 0: #何も検知できなかった
        return 2
    else:
        min_distance = min(appli_distance2)
        app_index = appli_distance.index(min_distance)
        appli_lange = 1.3242*math.exp(0.0388*min_distance)#ゴール距離を計算
        if appli_lange >= 20:#ゴール距離が遠すぎないか
            print("lange over")
            sent_data[3] = 0
            #kick_ang = 0
            shoot_ang = appli_ang[app_index]
            Xshot = X_table1[int(shoot_ang)] #//
            Yshot = Y_table1[int(shoot_ang)] #//
            img.draw_line(center_x,center_y,Xshot ,Yshot ,color = (0,255,0)) #//
            sent_data[1] = int((angle_mirror(shoot_ang,0)/360)*255)
            #target_ang = int((angle_mirror(shoot_ang)/360)*255)
            #print(target_ang)
            uart_sender()
            return 0
        else:
            return 1


#correct angle function /角度補正用関数
def angle_mirror(angle):
   real_ang = 360 - angle
   if real_ang >= 180:
       real_ang -= 180
   else:
       real_ang += 180
   return real_ang


#Wrapping around function (you need adjust) /回り込み用関数（調整必要）
def mawarikomi(ang, lange): 
    if ang < 20 or ang > 340:
        ang_m = 1000
    else:
        ang_m = 1
    lange_m = lange
    if lange > 0:
        if lange > 30:
            lange_m = 10
        elif ang < 13:
            lange_m = 1
        else:
            lange_m = -5.8822 + 0.5294*lange
    move_ang_1 = 100*(1/lange_m)*(1/ang_m)
    if ang <= 180: #in case of objective angle is right side /目標物のある角度が右側であった場合
       move_ang = ang + move_ang_1
    elif ang > 180: #in case of objective angle is left side /目標物のある角度が左側であった場合
       move_ang = ang - move_ang_1
    else:#in case of objective angle is in front /目標物のある角度が正面であった場合
       move_ang = ang
    #print("move_ang",move_ang)
    return move_ang


#function of sending data in UART /UARTでデータを送信する用関数
#data format -> [header(0x66),target_ang,header(0x54),kick_ang,checksum] /データフォーマット->[ヘッダ(0x66),移動角度, ヘッダ(0x54)、 シュート角度,SUM]
def uart_sender():
    sent_data[1] = target_ang
    sent_data[3] = kick_ang
    uart_sum = sum(sent_data[0:3])&0xFF
    sent_data[4] = uart_sum
    print("ball_ang=",sent_data[1],"shoot_ang=",sent_data[3],"SUM=",sent_data[4]) #out put from P1 pin
    for i in range(4):
        uart.writechar(sent_data[i])

clock = time.clock()
uart = UART(3, 115200, timeout_char = 10) # uart start
uart.init(115200, bits=8, parity=None, stop=1, timeout_char=1000)

#main codes /メインコード
while(True):
    clock.tick()
    img = sensor.snapshot()
    print(clock.fps())
    radial_result=radial_checker(360,0,180,11,30,120,my_goal_threshold) #(スキャン終了角度、スキャン終了角度、スキャン中心角度、角度分割数、スキャン開始半径、スキャン終了半径、閾値、モード｛０＝自ゴール距離　１＝敵スキャン｝）
    #print(radial_result)
    if radial_result == 0:
        continue
    elif radial_result == 1:#ゴールが遠くなかった→ボールを探す
        continue
    else: #ゴールを検知できなかった
        sent_data[1] = 127#後ろ向きに移動------------------------------------------------------------------------------------------------------------------
        sent_data[3] = 0
        uart_sender()
        continue
