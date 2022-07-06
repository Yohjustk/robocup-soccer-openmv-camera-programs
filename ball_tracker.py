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
ball_threshold = [(30, 85, 35, 83, -10, 69)] #ball color threshold in LAB color space[Lmin,Lmax,Amin,Amax,Bmin,Bmax] / LAB色座標系でのボール色閾値[L最小値,L最大値,A最小値,A最大値,B最小値,B最大値]
ball_cordinate1 = [] #ball sizes and x,ycordinates list /ボールサイズ・座標x,y一時格納用リスト
center_x = 160 #screen's x center cordinate /スクリーンのx軸中心座標
center_y = 120 #screen's y center cordinate /スクリーンのy軸中心座標
target_ang = 0 #robot moving angle /ロボットの移動する角度
kick_ang = 0 #robot facing angle /ロボットが向く角度
sent_data = [0x66, 0, 0x54, 0, 0] #sending data list /送信するデータリスト


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


#main codes /メインコード
while(True):
    clock.tick()
    img = sensor.snapshot()
    print(clock.fps())
    for blob in img.find_blobs(ball_threshold, pixels_threshold = 30, area_threshold = 30):
            ball_x = blob.cx() #ball's x coordinate
            ball_y = blob.cy() #ball's y coordinate
            
            #aviod detecting color of camera as ball(that is red) /カメラ基盤の色をボールとして誤検知することを避ける
            if ball_x > 140 and  ball_x < 170 and ball_y > 90 and ball_y < 130: 
                continue
            else:
                ball_cordinate1.append((blob.area(),ball_x,ball_y))


    if len(ball_cordinate1) > 0:#ball was detected /ボールを検知した
        
        #extract elements of ball_cordinate1 /ball_cordinate1の要素を抜き出し
        ball_check = [0]*len(ball_cordinate1)
        for i in range(len(ball_cordinate1)-1):
            ball_check[i] = ball_cordinate1[i][0]
        max_index =  ball_check.index(max(ball_check)) ##detect a element including a biggest size as a ball /最も大きいsizeを含む要素をボールとして検知
        ball_size = ball_cordinate1[max_index][0]
        Xball = ball_cordinate1[max_index][1]
        Yball = ball_cordinate1[max_index][2]

        #draw a cross on ball in the screen /画面のボールに十字を描く
        img.draw_cross(Xball,Yball,color=(0,255,0)) 
        
        #convert ball cordinates /ボール座標を画面中心が中心の座標系へ変換
        Xball_re = ball_cordinate1[max_index][1] - center_x
        Yball_re = ball_cordinate1[max_index][2] - center_y

        #calculate lange to the ball from the robot /ロボットからボールまでの距離を計算
        silhet_lange = math.sqrt((Xball_re)**2+(Yball_re)**2)
        ball_lange = 1.3242*math.exp(0.0388*silhet_lange) #you should change this formula because it was veried with shape of mirror /ミラーによって変化するためこの式は変更する必要がある
        
        #calulate angle of the ball /ボールのある角度を計算
        if Xball_re > 0:
            if Yball_re > 0:#0 ~ pi/2
                ball_ang_rad = math.atan(Yball_re/Xball_re)
            elif Yball_re < 0: #3*pi/2 ~ 2pi
                ball_ang_rad = 2*math.pi+math.atan(Yball_re/Xball_re)
            else: #0
                ball_ang_rad = 0
        elif Xball_re < 0:
            if Yball_re > 0: #pi/2 ~ pi
                ball_ang_rad = math.pi + math.atan(Yball_re/Xball_re)
            elif Yball_re < 0: #pi ~ 2*pi/3
                ball_ang_rad = math.pi + math.atan(Yball_re/Xball_re)
            else: #pi
                ball_ang_rad = math.pi
        else:
            if Yball_re > 0:
                ball_ang_rad = math.pi/2
            if Yball_re < 0:
                ball_ang_rad = 3*math.pi/2

        #convert angle to sent data format /送信するデータ形式に角度を変換
        ball_ang_deg = math.degrees(ball_ang_rad)
        movement_ang = mawarikomi(angle_mirror(ball_ang_deg), ball_lange)
        target_ang = int((movement_ang/360)*255)
        kick_ang = 0
        uart_sender()
        ball_cordinate1 = []
        continue
    else: #ball wasn't detacted /ボールは検知されなかった
        target_ang = 0

    img.draw_cross(center_x, center_y,color=(0,0,255)) #draw center pointer /画面中心に目標を描く