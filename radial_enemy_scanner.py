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
enemy_mg = 8 #ゴール前に敵がいると判断する許容範囲
enemy_skip = 4 #敵スキャン時スキャンしていくピクセルの間隔
shoot_mg = 0 #シュート角度マージン

#circle cordinate tables /円座標テーブル
#inner circle's tabele /内円テーブル
X_table1 = (240, 239, 239, 239, 239, 239, 239, 239, 239, 239, 238, 238, 238, 237, 237, 237, 236, 236, 236, 235, 235, 234, 234, 233, 233, 232, 231, 231, 230, 229, 229, 228, 227, 227, 226, 225, 224, 223, 223, 222, 221, 220, 219, 218, 217, 216, 215, 214, 213, 212, 211, 210, 209, 208, 207, 205, 204, 203, 202, 201, 200, 198, 197, 196, 195, 193, 192, 191, 189, 188, 187, 186, 184, 183, 182, 180, 179, 177, 176, 175, 173, 172, 171, 169, 168, 166, 165, 164, 162, 161, 160, 159, 158, 156, 155, 154, 152, 151, 149, 148, 147, 145, 144, 143, 141, 140, 138, 137, 136, 134, 133, 132, 131, 129, 128, 127, 125, 124, 123, 122, 121, 119, 118, 117, 116, 115, 113, 112, 111, 110, 109, 108, 107, 106, 105, 104, 103, 102, 101, 100, 99, 98, 97, 97, 96, 95, 94, 93, 93, 92, 91, 91, 90, 89, 89, 88, 87, 87, 86, 86, 85, 85, 84, 84, 84, 83, 83, 83, 82, 82, 82, 81, 81, 81, 81, 81, 81, 81, 81, 81, 80, 81, 81, 81, 81, 81, 81, 81, 81, 81, 82, 82, 82, 83, 83, 83, 84, 84, 84, 85, 85, 86, 86, 87, 87, 88, 89, 89, 90, 91, 91, 92, 93, 93, 94, 95, 96, 97, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 115, 116, 117, 118, 119, 120, 122, 123, 124, 125, 127, 128, 129, 131, 132, 133, 134, 136, 137, 138, 140, 141, 143, 144, 145, 147, 148, 149, 151, 152, 154, 155, 156, 158, 159, 160, 161, 162, 164, 165, 166, 168, 169, 171, 172, 173, 175, 176, 177, 179, 180, 182, 183, 184, 186, 187, 188, 189, 191, 192, 193, 195, 196, 197, 198, 199, 201, 202, 203, 204, 205, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 223, 224, 225, 226, 227, 227, 228, 229, 229, 230, 231, 231, 232, 233, 233, 234, 234, 235, 235, 236, 236, 236, 237, 237, 237, 238, 238, 238, 239, 239, 239, 239, 239, 239, 239, 239, 239, 240)
Y_table1 = (120, 121, 122, 124, 125, 126, 128, 129, 131, 132, 133, 135, 136, 137, 139, 140, 142, 143, 144, 146, 147, 148, 149, 151, 152, 153, 155, 156, 157, 158, 159, 161, 162, 163, 164, 165, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 183, 184, 185, 186, 187, 187, 188, 189, 189, 190, 191, 191, 192, 193, 193, 194, 194, 195, 195, 196, 196, 196, 197, 197, 197, 198, 198, 198, 199, 199, 199, 199, 199, 199, 199, 199, 199, 200, 199, 199, 199, 199, 199, 199, 199, 199, 199, 198, 198, 198, 197, 197, 197, 196, 196, 196, 195, 195, 194, 194, 193, 193, 192, 191, 191, 190, 189, 189, 188, 187, 187, 186, 185, 184, 183, 183, 182, 181, 180, 179, 178, 177, 176, 175, 174, 173, 172, 171, 170, 169, 168, 167, 165, 164, 163, 162, 161, 160, 158, 157, 156, 155, 153, 152, 151, 149, 148, 147, 146, 144, 143, 142, 140, 139, 137, 136, 135, 133, 132, 131, 129, 128, 126, 125, 124, 122, 121, 120, 119, 118, 116, 115, 114, 112, 111, 109, 108, 107, 105, 104, 103, 101, 100, 98, 97, 96, 94, 93, 92, 91, 89, 88, 87, 85, 84, 83, 82, 81, 79, 78, 77, 76, 75, 73, 72, 71, 70, 69, 68, 67, 66, 65, 64, 63, 62, 61, 60, 59, 58, 57, 57, 56, 55, 54, 53, 53, 52, 51, 51, 50, 49, 49, 48, 47, 47, 46, 46, 45, 45, 44, 44, 44, 43, 43, 43, 42, 42, 42, 41, 41, 41, 41, 41, 41, 41, 41, 41, 40, 41, 41, 41, 41, 41, 41, 41, 41, 41, 42, 42, 42, 43, 43, 43, 44, 44, 44, 45, 45, 46, 46, 47, 47, 48, 49, 49, 50, 51, 51, 52, 53, 53, 54, 55, 56, 57, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 75, 76, 77, 78, 79, 80, 82, 83, 84, 85, 87, 88, 89, 91, 92, 93, 94, 96, 97, 98, 100, 101, 103, 104, 105, 107, 108, 109, 111, 112, 114, 115, 116, 118, 119, 120)
#outer circle's table /外円テーブル
X_table2 = (270, 269, 269, 269, 269, 269, 269, 269, 268, 268, 268, 267, 267, 267, 266, 266, 265, 265, 264, 264, 263, 262, 261, 261, 260, 259, 258, 258, 257, 256, 255, 254, 253, 252, 251, 250, 248, 247, 246, 245, 244, 243, 241, 240, 239, 237, 236, 235, 233, 232, 230, 229, 227, 226, 224, 223, 221, 219, 218, 216, 215, 213, 211, 209, 208, 206, 204, 202, 201, 199, 197, 195, 193, 192, 190, 188, 186, 184, 182, 180, 179, 177, 175, 173, 171, 169, 167, 165, 163, 161, 160, 159, 157, 155, 153, 151, 149, 147, 145, 143, 141, 140, 138, 136, 134, 132, 130, 128, 127, 125, 123, 121, 119, 118, 116, 114, 112, 111, 109, 107, 106, 104, 102, 101,  99,  97,  96,  94,  93,  91,  90,  88,  87,  85,  84,  83,  81,  80,  79,  77,  76,  75,  74,  73,  72,  70,  69,  68,  67,  66,  65,  64,  63,  62,  62, 61, 60, 59, 59, 58, 57, 56, 56, 55, 55, 54, 54, 53, 53, 53, 52, 52, 52, 51, 51, 51, 51, 51, 51, 51, 50, 51, 51, 51, 51, 51, 51, 51, 52, 52, 52, 53, 53, 53, 54, 54, 55, 55, 56, 56, 57, 58, 59, 59, 60, 61, 62, 62, 63, 64, 65, 66, 67, 68, 69, 70, 72, 73, 74, 75, 76, 77, 79, 80, 81, 83, 84, 85, 87, 88, 90, 91, 93, 94, 96, 97, 99, 101, 102, 104, 105, 107, 109, 111, 112, 114, 116, 118, 119, 121, 123, 125, 127, 128, 130, 132, 134, 136, 138, 140, 141, 143, 145, 147, 149, 151, 153, 155, 157, 159, 160, 161, 163, 165, 167, 169, 171, 173, 175, 177, 179, 180, 182, 184, 186, 188, 190, 192, 193, 195, 197, 199, 201, 202, 204, 206, 208, 209, 211, 213, 214, 216, 218, 219, 221, 223, 224, 226, 227, 229, 230, 232, 233, 235, 236, 237, 239, 240, 241, 243, 244, 245, 246, 247, 248, 250, 251, 252, 253, 254, 255, 256, 257, 258, 258, 259, 260, 261, 261, 262, 263, 264, 264, 265, 265, 266, 266, 267, 267, 267, 268, 268, 268, 269, 269, 269, 269, 269, 269, 269, 270)
Y_table2 = (120, 121, 123, 125, 127, 129, 131, 133, 135, 137, 139, 140, 142, 144, 146, 148, 150, 152, 153, 155, 157, 159, 161, 162, 164, 166, 168, 169, 171, 173, 174, 176, 178, 179, 181, 183, 184, 186, 187, 189, 190, 192, 193, 195, 196, 197, 199, 200, 201, 203, 204, 205, 206, 207, 208, 210, 211, 212, 213, 214, 215, 216, 217, 218, 218, 219, 220, 221, 221, 222, 223, 224, 224, 225, 225, 226, 226, 227, 227, 227, 228, 228, 228, 229, 229, 229, 229, 229, 229, 229, 230, 229, 229, 229, 229, 229, 229, 229, 228, 228, 228, 227, 227, 227, 226, 226, 225, 225, 224, 224, 223, 222, 221, 221, 220, 219, 218, 218, 217, 216, 215, 214, 213, 212, 211, 210, 208, 207, 206, 205, 204, 203, 201, 200, 199, 197, 196, 195, 193, 192, 190, 189, 187, 186, 184, 183, 181, 179, 178, 176, 175, 173, 171, 169, 168, 166, 164, 162, 161, 159, 157, 155, 153, 152, 150, 148, 146, 144, 142, 140, 139, 137, 135, 133, 131, 129, 127, 125, 123, 121, 120, 119, 117, 115, 113, 111, 109, 107, 105, 103, 101, 100, 98, 96, 94, 92, 90, 88, 87, 85, 83, 81, 79, 78, 76, 74, 72, 71, 69, 67, 66, 64, 62, 61, 59, 57, 56, 54, 53, 51, 50, 48, 47, 45, 44, 43, 41, 40, 39, 37, 36, 35, 34, 33, 32, 30, 29, 28, 27, 26, 25, 24, 23, 22, 22, 21, 20, 19, 19, 18, 17, 16, 16, 15, 15, 14, 14, 13, 13, 13, 12, 12, 12, 11, 11, 11, 11, 11, 11, 11, 10, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16, 17, 18, 19, 19, 20, 21, 22, 22, 23, 24, 25, 26, 27, 28, 29, 30, 32, 33, 34, 35, 36, 37, 39, 40, 41, 43, 44, 45, 47, 48, 50, 51, 53, 54, 56, 57, 59, 61, 62, 64, 65, 67, 69, 71, 72, 74, 76, 78, 79, 81, 83, 85, 87, 88, 90, 92, 94, 96, 98, 100, 101, 103, 105, 107, 109, 111, 113, 115, 117, 119, 120)
#middle circle's table /中間円テーブル
X_table3 = [255, 254, 254, 254, 254, 254, 254, 254, 254, 253, 253, 253, 252, 252, 252, 251, 251, 250, 250, 249, 249, 248, 248, 247, 246, 246, 245, 244, 243, 243, 242, 241, 240, 239, 238, 237, 236, 235, 234, 233, 232, 231, 230, 229, 228, 227, 225, 224, 223, 222, 221, 219, 218, 217, 215, 214, 213, 211, 210, 208, 207, 206, 204, 203, 201, 200, 198, 197, 195, 194, 192, 190, 189, 187, 186, 184, 182, 181, 179, 178, 176, 174, 173, 171, 169, 168, 166, 164, 163, 161, 160, 159, 157, 156, 154, 152, 151, 149, 147, 146, 144, 142, 141, 139, 138, 136, 134, 133, 131, 130, 128, 126, 125, 123, 122, 120, 119, 117, 116, 114, 113, 112, 110, 109, 107, 106, 105, 103, 102, 101, 99, 98, 97, 96, 95, 93, 92, 91, 90, 89, 88, 87, 86, 85, 84, 83, 82, 81, 80, 79, 78, 77, 77, 76, 75, 74, 74, 73, 72, 72, 71, 71, 70, 70, 69, 69, 68, 68, 68, 67, 67, 67, 66, 66, 66, 66, 66, 66, 66, 66, 65, 66, 66, 66, 66, 66, 66, 66, 66, 67, 67, 67, 68, 68, 68, 69, 69, 70, 70, 71, 71, 72, 72, 73, 74, 74, 75, 76, 77, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 95, 96, 97, 98, 99, 101, 102, 103, 105, 106, 107, 109, 110, 112, 113, 114, 116, 117, 119, 120, 122, 123, 125, 126, 128, 130, 131, 133, 134, 136, 138, 139, 141, 142, 144, 146, 147, 149, 151, 152, 154, 156, 157, 159, 160, 161, 163, 164, 166, 168, 169, 171, 173, 174, 176, 178, 179, 181, 182, 184, 186, 187, 189, 190, 192, 194, 195, 197, 198, 200, 201, 203, 204, 206, 207, 208, 210, 211, 213, 214, 215, 217, 218, 219, 221, 222, 223, 224, 225, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 243, 244, 245, 246, 246, 247, 248, 248, 249, 249, 250, 250, 251, 251, 252, 252, 252, 253, 253, 253, 254, 254, 254, 254, 254, 254, 254, 254, 255]
Y_table3 = [120, 121, 123, 124, 126, 128, 129, 131, 133, 134, 136, 138, 139, 141, 142, 144, 146, 147, 149, 150, 152, 154, 155, 157, 158, 160, 161, 163, 164, 166, 167, 168, 170, 171, 173, 174, 175, 177, 178, 179, 181, 182, 183, 184, 185, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 203, 204, 205, 206, 206, 207, 208, 208, 209, 209, 210, 210, 211, 211, 212, 212, 212, 213, 213, 213, 214, 214, 214, 214, 214, 214, 214, 214, 215, 214, 214, 214, 214, 214, 214, 214, 214, 213, 213, 213, 212, 212, 212, 211, 211, 210, 210, 209, 209, 208, 208, 207, 206, 206, 205, 204, 203, 203, 202, 201, 200, 199, 198, 197, 196, 195, 194, 193, 192, 191, 190, 189, 188, 187, 185, 184, 183, 182, 181, 179, 178, 177, 175, 174, 173, 171, 170, 168, 167, 166, 164, 163, 161, 160, 158, 157, 155, 154, 152, 150, 149, 147, 146, 144, 142, 141, 139, 138, 136, 134, 133, 131, 129, 128, 126, 124, 123, 121, 120, 119, 117, 116, 114, 112, 111, 109, 107, 106, 104, 102, 101, 99, 98, 96, 94, 93, 91, 90, 88, 86, 85, 83, 82, 80, 79, 77, 76, 74, 73, 72, 70, 69, 67, 66, 65, 63, 62, 61, 59, 58, 57, 56, 55, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 37, 36, 35, 34, 34, 33, 32, 32, 31, 31, 30, 30, 29, 29, 28, 28, 28, 27, 27, 27, 26, 26, 26, 26, 26, 26, 26, 26, 25, 26, 26, 26, 26, 26, 26, 26, 26, 27, 27, 27, 28, 28, 28, 29, 29, 30, 30, 31, 31, 32, 32, 33, 34, 34, 35, 36, 37, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 55, 56, 57, 58, 59, 61, 62, 63, 65, 66, 67, 69, 70, 72, 73, 74, 76, 77, 79, 80, 82, 83, 85, 86, 88, 90, 91, 93, 94, 96, 98, 99, 101, 102, 104, 106, 107, 109, 111, 112, 114, 116, 117, 119, 120]
target_ang = 0 #robot moving angle /ロボットの移動する角度
kick_ang = 0 #robot facing angle /ロボットが向く角度
sent_data = [0x66, 0, 0x54, 0, 0] #sending data list /送信するデータリスト


#circular goal scan functtion /円形ゴールスキャン関数
def circle_checker(R,ang_st,ang_end):

        #check circle size and dicide which table is fine /円のサイズにより適用するテーブルを決定
       if R == radius_1:
           X_table = X_table1
           Y_table = Y_table1
       elif R == radius_2:
           X_table = X_table2
           Y_table = Y_table2
       else:
           X_table = X_table3
           Y_table = Y_table3
       
       #check the color(RGB) of each px /それぞれのピクセルの色を判定
       for ang in range(ang_st,ang_end+1):
           colorRGB = img.get_pixel(X_table[ang],Y_table[ang])
           if colorRGB[0] >= goal_threshold[0] and colorRGB[0] <= goal_threshold[1] and colorRGB[1] >= goal_threshold[2] and colorRGB[1] <= goal_threshold[3] and colorRGB[2] >= goal_threshold[4] and colorRGB[2] <= goal_threshold[5]:
               
               #if that px's color is ok, just to be sure, check the near px's color /そのピクセルの色が問題無かった場合、近くのピクセルの色も確認する
               colorRGB_a = img.get_pixel(X_table[ang + ang_margin],Y_table[ang + ang_margin])
               colorRGB_b = img.get_pixel(X_table[ang - ang_margin],Y_table[ang - ang_margin])
               if (colorRGB_a[0] >= goal_threshold[0] and colorRGB_a[0] <= goal_threshold[1] and colorRGB_a[1] >= goal_threshold[2] and colorRGB_a[1] <= goal_threshold[3] and colorRGB_a[2] >= goal_threshold[4] and colorRGB_a[2] <= goal_threshold[5])or(colorRGB_b[0] >= goal_threshold[0] and colorRGB_b[0] <= goal_threshold[1] and colorRGB_b[1] >= goal_threshold[2] and colorRGB_b[1] <= goal_threshold[3] and colorRGB_b[2] >= goal_threshold[4] and colorRGB_b[2] <= goal_threshold[5]):
                   #img.draw_circle(X_table[ang], Y_table[ang], 1,color=(0,255,0)) #//

                   if R == radius_1:
                       angle_checkerA.append(ang)
                   elif R == radius_2:
                       angle_checkerB.append(ang)
                   else:
                       angle_checkerC.append(ang)


#function to dicide next circular scanning area /次の円形ゴールスキャン範囲決定用関数
def next_ang_maker(max_ang, min_ang):
    if (angle_center - angle_param) < angle_st_const:
        angle_st = angle_st_const
    else:
        angle_st = angle_center - angle_param
    if (angle_center + angle_param) > angle_end_const:
        angle_end = angle_end_const
    else:
        angle_end = angle_center + angle_param


#function to find the goal area without enemy robot /ゴール内で敵のいない範囲を見つける関数
def enemy_checker(max_ang,min_ang,cent_ang):
    enemy_distance= [0]*split
    enemy_id1 = []
    enemy_cord = [(0,0)]*split#//
    split_ang = (max_ang - min_ang)/(split-1) 
    enemy_ang = [min_ang + 2, cent_ang - split_ang*4, cent_ang - split_ang*3, cent_ang - split_ang*2, cent_ang - split_ang, cent_ang, cent_ang + split_ang, cent_ang + split_ang*2,  cent_ang + split_ang*3, cent_ang + split_ang*4, max_ang-2]
    #split変更時enemy_ang調整必須
    shoot_ang = cent_ang

    #Scan pixels radially and judge the color /放射状にピクセルをスキャンし、色を判定する
    for i in range(split):
        for dist in range(50,120,enemy_skip):
            Rx = dist*math.cos(math.radians(enemy_ang[i])) #R cos θ
            Ry = dist*math.sin(math.radians(enemy_ang[i])) #R sin θ
            Rxin = center_x + int(Rx)
            Ryin = center_y + int(Ry)
            colorRGB = img.get_pixel(Rxin,Ryin)
            enemy_cord[i] = (Rxin,Ryin)
            if colorRGB[0] >= goal_threshold[0] and colorRGB[0] <= goal_threshold[1] and colorRGB[1] >= goal_threshold[2] and colorRGB[1] <= goal_threshold[3] and colorRGB[2] >= goal_threshold[4] and colorRGB[2] <= goal_threshold[5]:
               img.draw_circle(Rxin,Ryin,1,color=(0,255,0))
               enemy_cord[i] = (Rxin,Ryin)
               enemy_distance[i] = dist #ゴールにあたったらその距離をリストに格納
               break

    enemy_distance2 = [a for a in enemy_distance if a !=0]
    if len(enemy_distance2) == 0:
        return
    else:
        R_avg = sum(enemy_distance2)/len(enemy_distance2) #ゴールにあたるまでの平均距離計算
        R_min = min(enemy_distance2)
    for i in range(split): #ゴールまでの距離が著しく遠い（敵がいる）角度を探す
        if enemy_distance[i] > (R_avg + enemy_mg) or enemy_distance[i] == 0 or enemy_distance[i] < R_min:
            img.draw_circle(enemy_cord[i][0],enemy_cord[i][1],1,color=(255,0,0))
            enemy_id1.append(enemy_ang[i]) #敵がいると思われる角度を格納
    if  len(enemy_id1) > 0: #敵が検知できた場合（リストに要素がある）

        #judge enemy robot's left or right which area is larger / 敵ロボットの左右どちらが広いか判断する
        if (min(enemy_id1) - min_ang) >= (max_ang - max(enemy_id1)):#left is large /左側が大きい
            shoot_ang = (min(enemy_id1) + min_ang)/2 - shoot_mg
        else: #right is large /右側が広い
            shoot_ang = (max(enemy_id1) + max_ang)/2 + shoot_mg
    
    #draw line to point out where is the area without enemy robot /敵ロボットがいないゴール内の範囲を示す線を引く
    Xshot = X_table1[int(shoot_ang)]
    Yshot = Y_table1[int(shoot_ang)]
    img.draw_line(center_x,center_y,Xshot ,Yshot ,color = (255,0,0))

    #dicide where to 
    kick_ang = int((angle_mirror(shoot_ang)/360)*255)
    target_ang = kick_ang

clock = time.clock()
uart = UART(3, 115200, timeout_char = 10) # uart start
uart.init(115200, bits=8, parity=None, stop=1, timeout_char=1000)

#correct angle function /角度補正用関数
def angle_mirror(angle):
   real_ang = 360 - angle
   if real_ang >= 180:
       real_ang -= 180
   else:
       real_ang += 180
   return real_ang


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


while(True):
    clock.tick()
    img = sensor.snapshot()
    print(clock.fps())
    angle_checkerA = []#inner circle check result list /内円探知結果リスト
    angle_checkerB = []#outer circle chesk result list /外円探知結果リスト
    angle_checkerC = []#middle circle check result list /中間円探知結果リスト

    #start circular goal scan functtion / 円形ゴールスキャン関数開始
    circle_checker(radius_1,angle_st+10,angle_end+10)
    circle_checker(radius_2,angle_st,angle_end)
    circle_checker(radius_3,angle_st+5,angle_end+5)
    min_angles = [] #最小角リスト
    max_angles = [] #最大角リスト

    #in case of the goal wasn't detected /ゴールが見つからなかった場合
    if len(angle_checkerA) == 0 and len(angle_checkerB) == 0 and len(angle_checkerC) == 0:
        target_ang = 0
        kick_ang = 0
        angle_st = angle_st_const
        angle_end =angle_end_const
        uart_sender()
        continue
    
    #in case of the goal was detected /ゴールが見つかった場合
    else:

        #get the start and end goal's angles /ゴールの両端の角度・ゴールの中心の角度を導出
        for ang_array in[angle_checkerA, angle_checkerB,angle_checkerC]:
            if len(ang_array) > 0:
                min_angles.append(min(ang_array))
                max_angles.append(max(ang_array))
            else:
                min_angles.append(370)
                max_angles.append(-10) 
        min_angle = min(min_angles)
        max_angle = max(max_angles)
        angle_center = int((max_angle + min_angle)/2)

        #draw lines to point out goal detection　and goal area /ゴールエリアを示すために線を描く
        X_min = X_table1[int(min_angle)]
        Y_min = Y_table1[int(min_angle)]
        X_max = X_table1[int(max_angle)]
        Y_max = Y_table1[int(max_angle)]
        img.draw_line(center_x,center_y,X_min ,Y_min ,color = (0,255,0))
        img.draw_line(center_x,center_y,X_max ,Y_max ,color = (0,0,255))

        #find the goal area without enemy robot /ゴール内で敵のいない範囲を見つける
        enemy_checker(max_angle, min_angle,angle_center)
        uart_sender()

        #dicide next circular scanning ara /次の円形スキャン範囲を決定
        next_ang_maker(max_angle, min_angle)

    #draw center pointer /画面中心に目標を描く
    img.draw_cross(center_x, center_y,color=(0,0,255))