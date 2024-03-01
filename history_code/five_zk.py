bl_info = {
    "name": "rpu_pu",
    "author": "zzk",
    "version": (0, 0, 1),
    "blender": (2, 80, 0),
    "location": "",
    "description": "",
    "category": "机械臂",
}

import bpy  
import blf  
import bmesh
import pynput
from Robot import Robot
import time 
import serial
from serial.tools import list_ports
import threading
import math
import cv2
import numpy as np
import varmscara_fk

def draw_callback_px(self, context):
    blf.position(0, 20, 20, 0)  # 字体位置
    blf.size(0, 20)         # 字体大小
    blf.color(0,1,1,0,1)        # 字体颜色
    blf.draw(0, "       按TAB退出")  # 在当前上下文中绘制文本

button_opened,ser,cap_opened=False,None,True #串口开关
button_chessed,fivelua,button_game=False,None,False
a = 0.31  # 第一节骨骼长度
b = 0.245  # 第二节骨骼长度
check_precision = 0

ma=0
mb=0
Vector=[]
list_mn=[]
list_mn_black=[]
moved_mn=False

def max_r():
    return (a + b) - check_precision

def min_r():
    return math.fabs(a - b) + check_precision

def jn2xy(j1, j2):
    x=a*math.cos(j1)+b*math.cos(j1+j2)
    y=a*math.sin(j1)+b*math.sin(j1+j2)
    return x,y
#x,y转j1,j2
def xy2jn(x, y):
        squa_r = x ** 2 + y ** 2
        r = squa_r ** (1 / 2)
        if (r > max_r()):
            print("无法到达的位置,max")
            return None
        if (r < min_r()):
            print("无法到达的位置,min")
            return None
        cos_R = (a ** 2 + b ** 2 - squa_r) / 2 / a / b
        j2 =math.acos(cos_R)
        cos_B = (squa_r + a ** 2 - b ** 2) / 2 / a / r
        if (1 < cos_B < 1 + check_precision):
            cos_B = 1
        elif (-1 > cos_B > -1 - check_precision):
            cos_B = -1
        j1 = math.atan2(y, x) + math.acos(cos_B)
        return j1, j2
#x,y转m,n
def xy2mn(x,y): 
    Nmin,Nmax,Omin,Omax=0,18,0.06,0.51
    m = (Nmax - Nmin) /  (Omax - Omin) * (x - Omin) + Nmin
    Nmin,Nmax,Omin,Omax=0,18,-0.225,0.225
    n = (Nmax - Nmin) /  (Omax - Omin) * (y - Omin) + Nmin
    return (math.floor(m+0.5),math.floor(n+0.5))

#m,n转x,y   
def mn2xy(m,n): 
    Nmin,Nmax,Omin,Omax=0.06,0.51,0,18
    x = (Nmax - Nmin) /  (Omax - Omin) * (m - Omin) + Nmin
    Nmin,Nmax,Omin,Omax=-0.225,0.225,0,18,
    y = (Nmax - Nmin) /  (Omax - Omin) * (n - Omin) + Nmin
    return x,y

#set world coordinate
def set_location_world(object,x,y,z):
    bpy.data.objects[object].matrix_world[0][3]=x
    bpy.data.objects[object].matrix_world[1][3]=y
    bpy.data.objects[object].matrix_world[2][3]=z

# get world coordinate
def get_location_world(object):
    x=bpy.data.objects[object].matrix_world[0][3]
    y=bpy.data.objects[object].matrix_world[1][3]
    z=bpy.data.objects[object].matrix_world[2][3]
    return x,y,z
#获取机械臂角度(世界坐标系)
def robot_get_jn():
    j1=bpy.data.objects['Dof1'].rotation_euler[2] 
    j2=bpy.data.objects['Dof4'].rotation_euler[2]
    return j1,j2
#机械臂移动
def robot_move_xy(x,y,v):
    old_j1,old_j2=robot_get_jn()
    print("old_j1,old_j2:",old_j1,old_j2)
    new_j1,new_j2=xy2jn(x,y)
    print("new_j1,new_j2:",new_j1,new_j2)
    j1,j2=new_j1-old_j1,new_j2-old_j2
    return j1,j2

#复制物体
def clone(object,Vector):
    temp=xy2mn(Vector[0],Vector[2])
    if len(list_mn)==0:
        list_mn.append(temp)
        obj_copy=bpy.data.objects[object].copy()
        obj_copy.name="clone_white"+str(len(list_mn))
        bpy.context.scene.collection.objects.link(obj_copy)
        set_location_world(obj_copy.name,Vector[0],Vector[2],0.01)
    else:
        count_white=0
        count_black=0
        for mn in list_mn:
            if mn[0] !=temp[0] or mn[1] !=temp[1]:
                count_white+=1
        
        for mn in list_mn_black:
            if mn[0] !=temp[0] or mn[1] !=temp[1]:
                count_black+=1
        
        if len(list_mn)==count_white==count_black:
            list_mn.append(temp)
            obj_copy=bpy.data.objects[object].copy()
            obj_copy.name="clone_white"+str(len(list_mn))
            bpy.context.scene.collection.objects.link(obj_copy)
            set_location_world(obj_copy.name,Vector[0],Vector[2],0.01)


def choose_object(object):
    bpy.ops.object.select_all(action='DESELECT')
    bpy.context.view_layer.objects.active=None
    bpy.data.objects[object].select_set(True)
    bpy.context.view_layer.objects.active=bpy.data.objects[object]

#设置父级
def chose_parent(child,parent):
    obj_copy=bpy.data.objects[child].copy()
    obj_copy.name="clone_black"+str(len(list_mn_black))
    bpy.context.scene.collection.objects.link(obj_copy)
    x,y,z=get_location_world(obj_copy.name)
    bpy.data.objects[obj_copy.name].parent = bpy.data.objects[parent]
    bpy.ops.object.parent_no_inverse_set()
    set_location_world(obj_copy.name,x,y,z)
    return obj_copy.name

#取消父级
def cancel_parent(copy,x,y):
    bpy.data.objects[copy].parent =None
    set_location_world(copy,x,y,0.01)

#删除物体
def delete_object(object,N):
    for i in range(N):
        object1=object+str(i+1)
        print("ooobbbject",object1)
        obj = bpy.data.objects[object1]
        bpy.data.objects.remove(obj) 


class five2lua: 
    def __init__(self):
        self.list=[]
        self.count = 0
        self.fk = varmscara_fk.fk()
        self.Capture_ID = 0  # 摄像头ID
        self.start = False  # 判断游戏是否开始
        self.fkp = False  # 判断是否可以落子
        self.button_fall=0
        self.player = 0  # player = 1 表示电脑走白棋
        self.geted_mn = False
        self.u1, self.u2 = 165, 500
        self.v1, self.v2 = 150, 470

    def get_jiaoyanwei(self, msg):
        sum = 0
        jiaoyanwei = int(msg[len(msg) - 2:], 16)
        for i in range(4, len(msg) - 2, 2):
            sum += int(msg[i:i + 2], 16)
        jy = 255 - sum % 256
        if jiaoyanwei == jy:
            return True
        else:
            return False

    def set_jiaoyanwei(self, msg):
        sum = 0
        for i in range(4, len(msg) - 2, 2):
            sum += int(msg[i:i + 2], 16)
        jy = 255 - sum % 256
        return hex(jy)[2:]

    def set_msg(self, ID, matrix):  # ID:# 开始：”00“ 结束：”11“   标定：”01“  落子：”02“ 白色赢：”03“ 黑色赢：”04“
        status = "01"  # 工作
        msg = "FFFF" + ID + status + hex(matrix[0])[2:].zfill(4) + hex(matrix[1])[2:].zfill(4)
        jiaoyanwei = self.set_jiaoyanwei(msg)
        return msg + jiaoyanwei

    def finished(self, received):
        f_jyed = self.get_jiaoyanwei(received)
        if f_jyed:
            if received[len(received) - 4:len(received) - 2] == "01":
                print("标定指令,已完成发送:", received)
                return True
            elif received[len(received) - 4:len(received) - 2] == "02":
                print("落子指令,已完成发送:", received)
                self.geted_mn = True
                return True
            elif received[len(received) - 4:len(received) - 2] == "00":
                print("开始指令,已完成发送:", received)
                self.list=[]
                self.start=True
                return True
            elif received[len(received) - 4:len(received) - 2] == "11":
                print("结束指令，已完成发送:", received)
                self.start=False
                return True
    def GameStart(self,received):
        if ser.isOpen():
            jyed = self.get_jiaoyanwei(received)
            if jyed:
                msg = self.set_msg("00",(0,0))
                ser.write(bytes.fromhex(msg))
    def GameOver(self,received):
        if ser.isOpen():
            jyed = self.get_jiaoyanwei(received)
            if jyed:
                msg = self.set_msg("11",(1,1))
                ser.write(bytes.fromhex(msg))

    def biaoding(self, received):
        if ser.isOpen():
            jyed = self.get_jiaoyanwei(received)
            if jyed:
                msg = self.set_msg(self.fk, "01")
                ser.write(bytes.fromhex(msg))

    def fall(self, received):
        if ser.isOpen():
            jyed = self.get_jiaoyanwei(received)
            if jyed:
                print("按钮按下")
                # keyboard.press_and_release("enter")
                self.button_fall=1
    def read_msg(self):  # num表示等待的长度
        while True:
            count = ser.inWaiting()
            if count == 8:
                read_data = ser.read(count).decode('UTF-8', 'strict')  # 解码后为str类型
                if read_data[len(read_data) - 4:len(read_data) - 2] == "01" or read_data[len(read_data) - 4:len(
                        read_data) - 2] == "02" or read_data[len(read_data) - 4:len(
                        read_data) - 2] == "00"or read_data[len(read_data) - 4:len(
                        read_data) - 2] == "11":
                    self.finished(read_data)
                elif read_data[len(read_data) - 2:] == "01":
                    self.biaoding(read_data)
                elif read_data[len(read_data) - 2:] == "00":
                    self.GameStart(read_data)
                elif read_data[len(read_data) - 2:] == "11":
                    self.GameOver(read_data)
                elif read_data[len(read_data) - 2:] == "02":
                    self.fall(read_data)

    def output(self, board):
        for x in range(15):
            for y in range(15):
                if (board[x][y] == -1):
                    print(".", end=" ")
                else:
                    print(board[x][y], end=" ")
            print()

    def best(self, board, player):
        r = Robot(board)
        list=[]
        pos = r.haveValuePoints(player, 1 - player, board)
        print(pos)
        max_s = 0
        max_x = 0
        max_y = 0
        for (x, y, s) in pos:
            if (max_s < s):
                max_s = s
                max_x = x
                max_y = y
        for (x, y, s) in pos:
            if max_s==s :
                list.append((x,y))
        return (max_x, max_y),list

    def check_board(self, board, color, point):
        if len(board) == 0:
            return False
        if 0 <= point[0] <= 14 and 0 <= point[1] <= 14:
            if board[point[0]][point[1]] == color:
                return True
        else:
            return False

    def have_five(self, board, color):
        if len(board) == 0:
            return False
        for n in range(15):
            for m in range(15):
                if self.check_board(board, color, (m, n)) == True and self.check_board(board, color,
                                                                                       (m, n + 1)) == True and \
                        self.check_board(board, color, (m, n + 2)) == True and self.check_board(board, color,
                                                                                                (m, n + 3)) == True and \
                        self.check_board(board, color, (m, n + 4)) == True:
                    return True
                elif self.check_board(board, color, (m, n)) == True and self.check_board(board, color,
                                                                                         (m + 1, n)) == True and \
                        self.check_board(board, color, (m + 2, n)) == True and self.check_board(board, color,
                                                                                                (m + 3, n)) == True and \
                        self.check_board(board, color, (m + 4, n)) == True:
                    return True
                elif self.check_board(board, color, (m, n)) == True and self.check_board(board, color,
                                                                                         (m + 1, n + 1)) == True and \
                        self.check_board(board, color, (m + 2, n + 2)) == True and self.check_board(board, color, (
                m + 3, n + 3)) == True and \
                        self.check_board(board, color, (m + 4, n + 4)) == True:
                    return True
                elif self.check_board(board, color, (m, n)) == True and self.check_board(board, color,
                                                                                         (m + 1, n - 1)) == True and \
                        self.check_board(board, color, (m + 2, n - 2)) == True and self.check_board(board, color, (
                m + 3, n - 3)) == True and \
                        self.check_board(board, color, (m + 4, n - 4)) == True:
                    return True

    def check_win(self, board):
        if self.have_five(board, 1) == True:
            print("White Win!")
            self.start=False
            msg = self.set_msg("03", (1, 1))
            ser.write(bytes.fromhex(msg))
            return True
        elif self.have_five(board, 0) == True:
            print("Black Win!")
            self.start=False
            msg = self.set_msg("04", (0, 0))
            ser.write(bytes.fromhex(msg))
            return True
        else:
            return False

    def img2mask(self, img):
        # 转换为HSV颜色空间
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # 分割黑色区域
        # lower_black = np.array([0, 0, 0])         # low,up 之间的值变为白色
        # upper_black = np.array([200, 255, 60])  # 颜色通道：蓝色、绿色和红色
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([130, 100, 150])  # 颜色通道：蓝色、绿色和红色
        mask_black = cv2.inRange(hsv, lower_black, upper_black)

        # 分割白色区域
        # lower_white = np.array([0, 0, 211])
        # upper_white = np.array([200, 30, 255])
        lower_white = np.array([0, 0, 211])
        upper_white = np.array([200, 30, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # 将黑白两个部分合并
        mask = mask_black + mask_white
        return mask, mask_black, mask_white

    def main(self):
        global button_game
        board = []
        for x in range(15):
            b = [0] * 15
            for y in range(15):
                b[y] = -1
            board.append(b)
        cap = cv2.VideoCapture(self.Capture_ID)
        while cap_opened:
            self.button_fall = 0
            # 读入图像
            ret, img = cap.read()
            if ret and self.start:
                print("摄像头已经打开")
                point = []
                count = 0
                cimg = img.copy()
                cimg = cimg[self.v1:self.v2, self.u1:self.u2, :]  # 先v再u
                gray = cv2.cvtColor(cimg, cv2.COLOR_BGR2GRAY)
                hsv = cv2.cvtColor(cimg, cv2.COLOR_BGR2HSV)
                lower_biao = np.array([170, 100, 70])
                upper_biao = np.array([255, 255, 255])
                mask_biao = cv2.inRange(hsv, lower_biao, upper_biao)
                # 检测图像中的圆形
                circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1, minDist=300, param1=20, param2=8,minRadius=4,maxRadius=7)
                # 绘制检测到的圆形
                if circles is not None:
                    circles = np.round(circles[0, :]).astype("int")
                    for (x, y, r) in circles:
                        xy = "%d,%d" % (x, y)
                        cv2.circle(cimg, (x, y), r, (0, 0, 255), 2)  # 红色
                        cv2.circle(img, (x+self.u1, y+self.v1), r, (0, 0, 255), 2)  # 红色
                        print((x, y),r)
                        cv2.putText(cimg, xy, (x, y), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 0), thickness=1)
                        cv2.putText(img, xy, (x+self.u1, y+self.v1), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 0), thickness=1)
                        # if len(circles) == 3 and self.fkp==False and (y<20 or (y>300 and x<12)):
                        #     point.insert(count, (x, y))
                        #     count += 1
                                # fk.point.insert(count, (x, y))
                            # if count == 3:
                            #     # count = 0
                            #     sorted_list = sorted(point, key=lambda x: sum(x))  #
                            #     self.fk.point[0] = sorted_list[0]
                            #     sorted_list = sorted(point, key=lambda x: x[1])  # v号点
                            #
                            #     self.fk.point[1] = sorted_list[2]
                            #     sorted_list = sorted(point, key=lambda x: x[0])  # u号点
                            #     self.fk.point[2] = sorted_list[2]
                            #     print("fk.point", self.fk.point)
                        # self.fkp = True
                if self.fkp:
                    for n in range(15):
                        for m in range(15):
                            board[m][n] = -1
                    mask, mask_black, mask_white = self.img2mask(img)

                    mask = mask[self.v1:self.v2, self.u1:self.u2]
                    mask_black = mask_black[self.v1:self.v2, self.u1:self.u2]
                    mask_white = mask_white[self.v1:self.v2, self.u1:self.u2]
                    mask_black = cv2.medianBlur(mask_black, 1)
                    # mask_white = cv2.medianBlur(mask_white, 5)

                    circles_black = cv2.HoughCircles(mask_black, cv2.HOUGH_GRADIENT, dp=1, minDist=12, param1=20,param2=6,minRadius=7, maxRadius=8)
                    circles_white = cv2.HoughCircles(mask_white, cv2.HOUGH_GRADIENT, dp=1, minDist=12, param1=20,param2=9,minRadius=7, maxRadius=9)
                    # if circles_black is not None:
                    #     circles_black = np.round(circles_black[0, :]).astype("int")
                    #     for (x, y, r) in circles_black:
                    #         cv2.circle(cimg, (x, y), r, (0, 255, 0), 2)  # 绿色
                    #         m, n = self.fk.interp_uv2mn(x, y)
                    #         print("黑色:", m, n)
                    #         board[m][n] = 0  # 黑色
                    if circles_white is not None:
                        circles_white = np.round(circles_white[0, :]).astype("int")
                        for (x, y, r) in circles_white:
                            cv2.circle(img, (x+self.u1, y+self.v1), r, (255, 0, 0), 2)
                            m, n = self.fk.interp_uv2mn(x, y)
                            print("白色:", m, n)
                            board[m][n] = 1  # 白色
                    for (m, n) in self.list:
                        board[m][n] = 0     # 黑色
                    cv2.imshow('mask_white', mask_white)
                    cv2.imshow('mask_black', mask_black)
                    cv2.imshow('mask', mask)
                    if self.check_win(board) == False:
                        (max_x, max_y) ,list= self.best(board, self.player)  # 行列
                        if len(list)<2:
                            print("你该下的位置:", (max_x, max_y))
                            msg = self.set_msg("02", (max_x, max_y))
                            print("msg", msg)
                            ser.write(bytes.fromhex(msg))
                            self.geted_mn = False
                            print("已发送lua位置")
                            self.list.append((max_x, max_y))
                            board[max_x][max_y] = "*"  # 机械臂下的位置
                        else:
                            temp=0
                            for (a,b) in list:
                                for (a1,b1)in self.list:
                                    if (a,b)!=(a1,b1):
                                        temp+=1
                                if temp==len(self.list):
                                    print("best有重复:", (a,b))
                                    print((a, b), "重复了")
                                    msg = self.set_msg("02", (a,b))
                                    print("msg", msg)
                                    ser.write(bytes.fromhex(msg))
                                    self.geted_mn = False
                                    print("已发送lua位置",(a,b))
                                    self.list.append((a,b))
                                    board[a][b] = "*"   #机械臂下的位置
                                    break
                                temp=0
                        self.output(board)
                cv2.imshow('cimg', cimg)
                cv2.imshow('img', img)
                # while  self.button_fall==0 and self.fkp:
                #     if self.button_fall or self.start==False:
                #         break
            if self.fkp:
                k = cv2.waitKey(1) & 0xFF
            else:
                k = cv2.waitKey(0) & 0xFF
            if k == 27:
                cv2.destroyAllWindows()
                self.start=False
                button_game=False

def arms_move(s=0.027,N=30):
    k=s/N
    for i in range(N):
        bpy.data.objects['Dof7'].location[2] +=k
        bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
    print("上升")
    return None 
def arms_up(s=0.0235,N=20):
    k=s/N
    for i in range(N):
        bpy.data.objects['Dof7'].location[2] +=k
        bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
    print("上升")
    return None 
def arms_down(s=0.0235,N=20):
    k=s/N
    for i in range(N):
        bpy.data.objects['Dof7'].location[2] -=k
        bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
    print("下降")
    return None 

def robot_process(x,y): #流程
    xp,yp=0,0.066
    arms_down()
    copy=chose_parent("Chess_black","4280_吸盘")
    arms_up()
    j1,j2=robot_move_xy(x,y,40)
    arms_down()
    cancel_parent(copy,x,y)
    arms_up()
    jxp,jyp=robot_move_xy(xp,yp,40)
    return j1,j2,jxp,jyp

def best(board, player):# player = 1 表示电脑走白棋
    r = Robot(board)
    pos = r.haveValuePoints(player, 1 - player, board)
    print(pos)
    max_s = 0
    max_x = 0
    max_y = 0
    for (x, y, s) in pos:
        if (max_s < s):
            max_s = s
            max_x = x
            max_y = y
    return (max_x, max_y)
def Game_start():
    global button_game,ma,mb,Vector,list_mn,moved_mn,list_mn_black
    button_game=True
    delete_object("clone_black",len(list_mn_black))
    delete_object("clone_white",len(list_mn))
    ma=0
    mb=0
    Vector=[]
    list_mn=[]  
    list_mn_black=[]
    moved_mn=False
def Game_over():
    global button_game,ma,mb,Vector,list_mn,moved_mn,list_mn_black
    button_game=False
def check_win(board):
    global button_game
    five=five2lua()
    if five.have_five(board, 1) == True:
        print("White Win!")
        Game_over()
        return True
    elif five.have_five(board, 0) == True:
        print("Black Win!")
        Game_over()
        return True
    else:
        return False
def move_mn():
    board = []
    for x in range(15):
        b = [0] * 15
        for y in range(15):
            b[y] = -1
        board.append(b)
    for (m,n) in list_mn:
        board[m][n] = 1  # 白色
    for (m,n) in list_mn_black:
        board[m][n] = 0  # 黑色
    if check_win(board)==False and len(list_mn)>len(list_mn_black):
        m,n=best(board,0)
        list_mn_black.append((m,n))
        board[m][n] = 0  # 黑色
        print("m,n:",m,n)
        x,y=mn2xy(m,n)
        j1,j2,jxp,jyp=robot_process(x,y)
        check_win(board)
        return j1,j2,jxp,jyp




def hh():
    global Vector,old_co
    print("hhh")
    Vector=[]
    obj=bpy.data.objects["Chessboard"]
    if obj.mode == 'EDIT':
        bm=bmesh.from_edit_mesh(obj.data)
        for v in bm.verts:
            if v.select:
                    xp,yp,zp=get_location_world("Chess_piece")
                    xb,yb,zb=get_location_world("Chessboard")
                    tx,ty=xb-xp,yb-yp
                    for i in range (3):
                        Vector.append(v.co[i])
                    Vector[0]=Vector[0]+tx+xp
                    Vector[2]=ty-Vector[2]+yp
                    print("Vector",Vector)
def on_click(x, y, button, pressed):
    global ma,mb
    print('{0} at {1}'.format(
        'Pressed' if pressed else 'Released',
        (x, y)))
    if not pressed:
        mb+=1
        if mb==2:
            mb=0
            # Stop listener
            ma=0
            hh()
            return False
    if button_game==False:
        return False
def on_mouse_click():
    global ma
    ma=1
    with pynput.mouse.Listener(
            on_click=on_click,) as listener:
        listener.join()  
count=0
def mouse_control():
    global Vector,count,moved_mn
    if ma==0:
        thread1 = threading.Thread(target=on_mouse_click)  
        thread1.start()
    if button_game==False:
        return None # 返回 None 给计时器，计时器会停止
    return 0.00001
def move_control():
    if moved_mn:
        move_mn()
    if button_game==False:
        return None # 返回 None 给计时器，计时器会停止
    return 0.0001




# 端口
coms=[]

# 波特率
bit_rate=[
    ("4800","4800","","icon",1),
    ("9600","9600","","icon",2),
    ("38400","38400","","icon",3),
    ("115200","115200","","icon",4),
]

# 指令集
instructions=[
    ("FF FF 01 FE"),
    ("FF FF 02 FD"),
]




def rad1(self):
    global coms
    coms=[]
    coms1 = serial.tools.list_ports.comports()
    num = len(coms1)
    if num>0:
        for i in range(num):
            items = list(coms1[i])
            # print("hhhhhhhhhhhhhhhhh",items[0])
            coms.append((items[0],items[1],"","icon",i+1)) 
    bpy.types.Scene.Enum=bpy.props.EnumProperty(items=coms)

    # # 激活物体
    # choose_objects=[]
    # count=0
    # bpy.context.view_layer.objects.active=bpy.data.objects["Dof7"]
    # for i in bpy.context.view_layer.objects:
    #     choose_objects.append((i.name,i.name,"","icon",count+1)) 
    #     count+=1
    # bpy.types.Scene.Enum2=bpy.props.EnumProperty(items=choose_objects)

    return bpy.data.objects["Dof1"].rotation_euler[2]
def rad2(self):
    return bpy.data.objects["Dof4"].rotation_euler[2]
def angle1(self):
    return bpy.data.objects["Dof1"].rotation_euler[2]/math.pi*180
def angle2(self):
    return bpy.data.objects["Dof4"].rotation_euler[2]/math.pi*180
def x(self):
    return bpy.data.objects["Dof7"].matrix_world[0][3]
def y(self):
    return bpy.data.objects["Dof7"].matrix_world[1][3]
def Dof1(self,context):
    scene=context.scene
    bpy.data.objects["Dof1"].rotation_euler[1]=scene.Dof1/180*math.pi
def Dof2(self,context):
    scene=context.scene
    bpy.data.objects["Dof2"].rotation_euler[1]=scene.Dof2/180*math.pi
def Dof3(self,context):
    scene=context.scene
    bpy.data.objects["Dof3"].rotation_euler[1]=scene.Dof3/180*math.pi
def Dof4(self,context):
    scene=context.scene
    bpy.data.objects["Dof4"].rotation_euler[1]=scene.Dof4/180*math.pi
def Dof5(self,context):
    scene=context.scene
    bpy.data.objects["Dof5"].rotation_euler[1]=scene.Dof5/180*math.pi
def Dof6(self,context):
    scene=context.scene
    bpy.data.objects["Dof6"].rotation_euler[1]=scene.Dof6/180*math.pi
def Dof7(self,context):
    scene=context.scene
    bpy.data.objects["Dof7"].rotation_euler[1]=scene.Dof7/180*math.pi

def myproperty():   # 属性创建
    # 枚举类型:下拉菜单
    bpy.types.Scene.Enum=bpy.props.EnumProperty(items=coms)
    bpy.types.Scene.Enum1=bpy.props.EnumProperty(items=bit_rate)
    #浮点类型
    bpy.types.Scene.rad1=bpy.props.FloatProperty(name="rad1",get=rad1)
    bpy.types.Scene.rad2=bpy.props.FloatProperty(name="rad2",get=rad2)
    bpy.types.Scene.angle1=bpy.props.FloatProperty(name="angle1",get=angle1)
    bpy.types.Scene.angle2=bpy.props.FloatProperty(name="angle2",get=angle2)
    bpy.types.Scene.x=bpy.props.FloatProperty(name="x",get=x)
    bpy.types.Scene.y=bpy.props.FloatProperty(name="y",get=y)
    bpy.types.Scene.Dof1=bpy.props.FloatProperty(name="Dof1",min=-180,max=180,step=10,update=Dof1)
    bpy.types.Scene.Dof2=bpy.props.FloatProperty(name="Dof2",min=-180,max=180,step=10,update=Dof2)
    bpy.types.Scene.Dof3=bpy.props.FloatProperty(name="Dof3",min=-180,max=180,step=10,update=Dof3)
    bpy.types.Scene.Dof4=bpy.props.FloatProperty(name="Dof4",min=-180,max=180,step=10,update=Dof4)
    bpy.types.Scene.Dof5=bpy.props.FloatProperty(name="Dof5",min=-180,max=180,step=10,update=Dof5)
    bpy.types.Scene.Dof6=bpy.props.FloatProperty(name="Dof6",min=-180,max=180,step=10,update=Dof6)
    bpy.types.Scene.Dof7=bpy.props.FloatProperty(name="Dof7",min=-180,max=180,step=10,update=Dof7)


test_button=True
def test():
    global fivelua
    fivelua = five2lua()
    fivelua.main()



class eyes_control(bpy.types.Operator):
    bl_idname="button.eyes"
    bl_label="eyes_control"  #别名

    # ---------------------------------------初始化-------------------------------------------------------
    
    _timer = None       # 用于承载计时器
    info = None         # 用于承载文字绘制
    j1,j2=0.3,0.3
    jxp,jyp=None,None
    v=30
    count=0
    moved=0
    # ---------------------------------------模态函数-------------------------------------------------------
    # 模态函数
    def modal(self, context, event):       
        global Vector
        # 按ESC键退出运算符
        if event.type =='ESC':
            self.cancel(context) # 运行"退出"函数
                        
            # 更新3d视图(如果有)
            for area in bpy.context.screen.areas:
                area.tag_redraw()
            return {'CANCELLED'} # 退出模态运算符
        
        if event.type == 'TIMER': # 如果是计时器事件，即操作物体
            if bpy.data.objects["Chessboard"].mode=='EDIT' and self.moved==1:
                    self.count+=1
                    bpy.data.objects['Dof1'].rotation_euler[2] +=self.j1/self.v
                    bpy.data.objects['Dof4'].rotation_euler[2] +=self.j2/self.v
                    if self.v==self.count:
                        self.count=0
                        self.moved=0

        if bpy.data.objects["Chessboard"].mode=='EDIT' and len(Vector)==3:
            bpy.ops.object.editmode_toggle()
            choose_object("Chess_piece")
            clone("Chess_piece",Vector)
            choose_object("Chessboard") 
            bpy.ops.object.editmode_toggle()
            Vector=[]
            self.j1,self.j2,self.jxp,self.jyp=move_mn()
            self.moved=1
            return {'RUNNING_MODAL'}

        return {'PASS_THROUGH'} # 如果没有做事情，则绕过模态(这样在除了上面按键外，保留blender自身快捷键)
    
    def execute(self, context):
        if button_game:
            print("Game Over!")
            if fivelua:
                fivelua.start=False
            Game_over()
            return {"FINISHED"}
        else:
            if fivelua:
                fivelua.start=True
            Game_start()
            choose_object("Chessboard")
            bpy.ops.object.editmode_toggle()
            bpy.app.timers.register(mouse_control) 
            self.info = bpy.types.SpaceView3D.draw_handler_add(draw_callback_px, (None, None), 'WINDOW', 'POST_PIXEL') # 将文本绘制添加到窗口中
            self._timer = context.window_manager.event_timer_add(1/24, window=context.window) # 将计时器添加到窗口中
            context.window_manager.modal_handler_add(self) # 将模态处理函数添加到窗口中
            return {'RUNNING_MODAL'} # 返回到模态，保持运算符在blender上一直处于运行状态
        # "退出"函数(命令取消时)
    def cancel(self, context):
        bpy.types.SpaceView3D.draw_handler_remove(self.info,'WINDOW') # 将模态处理函数在窗口管理器中移除
        context.window_manager.event_timer_remove(self._timer) # 将给定的窗口中的计时器移除
class opened_control(bpy.types.Operator):
    bl_idname="button.opened"
    bl_label="opened_control"  #别名
    bl_options={"REGISTER","UNDO"}

    def execute(self,context): #执行函数
        global button_opened,ser,cap_opened
        scene=context.scene
        if button_opened:
            ser.close()
            print(scene.Enum,",摄像头","已经关闭")
            button_opened=False
            cap_opened=False
        else:
            ser = serial.Serial(scene.Enum,scene.Enum1)
            ser.write(bytes.fromhex("FF FF 01 00"))
            print(scene.Enum,"已经打开")
            button_opened,cap_opened=True,True
            thread1 = threading.Thread(target=test)  
            thread1.start()
        return {"FINISHED"}

class moveed_control(bpy.types.Operator):
    bl_idname="button.moveed"
    bl_label="moveed_control"  #别名
    bl_options={"REGISTER","UNDO"}

    def execute(self,context): #执行函数
        bpy.app.timers.register(arms_move) 
        return {"FINISHED"}

class uped_control(bpy.types.Operator):
    bl_idname="button.uped"
    bl_label="uped_control"  #别名
    bl_options={"REGISTER","UNDO"}

    def execute(self,context): #执行函数
        bpy.app.timers.register(arms_up) 
        return {"FINISHED"}

class downed_control(bpy.types.Operator):
    bl_idname="button.downed"
    bl_label="downed_control"  #别名
    bl_options={"REGISTER","UNDO"}

    def execute(self,context): #执行函数
        bpy.app.timers.register(arms_down) 
        return {"FINISHED"}
    
class chessed_control(bpy.types.Operator):
    bl_idname="button.chessed"
    bl_label="chessed_control"  #别名
    bl_options={"REGISTER","UNDO"}

    def execute(self,context): #执行函数
        global button_chessed
        scene=context.scene
        if button_chessed:
            print("吸")
            button_chessed=False
        else:
            print("放")
            button_chessed=True
        return {"FINISHED"}

# ui_type;固定各式
class ui_type:
    bl_space_type,bl_region_type,bl_context="VIEW_3D","UI","objectmode"     

# 面板1
class rpu_config(ui_type,bpy.types.Panel):  
    bl_idname="rpu_config"
    bl_label="机械臂配置"                                                   
    bl_category="机械臂"                                                

    def draw(self,context):
        layout=self.layout
        scene=context.scene
        layout.label(text="configure",icon="BLENDER")
        row3=layout.row()
        row4=layout.row()
        row5=layout.row()
        row6=layout.row()
        row7=layout.row()
        row8=layout.row()
        row9=layout.row()
        row10=layout.row()


        row3.prop(scene,"Enum",text="端口号")
        row3.prop(scene,"Enum1",text="波特率")
        row4.prop(scene,"Dof1",text="Dof1")
        row5.prop(scene,"Dof2",text="Dof2")
        row6.prop(scene,"Dof3",text="Dof3")
        row7.prop(scene,"Dof4",text="Dof4")
        row8.prop(scene,"Dof5",text="Dof5")
        row9.prop(scene,"Dof6",text="Dof6")
        row10.prop(scene,"Dof7",text="Dof7")

        #生成按钮
        if button_opened:
            row3.operator("button.opened",text="关闭",icon="CUBE",depress=True)
        else:
            row3.operator("button.opened",text="打开",icon="CUBE")


# 面板2
class rpu_arms(ui_type,bpy.types.Panel):  
    bl_idname="rpu_arms"
    bl_label="机械臂控制"                                                    #  显示的标签名字
    bl_category="机械臂"                                                     #  标签分类
    
    def draw(self,context):
        layout=self.layout
        scene=context.scene
        layout.label(text="status",icon="BLENDER")
        row1=layout.row()
        row2=layout.row()
        layout.label(text="control",icon="BLENDER")
        row1.prop(scene,"rad1",text="rad1:")
        row1.prop(scene,"angle1",text="angle1:")
        row1.prop(scene,"x",text="x:")
        row2.prop(scene,"rad2",text="rad2:")
        row2.prop(scene,"angle2",text="angle2:")
        row2.prop(scene,"y",text="y:")
        row3=layout.row()
        row3.operator("button.uped",text="上升",icon="CUBE")
        row3.operator("button.downed",text="下降",icon="CUBE")
        
# 面板3
class rpu_eyes(ui_type,bpy.types.Panel):  
    bl_idname="rpu_eyes"
    bl_label="机械臂视觉"                                                    #  显示的标签名字
    bl_category="机械臂"                                                     #  标签分类
    
    def draw(self,context):
        layout=self.layout
        layout.label(text="摄像头",icon="BLENDER")
        row1=layout.row()
        row1.operator("button.eyes",text="开始游戏",icon="CUBE",depress=button_game)






classes=[
    rpu_config,
    rpu_arms,
    rpu_eyes,
    opened_control,
    uped_control,
    downed_control,
    eyes_control,
]


def register():
    # 属性的创建
    myproperty()
    for item in classes:
        bpy.utils.register_class(item)
    
def unregister():
    global button_game
    for item in classes:
        bpy.utils.unregister_class(item)
    button_game=False


if __name__ == "__main__":
    register()