import cv2
import mediapipe as mp
import pyautogui #thu vien ho tro tac vu cua chuot
import math
from enum import IntEnum
from ctypes import cast, POINTER
from comtypes import CLSCTX_ALL
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume
from google.protobuf.json_format import MessageToDict
import screen_brightness_control as sbcontrol

pyautogui.FAILSAFE = False #dong chuong trinh neu spam chuot
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

# lớp các cử chỉ
class Gest(IntEnum):
    #Ma nhi phan cu chi tay
    FIST = 0 #nam tay lai
    PINKY = 1 #ngon ut
    RING = 2 #ngon ap ut
    MID = 4 #ngon giua
    LAST3 = 7 #gio 3 ngon giua, tro,cai
    INDEX = 8 #ngon tro
    FIRST2 = 12 #hai ngon giua + tro
    LAST4 = 15 #gio 4 ngon tru ngon cai
    THUMB = 16 #ngon cai
    PALM = 31 #gio ca ban tay

    # Cac ma cu chi mo rong
    V_GEST = 33 #hinh chu V
    TWO_FINGER_CLOSED = 34  #2 ngon tay khep lai
    PINCH_MAJOR = 35 #kẹp ngón trỏ và ngón cái tay phải
    PINCH_MINOR = 36 #kẹp ngón trỏ và ngón cái tay trái


# Multi-handedness Labels
class HLabel(IntEnum):
    MINOR = 0 #nhãn phụ
    MAJOR = 1 #nhãn chính


# chuyển đổi các đốt ngón tay thành các cử chỉ dễ nhận biết
#nhận biết cử chỉ tay
class HandRecog:

    def __init__(self, hand_label):
        self.finger = 0
        self.ori_gesture = Gest.PALM
        self.prev_gesture = Gest.PALM
        self.frame_count = 0
        self.hand_result = None
        self.hand_label = hand_label

    def update_hand_result(self, hand_result):
        self.hand_result = hand_result

    # tinh khoang cach giua cac diem dot ngon tay neu diem 0 thap hon diem 1
    def get_signed_dist(self, point):
        sign = -1
        if self.hand_result.landmark[point[0]].y < self.hand_result.landmark[point[1]].y:
            sign = 1
        dist = (self.hand_result.landmark[point[0]].x - self.hand_result.landmark[point[1]].x) ** 2
        dist += (self.hand_result.landmark[point[0]].y - self.hand_result.landmark[point[1]].y) ** 2
        dist = math.sqrt(dist)
        return dist * sign

    #tinh khoan cach cac diem dot ngon tay
    def get_dist(self, point):
        dist = (self.hand_result.landmark[point[0]].x - self.hand_result.landmark[point[1]].x) ** 2
        dist += (self.hand_result.landmark[point[0]].y - self.hand_result.landmark[point[1]].y) ** 2
        dist = math.sqrt(dist)
        return dist

    #tinh khoan cach theo toa do z (do sau)
    def get_dz(self, point):
        return abs(self.hand_result.landmark[point[0]].z - self.hand_result.landmark[point[1]].z)


    # hàm tìm mã hóa cử chỉ sử dụng trạng thái hiện tại của ngón tay
    # Finger_state: 1 if finger is open, else 0
    def set_finger_state(self):
        if self.hand_result == None:
            return

        #diem tai cac ngon tro, ngon giua, ngon ap ut và ngon ut
        points = [[8, 5, 0], [12, 9, 0], [16, 13, 0], [20, 17, 0]]
        self.finger = 0
        self.finger = self.finger | 0  # thumb
        for idx, point in enumerate(points):

            dist = self.get_signed_dist(point[:2]) #cat lay hai phan tu dau ,khoang cach tu dau ngon tay den cuoi ngon tay
            dist2 = self.get_signed_dist(point[1:]) #cat lay tu phan tu thu 1 den het,khoan cach tu cuoi ngon den diem 0

            try:
                ratio = round(dist / dist2, 1) # ti le kc = do dai ngon tay/do dai long ban tay
            except:
                ratio = round(dist1 / 0.01, 1)

            self.finger = self.finger << 1 #dich 1 bit sang trai van bang 0
            if ratio > 0.5:
                self.finger = self.finger | 1 #neu ti le khoang cach lon hon 0.5 thi self.finger = 1 => ngon tay chua duoc gap lai


    # Xu ly cac dao dong do nhieu hinh anh

    #ham nhan dang cac cu chi
    def get_gesture(self):
        if self.hand_result == None:
            return Gest.PALM  #gest = 31 => gio 5 ngon tay

        current_gesture = Gest.PALM #gio nam ngon tay de nhan dang
        if self.finger in [Gest.LAST3, Gest.LAST4] and self.get_dist([8, 4]) < 0.05: #tro vai cai cham nhau
            if self.hand_label == HLabel.MINOR:
                current_gesture = Gest.PINCH_MINOR #tay trai
            else:
                current_gesture = Gest.PINCH_MAJOR #tay phai

        elif Gest.FIRST2 == self.finger: #giua + tro
            point = [[8, 12], [5, 9]]  #dau va cuoi hai ngon giua va tro
            dist1 = self.get_dist(point[0])
            dist2 = self.get_dist(point[1])
            ratio = dist1 / dist2
            if ratio > 1.7: #ti le khoang cach hai dau ngon tay/cuoi ngon tay >1.7
                current_gesture = Gest.V_GEST
            else:
                if self.get_dz([8, 12]) < 0.1: #tinh theo z hai dau ngon tay <0.1
                    current_gesture = Gest.TWO_FINGER_CLOSED
                else:
                    current_gesture = Gest.MID

        else:
            current_gesture = self.finger

        if current_gesture == self.prev_gesture:
            self.frame_count += 1
        else:
            self.frame_count = 0

        self.prev_gesture = current_gesture

        # neu khong thuc hien cac cu chi khac
        if self.frame_count > 4:
            self.ori_gesture = current_gesture
        return self.ori_gesture  #trang thai cu chi tay la gio nam ngon tay de dung


# Thực hiện các lệnh theo các cử chỉ được phát hiện
class Controller:
    tx_old = 0
    ty_old = 0
    trial = True
    flag = False
    grabflag = False #nam
    pinchmajorflag = False
    pinchminorflag = False
    pinchstartxcoord = None #pinch theo chieu x
    pinchstartycoord = None #pinch theo chieu y
    pinchdirectionflag = None
    prevpinchlv = 0
    pinchlv = 0
    framecount = 0
    prev_hand = None
    pinch_threshold = 0.3 #ngưỡng pinch

    #lay muc pinch theo toa do y
    def getpinchylv(hand_result):
        dist = round((Controller.pinchstartycoord - hand_result.landmark[8].y) * 10, 1) #khoang cach tu pich bat dau den 8(y)
        return dist

    #lay muc pinch theo toa do x
    def getpinchxlv(hand_result):
        dist = round((hand_result.landmark[8].x - Controller.pinchstartxcoord) * 10, 1) #khoang cach tu pinch bat dau den 8(x)
        return dist

    #thay doi do sang man hinh
    def changesystembrightness():
        currentBrightnessLv = sbcontrol.get_brightness() / 100.0 #do sang man hinh hien tai
        currentBrightnessLv += Controller.pinchlv / 50.0 #
        if currentBrightnessLv > 1.0:
            currentBrightnessLv = 1.0
        elif currentBrightnessLv < 0.0:
            currentBrightnessLv = 0.0
        sbcontrol.fade_brightness(int(100 * currentBrightnessLv), start=sbcontrol.get_brightness()) #giam do sang man hinh

    #thay doi am luong
    def changesystemvolume():
        devices = AudioUtilities.GetSpeakers()
        interface = devices.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
        volume = cast(interface, POINTER(IAudioEndpointVolume))
        currentVolumeLv = volume.GetMasterVolumeLevelScalar()
        currentVolumeLv += Controller.pinchlv / 50.0
        if currentVolumeLv > 1.0:
            currentVolumeLv = 1.0
        elif currentVolumeLv < 0.0:
            currentVolumeLv = 0.0
        volume.SetMasterVolumeLevelScalar(currentVolumeLv, None)

    #cuon chuot theo chieu doc
    def scrollVertical():
        pyautogui.scroll(120 if Controller.pinchlv > 0.0 else -120)

    #cuon chuot chieu ngang
    def scrollHorizontal():
        pyautogui.keyDown('shift')
        pyautogui.keyDown('ctrl')
        pyautogui.scroll(-120 if Controller.pinchlv > 0.0 else 120)
        pyautogui.keyUp('ctrl')
        pyautogui.keyUp('shift')

    # Xác định vị trí tay để có được vị trí con trỏ
    # Ôn định con trỏ chuột
    #tim vi tri con tro chuot tren man hinh
    def get_position(hand_result):
        point = 9 #cuoi ngon tay tro
        position = [hand_result.landmark[point].x, hand_result.landmark[point].y]
        sx, sy = pyautogui.size() #tim chieu cao va chieu rong cua man hinh
        x_old, y_old = pyautogui.position() #tra ve vi tri x, y cua con tro chuot
        x = int(position[0] * sx)
        y = int(position[1] * sy)
        if Controller.prev_hand is None: #neu truoc do khong lam gi ca
            Controller.prev_hand = x, y
        delta_x = x - Controller.prev_hand[0]
        delta_y = y - Controller.prev_hand[1]

        distsq = delta_x ** 2 + delta_y ** 2
        ratio = 1
        Controller.prev_hand = [x, y]

        if distsq <= 25:
            ratio = 0
        elif distsq <= 900:
            ratio = 0.07 * (distsq ** (1 / 2))
        else:
            ratio = 2.1
        x, y = x_old + delta_x * ratio, y_old + delta_y * ratio
        return (x, y)

    #ham khoi tao cac bien nheo ngon tro va cai
    def pinch_control_init(hand_result):
        Controller.pinchstartxcoord = hand_result.landmark[8].x #pinch theo chieu x bat dau tinh toa do tai x moc 8 dau tien
        Controller.pinchstartycoord = hand_result.landmark[8].y #pinch theo chieu y bat dau tinh tu toa do y moc 8 dau tien
        Controller.pinchlv = 0
        Controller.prevpinchlv = 0
        Controller.framecount = 0

    #giữ vị trí cuối cùng trong 5 khung hình để thay đổi trạng thái
    #dieu khien theo cu chi pinch
    def pinch_control(hand_result, controlHorizontal, controlVertical):
        if Controller.framecount == 5:
            Controller.framecount = 0
            Controller.pinchlv = Controller.prevpinchlv

            if Controller.pinchdirectionflag == True:
                controlHorizontal()  # x

            elif Controller.pinchdirectionflag == False:
                controlVertical()  # y

        lvx = Controller.getpinchxlv(hand_result)
        lvy = Controller.getpinchylv(hand_result)

        if abs(lvy) > abs(lvx) and abs(lvy) > Controller.pinch_threshold:
            Controller.pinchdirectionflag = False
            if abs(Controller.prevpinchlv - lvy) < Controller.pinch_threshold:
                Controller.framecount += 1
            else:
                Controller.prevpinchlv = lvy
                Controller.framecount = 0

        elif abs(lvx) > Controller.pinch_threshold:
            Controller.pinchdirectionflag = True
            if abs(Controller.prevpinchlv - lvx) < Controller.pinch_threshold:
                Controller.framecount += 1
            else:
                Controller.prevpinchlv = lvx
                Controller.framecount = 0

    def handle_controls(gesture, hand_result):
        x, y = None, None
        if gesture != Gest.PALM:
            x, y = Controller.get_position(hand_result)

        #đặt lại cờ
        if gesture != Gest.FIST and Controller.grabflag: #neu dang khong nam tay lai
            Controller.grabflag = False
            pyautogui.mouseUp(button="left") #chuot trai len

        if gesture != Gest.PINCH_MAJOR and Controller.pinchmajorflag: #neu khong phai pinch tay phai
            Controller.pinchmajorflag = False

        if gesture != Gest.PINCH_MINOR and Controller.pinchminorflag: #neu khong phai  pinch tay trai
            Controller.pinchminorflag = False

        if gesture == Gest.V_GEST: #hinh chu V
            Controller.flag = True
            pyautogui.moveTo(x, y, duration=0.1)

        elif gesture == Gest.FIST: #nam tay lai khong lam gi
            if not Controller.grabflag:
                Controller.grabflag = True
                pyautogui.mouseDown(button="left")
            pyautogui.moveTo(x, y, duration=0.1)

        elif gesture == Gest.MID and Controller.flag:
            pyautogui.click()
            Controller.flag = False

        elif gesture == Gest.INDEX and Controller.flag:
            pyautogui.click(button='right')
            Controller.flag = False

        elif gesture == Gest.TWO_FINGER_CLOSED and Controller.flag:
            pyautogui.doubleClick()
            Controller.flag = False

        elif gesture == Gest.PINCH_MINOR:
            if Controller.pinchminorflag == False:
                Controller.pinch_control_init(hand_result)
                Controller.pinchminorflag = True
            Controller.pinch_control(hand_result, Controller.scrollHorizontal, Controller.scrollVertical)

        elif gesture == Gest.PINCH_MAJOR:
            if Controller.pinchmajorflag == False:
                Controller.pinch_control_init(hand_result)
                Controller.pinchmajorflag = True
            Controller.pinch_control(hand_result, Controller.changesystembrightness, Controller.changesystemvolume)


'''
----------------------------------------  Main Class  ----------------------------------------
    Entry point of Gesture Controller
'''


class GestureController:
    gc_mode = 0
    cap = None
    CAM_HEIGHT = None
    CAM_WIDTH = None
    hr_major = None  # Right Hand by default
    hr_minor = None  # Left hand by default
    dom_hand = True

    def __init__(self):
        GestureController.gc_mode = 1
        GestureController.cap = cv2.VideoCapture(0)
        GestureController.CAM_HEIGHT = GestureController.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        GestureController.CAM_WIDTH = GestureController.cap.get(cv2.CAP_PROP_FRAME_WIDTH)

    #phan loai ban tay
    def classify_hands(results):
        left, right = None, None
        try:
            handedness_dict = MessageToDict(results.multi_handedness[0])
            if handedness_dict['classification'][0]['label'] == 'Right':
                right = results.multi_hand_landmarks[0]
            else:
                left = results.multi_hand_landmarks[0]
        except:
            pass

        try:
            handedness_dict = MessageToDict(results.multi_handedness[1])
            if handedness_dict['classification'][0]['label'] == 'Right':
                right = results.multi_hand_landmarks[1]
            else:
                left = results.multi_hand_landmarks[1]
        except:
            pass

        if GestureController.dom_hand == True:
            GestureController.hr_major = right
            GestureController.hr_minor = left
        else:
            GestureController.hr_major = left
            GestureController.hr_minor = right

    def start(self):

        handmajor = HandRecog(HLabel.MAJOR)
        handminor = HandRecog(HLabel.MINOR)

        #goi lớp Hands của mediapipe với các điều kiện tương ứng
        # dieu kien with goi lop Hand mediapipe voi cac tham so
        # toi da 2 tay, do tin cay phat hien toi thieu la 0.5, do tin cay theo doi toi thieu la 0.5
        with mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
            #chạy vong lap vo tan cho tan khi nhan q de dung
            while GestureController.cap.isOpened() and GestureController.gc_mode:
                success, image = GestureController.cap.read() #du lieu tu webcam duoc luu tru trong lop read

                if not success:
                    print("Ignoring empty camera frame.")
                    continue
                #thay doi dinh dang cua hinh anh truoc khi xu ly
                #opencv hoat dong o dinh dang BGR nhung mediapipe hoat dong o dinh dang RGB
                image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
                image.flags.writeable = False
                results = hands.process(image)

                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                if results.multi_hand_landmarks:
                    GestureController.classify_hands(results)
                    handmajor.update_hand_result(GestureController.hr_major)
                    handminor.update_hand_result(GestureController.hr_minor)

                    handmajor.set_finger_state()
                    handminor.set_finger_state()
                    gest_name = handminor.get_gesture()

                    if gest_name == Gest.PINCH_MINOR:
                        Controller.handle_controls(gest_name, handminor.hand_result)
                    else:
                        gest_name = handmajor.get_gesture()
                        Controller.handle_controls(gest_name, handmajor.hand_result)

                    for hand_landmarks in results.multi_hand_landmarks:
                        mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                else:
                    Controller.prev_hand = None
                cv2.imshow('Virtual Mouse', image)

                pressedKey = cv2.waitKey(1)
                if pressedKey == ord('q'):
                    break
                # if cv2.waitKey(5) & 0xFF == 27:
                #     break
        GestureController.cap.release()
        cv2.destroyAllWindows()

