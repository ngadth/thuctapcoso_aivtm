import cv2
import numpy as np


class Key():

    def __init__(self, x, y, w, h, text):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.text = text
    #ve mot hinh chu nhat la o chu và ve chu tren hinh do
    def drawKey(self, img, text_color=(255, 255, 255), bg_color=(0, 0, 0), alpha=0.5, fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.8, thickness=2):
        # ve o chu
        bg_rec = img[self.y: self.y + self.h, self.x: self.x + self.w]
        white_rect = np.ones(bg_rec.shape, dtype=np.uint8)  # * 25
        white_rect[:] = bg_color
        res = cv2.addWeighted(bg_rec, alpha, white_rect, 1 - alpha, 1.0)

        #dat hinh anh lai vi tri cua no
        img[self.y: self.y + self.h, self.x: self.x + self.w] = res

        # viet chu len
        tetx_size = cv2.getTextSize(self.text, fontFace, fontScale, thickness)
        text_pos = (int(self.x + self.w / 2 - tetx_size[0][0] / 2), int(self.y + self.h / 2 + tetx_size[0][1] / 2))
        cv2.putText(img, self.text, text_pos, fontFace, fontScale, text_color, thickness)

    #kiem tra xem toa do x,y co nam trong khung chu nhat khong
    def isOver(self, x, y):
        if (self.x + self.w > x > self.x) and (self.y + self.h > y > self.y):
            return True
        return False