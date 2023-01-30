from tkinter import *
from Virtual_Mouse import *
from Vituarl_Paint import *
from Virtual_Keyboard import *

window = Tk()
window.title("Welcome To Virtual_Mouse App")
window.geometry('750x500')
cvFist = Canvas(window,width=750,height=500,bg="white")
cvFist.pack()

img1=PhotoImage(file='img1.png')
img2=PhotoImage(file='img2.png')
img3=PhotoImage(file='img3.png')
cvFist.create_image(0,0,image=img1,anchor=NW)
cvFist.create_image(341,2,image=img2,anchor=NW)
cvFist.create_image(341,380,image=img3,anchor=NW)

#Hàm khi nút được nhấn
def mouse():
    gc1 = GestureController()
    gc1.start()

def paint():
    m1 = Painter()
    m1.build()

def keyboard():
    k1 = KeyBoard()
    k1.build()


#Gọi hàm clicked khi nút được nhấn


btnMouse = Button(window, text="Virtual Mouse" ,command=mouse,bg='red',font = 'Times 9 bold')
btnMouse.place(x=350, y=335)

btnPaint = Button(window, text="Virtual Paint" ,command=paint,bg='yellow',font = 'Times 9 bold')
btnPaint.place(x=500, y=335)

btnKeyB = Button(window, text="Virtual KeyBoard" ,command=keyboard,bg='gray',font = 'Times 9 bold')
btnKeyB.place(x=620, y=335)

window.mainloop()
