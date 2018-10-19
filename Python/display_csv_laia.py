import serial
import time
import datetime
import math
from tkinter import *
from string import ascii_letters

tk = Tk()
fondo = "bisque"
# Relacion entre los circulos
R = 125
pos_c_x = 200
pos_c_y = 200
diagonal = math.sin(math.pi/4)
#Propiedades de todos los circulos
radio = 30
color = "MediumPurple1"
color_sel = "MediumPurple4"

# Prodiedades circulos con letras
R_letras = 55
radio_letras = 25
color_letras = "green4"
cadena_letras = ["", "", "", "",    "a","b","c","d",    "e","f","g","h",    "y","z","","",    "","","","",    "i","j","k","l",  "u","v","w","x",    "q","r","s","t",    "m","n","o","p",]


tk.geometry("800x800")

tk.title("Laia")
cadena_letras = ["", "", "", "",    "a","b","c","d",    "e","f","g","h",    "y","z","","",    "","","","",    "i","j","k","l",  "u","v","w","x",    "q","r","s","t",    "m","n","o","p",]
def selector_letra(joy,pie):
    num_letra = joy * 4 + pie
    return cadena_letras[num_letra]



def letter_selection(letter):
    labelfont = ('times', 202, 'bold')
    widget = Label(tk, text=letter)
    widget.config(bg=fondo, fg='black')
    widget.config(font=labelfont)
    widget.config(height=0, width=2)
    # widget.place(x=0,y=0)
    # widget.pack(fill=Y, side=LEFT)
    widget.grid(row=0, column=0,sticky=N+S+E+W)

# def joy(pos):
#     labelfont = ('times', 125, 'bold')
#     widget1 = Label(tk, text=pos)
#     widget1.config(bg='yellow', fg='black')
#     widget1.config(font=labelfont)
#     widget1.config(height=0, width=2)
#     # widget1.place(x=406,y=0)
#     # widget1.pack(side=LEFT)
#     widget1.grid(row=0, column=1,sticky=N)

# def joy(selec):
#     canvas = Canvas(tk, width=425,height=425,bg=fondo)
#     # canvas.pack()
#     canvas.grid(row=0, column=1,sticky=N)
#     pos = []
#     def circulo(centro_x, centro_y, radiom,color, num):
#         pos.append(canvas.create_oval(centro_x - radio, centro_y - radio, centro_x + 2 * radio, centro_y + 2 * radio, fill=color))
#
#     circulo(pos_c_x - R * diagonal ,pos_c_y - R * diagonal  ,radio,color,0)             #Circulo 0
#     circulo(pos_c_x,pos_c_y - R,radio,color,1)                                          #Circulo 1
#     circulo(pos_c_x + R * diagonal ,pos_c_y - R * diagonal  ,radio,color,2)             #Circulo 2
#     circulo(pos_c_x - R, pos_c_y,radio,color,3)                                         #Circulo 3
#     circulo(pos_c_x,pos_c_y,radio,color,4)                                              #Circulo 4
#     circulo(pos_c_x + R,pos_c_y ,radio,color,5)                                         #Circulo 5
#     circulo(pos_c_x - R * diagonal ,pos_c_y + R * diagonal  ,radio,color,6)             #Circulo 6
#     circulo(pos_c_x ,pos_c_y + R,radio,color,7)                                         #Circulo 7
#     circulo(pos_c_x + R * diagonal ,pos_c_y + R * diagonal  ,radio,color,8)             #Circulo 8
#
#     canvas.itemconfig(pos[selec], fill=color_sel)

def joy(selec):
    canvas = Canvas(tk, width=800,height=800,bg=fondo)
    # canvas.pack()
    canvas.grid(row=0, column=1, rowspan=2,sticky=N+S+E+W)
    pos = []

    def circulo(centro_x, centro_y, radio,color, num):
        return canvas.create_oval(centro_x - radio, centro_y - radio, centro_x + radio, centro_y + radio, fill=color)

    def interior_circulo(centro_x, centro_y, radio,color, num):
        circulo_letra(centro_x - R_letras, centro_y, radio_letras,"blue4",cadena_letras[num * 4])           #Pos pie 0
        circulo_letra(centro_x, centro_y - R_letras, radio_letras,'yellow3', cadena_letras[num * 4 + 1])           #Pos pie 1
        circulo_letra(centro_x + R_letras, centro_y, radio_letras,"red3", cadena_letras[num * 4 + 2])           #Pos pie 2
        circulo_letra(centro_x, centro_y + R_letras, radio_letras,"green4", cadena_letras[num * 4 + 3])           #Pos pie 3

    def circulo_letra(centro_x, centro_y, radio,color, letra):
        canvas.create_oval(centro_x - radio, centro_y - radio, centro_x + radio, centro_y + radio, fill=color)
        canvas.create_text(centro_x - radio+radio, centro_y - radio+radio, text=letra,font=("Purisa", radio),fill="white")

    def anyadir_circulo(centro_x, centro_y, radio,color, num):
        pos.append(circulo(centro_x, centro_y, radio,color, num))
        if num != 4:
            interior_circulo(centro_x, centro_y, radio,color, num)



    anyadir_circulo(pos_c_x - R * diagonal ,pos_c_y - R * diagonal  ,radio,color,0)             #Circulo 0
    anyadir_circulo(pos_c_x,pos_c_y - R,radio,color,1)                                          #Circulo 1
    anyadir_circulo(pos_c_x + R * diagonal ,pos_c_y - R * diagonal  ,radio,color,2)             #Circulo 2
    anyadir_circulo(pos_c_x - R, pos_c_y,radio,color,3)                                         #Circulo 3
    anyadir_circulo(pos_c_x,pos_c_y,radio,color,4)                                              #Circulo 4
    anyadir_circulo(pos_c_x + R,pos_c_y ,radio,color,5)                                         #Circulo 5
    anyadir_circulo(pos_c_x - R * diagonal ,pos_c_y + R * diagonal  ,radio,color,6)             #Circulo 6
    anyadir_circulo(pos_c_x ,pos_c_y + R,radio,color,7)                                         #Circulo 7
    anyadir_circulo(pos_c_x + R * diagonal ,pos_c_y + R * diagonal  ,radio,color,8)             #Circulo 8

    canvas.itemconfig(pos[selec], fill=color_sel)

def foot(pos):
    color_letra = 'black'
    if pos == 0:
        color_letra = 'blue4'
    elif pos == 1:
        color_letra = 'yellow3'
    elif pos == 2:
        color_letra = 'red3'
    elif pos == 3:
        color_letra = 'green4'
    else:
        color_letra = 'black'

    labelfont = ('times', 125, 'bold')
    widget2 = Label(tk, text=pos)
    widget2.config(bg=fondo, fg=color_letra)
    widget2.config(font=labelfont)
    widget2.config(height=0, width=2)
    # widget2.place(x=406,y=190)
    # widget2.pack(side=LEFT)
    widget2.grid(row=0, column=1,sticky=N+S+E+W)

def init_display():
    letter_selection("E")
    joy(3)
    foot(1)


st_nucleo = serial.Serial('COM7', 9600)
x = datetime.datetime.now()
file_name = "laia_" + str(x.day) + "-" + str(x.month) + "-" + str(x.year) + ".csv"
fw = open(file_name ,"a")
fr = open(file_name ,"r")
contenido = fr.read()
if contenido=='':
   print("Nuevo archivo: " + file_name )
   fw.write("X, Xsel, Xmax, Xmin, Xpaso, Y, Ysel, Ymax, Ymin, Ypaso, Posicion, Sesion\n")
fr.close()
bandera = 0
# while  str(st_nucleo.readline()) != "b'\rOK\n'":
while  bandera == 0:
    msg = st_nucleo.readline()
    msg_decode = msg.decode('utf-8')
    # print(msg_decode)
    if msg_decode == "OK\n":
        print("Ready")
        bandera = bandera + 1

init_display()

x = 0

while True:
    try:
        msg = st_nucleo.readline()          #Lee el puerto serie
        msg_decode = msg.decode('utf-8')    #Traduce el mensaje
        fw.write(msg_decode)                #Lo guarda en un archivo
        msg_split = msg_decode.split(",")

        # letter_selection(msg_split[10])
        # joy(msg_split[1])
        # foot(msg_split[6])

        val_joy = int(float(msg_split[10]))
        joy(val_joy)
        val_pie = int(float(msg_split[12]))
        foot(val_pie)
        val_1 = selector_letra(val_joy,val_pie)
        letter_selection(val_1)

        x = x + 1
        if x > 1:
            x = 0
            tk.update_idletasks()
            tk.update()
            print(msg_decode)
    except (UnicodeDecodeError , IndexError, ValueError) as e:
        continue
