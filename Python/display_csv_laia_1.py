import serial
import time
import datetime
import math
from tkinter import *
from string import ascii_letters

tk = Tk()
fondo = "bisque"
# Relacion entre los circulos
R = 250
pos_c_x = 350
pos_c_y = 350
diagonal = math.sin(math.pi/4)
#Propiedades de todos los circulos
radio = 90
color_backspace = "OrangeRed3"
color_backspace_sel = "red2"
color = "MediumPurple1"
color_sel = "MediumPurple4"
# Prodiedades circulos con letras
R_letras = 55
radio_letras = 25
color_letra = ["blue4","yellow3","red3","green4"]
color_letra_sel = ["blue2","yellow2","red1","green3"]
cadena_letras = ["", "", "", "",    "a","b","c","d",    "e","f","g","h",    "y","z",".",",",    "","","","",    "i","j","k","l",  "u","v","w","x",    "q","r","s","t",    "m","n","o","p",]

tk.geometry("800x800")

tk.title("First title")

def letter_selection(letter):
    labelfont = ('times', 202, 'bold')
    widget = Label(tk, text=letter)
    widget.config(bg=fondo, fg='black')
    widget.config(font=labelfont)
    widget.config(height=0, width=2)
    # widget.place(x=0,y=0)
    # widget.pack(fill=Y, side=LEFT)
    widget.grid(row=0, column=0,sticky=N+S+E+W)

def joy(selec_joy, selec_pie):
    selec_letra = selec_joy * 4 + selec_pie
    #print(str(selec_joy) + "  " + str(selec_pie))
    canvas = Canvas(tk, width=800,height=800,bg=fondo)
    # canvas.pack()
    canvas.grid(row=0, column=1, rowspan=2,sticky=N+S+E+W)
    pos = []
    pos_letra = []
    text_letra = []

    def circulo(centro_x, centro_y, radio,color, num):
        return canvas.create_oval(centro_x - radio, centro_y - radio, centro_x + radio, centro_y + radio, fill=color)

    def interior_circulo(centro_x, centro_y, radio,color, num):
        circulo_letra(centro_x - R_letras, centro_y, radio_letras,color_letra[0],cadena_letras[num * 4])          #Pos pie 0
        circulo_letra(centro_x, centro_y - R_letras, radio_letras,color_letra[1], cadena_letras[num * 4 + 1])          #Pos pie 1
        circulo_letra(centro_x + R_letras, centro_y, radio_letras,color_letra[2], cadena_letras[num * 4 + 2])          #Pos pie 2
        circulo_letra(centro_x, centro_y + R_letras, radio_letras,color_letra[3], cadena_letras[num * 4 + 3])          #Pos pie 3

    def circulo_letra(centro_x, centro_y, radio,color, letra):
        pos_letra.append(canvas.create_oval(centro_x - radio, centro_y - radio, centro_x + radio, centro_y + radio, fill=color))
        text_letra.append(canvas.create_text(centro_x - radio+radio, centro_y - radio+radio, text=letra,font=("Purisa", radio),fill="white"))

    def anyadir_circulo(centro_x, centro_y, radio,color, num):
        pos.append(circulo(centro_x, centro_y, radio,color, num))
        if (num != 4 and num != 0):
            interior_circulo(centro_x, centro_y, radio,color, num)
        else:
            pos_letra.append("")
            pos_letra.append("")
            pos_letra.append("")
            pos_letra.append("")

    anyadir_circulo(pos_c_x - R * diagonal ,pos_c_y - R * diagonal  ,radio,color_backspace,0)             #Circulo 0
    anyadir_circulo(pos_c_x,pos_c_y - R,radio,color,1)                                          #Circulo 1
    anyadir_circulo(pos_c_x + R * diagonal ,pos_c_y - R * diagonal  ,radio,color,2)             #Circulo 2
    anyadir_circulo(pos_c_x - R, pos_c_y,radio,color,3)                                         #Circulo 3
    anyadir_circulo(pos_c_x,pos_c_y,radio,color,4)                                              #Circulo 4
    anyadir_circulo(pos_c_x + R,pos_c_y ,radio,color,5)                                         #Circulo 5
    anyadir_circulo(pos_c_x - R * diagonal ,pos_c_y + R * diagonal  ,radio,color,6)             #Circulo 6
    anyadir_circulo(pos_c_x ,pos_c_y + R,radio,color,7)                                         #Circulo 7
    anyadir_circulo(pos_c_x + R * diagonal ,pos_c_y + R * diagonal  ,radio,color,8)             #Circulo 8

    if(selec_joy == 0):
        canvas.itemconfig(pos[selec_joy], fill=color_backspace_sel)
    else:
        canvas.itemconfig(pos[selec_joy], fill=color_sel)
    canvas.itemconfig(pos_letra[selec_letra], fill="black")

    if (selec_joy != 4 and selec_joy != 0):
        coordenadas = canvas.coords(pos_letra[selec_letra])
        aumento = 7
        diametro = coordenadas[2] - coordenadas[0]
        radio_1 = int(diametro/2)
        centro_x = coordenadas[0] + radio_1
        centro_y = coordenadas[1] + radio_1
        radio_1 = radio_1 + aumento
        circulo_letra(centro_x, centro_y, radio_1,color_letra_sel[selec_pie], cadena_letras[selec_letra] )

def foot(pos):
    color_letra = 'black'
    if pos == 1:
        color_letra = 'blue4'
    elif pos == 2:
        color_letra = 'yellow3'
    elif pos == 3:
        color_letra = 'red3'
    elif pos == 4:
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
    widget2.grid(row=1, column=0,sticky=N+S+E+W)

def selector_letra(joy,pie):
    num_letra = joy * 4 + pie
    return cadena_letras[num_letra]

def init_display():
    letter_selection("E")
    joy(4,3)
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
    msg = st_nucleo.readline()          #Lee el puerto serie
    msg_decode = msg.decode('utf-8')    #Traduce el mensaje
    fw.write(msg_decode)                #Lo guarda en un archivo
    msg_split = msg_decode.split(",")

    # letter_selection(msg_split[10])
    # joy(msg_split[1])
    # foot(msg_split[6])

    val_joy = int(float(msg_split[10]))
    val_pie = int(float(msg_split[12]))
    val_1 = selector_letra(val_joy,val_pie)
    foot(val_pie)
    joy(val_joy,val_pie)
    letter_selection(val_1)

    x = x + 1
    if x > 1:
        x = 0
        tk.update_idletasks()
        tk.update()
        print("Selecion -->" + msg_decode)
