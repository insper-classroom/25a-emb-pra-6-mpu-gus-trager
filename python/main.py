import sys, glob, serial, threading
import pyautogui
pyautogui.FAILSAFE = False
pyautogui.PAUSE = 0

import tkinter as tk
from tkinter import ttk, messagebox
from pyautogui import FailSafeException

def move_mouse(axis, value):
    try:
        if axis == 0:
            pyautogui.moveRel(value, 0) 
        elif axis == 1:
            pyautogui.moveRel(0, -value)
    except FailSafeException:
        pass

def parse_data(data):
    axis = data[0]
    value = int.from_bytes(data[1:3], byteorder='big', signed=True)
    return axis, value

def controle(ser):
    try:
        while True:
            sync = ser.read(1)
            if not sync or sync[0] != 0xFF:
                continue
            packet = ser.read(3)
            if len(packet) < 3:
                continue
            axis, value = parse_data(packet)
            if axis == 2 and value == 1:
                try:
                    pyautogui.click()
                except FailSafeException:
                    pass
            else:
                move_mouse(axis, value)
    except serial.SerialException:
        pass
    finally:
        ser.close()

def serial_ports():
    ports = []
    if sys.platform.startswith('win'):
        ports = [f'COM{i}' for i in range(1,256)]
    elif sys.platform.startswith(('linux','cygwin')):
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Plataforma não suportada.')
    return [p for p in ports if _test_port(p)]

def _test_port(p):
    try:
        s = serial.Serial(p); s.close(); return True
    except: return False

def conectar_porta(port_name, btn, status_label, change_circle):
    if not port_name:
        messagebox.showwarning("Aviso", "Selecione uma porta serial.")
        return
    try:
        ser = serial.Serial(port_name, 115200, timeout=1)
    except Exception as e:
        messagebox.showerror("Erro", f"Não foi possível abrir {port_name}:\n{e}")
        change_circle("red")
        status_label.config(text="Erro de conexão", fg="red")
        return

    status_label.config(text=f"Conectado em {port_name}", fg="green")
    change_circle("green")
    btn.config(text="Conectado", state="disabled")
    btn.winfo_toplevel().iconify()

    threading.Thread(target=controle, args=(ser,), daemon=True).start()

def criar_janela():
    root = tk.Tk()
    root.title("Controle de Mouse por IMU")
    root.geometry("400x250")
    dark_bg, dark_fg = "#2e2e2e", "#ffffff"
    root.configure(bg=dark_bg)

    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure("TButton", font=("Segoe UI", 10, "bold"))
    style.configure("TLabel", background=dark_bg, foreground=dark_fg)

    frame = ttk.Frame(root, padding=20)
    frame.pack(expand=True, fill="both")

    ttk.Label(frame, text="Controle de Mouse", font=("Segoe UI", 14, "bold")).pack(pady=(0,10))

    portas = serial_ports()
    porta_var = tk.StringVar(value=portas[0] if portas else "")

    ttk.Combobox(frame, values=portas, textvariable=porta_var, state="readonly").pack()

    btn = ttk.Button(frame, text="Conectar",
                     command=lambda: conectar_porta(porta_var.get(), btn, status_label, change_circle))
    btn.pack(pady=10)

    status_label = tk.Label(frame, text="Aguardando porta...",
                            bg=dark_bg, fg=dark_fg, font=("Segoe UI", 11))
    status_label.pack()

    circle = tk.Canvas(frame, width=20, height=20, bg=dark_bg, highlightthickness=0)
    item = circle.create_oval(2,2,18,18, fill="red")
    circle.pack(pady=(5,0))

    def change_circle(color):   
        circle.itemconfig(item, fill=color)

    root.mainloop()

if __name__ == "__main__":
    criar_janela()