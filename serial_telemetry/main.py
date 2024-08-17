import random
import sys, threading, queue, serial
import serial.tools.list_ports
from time import sleep
import signal
from threading import Thread
from PyQt6.QtWidgets import QMainWindow
from PyQt6.QtWidgets import QApplication

import pglive.examples_pyqt6 as examples
from pglive.examples_pyqt6.designer_example.win_template import Ui_MainWindow
from pglive.sources.data_connector import DataConnector
from pglive.sources.live_plot import LiveLinePlot
from pglive.sources.live_plot_widget import LivePlotWidget


class MainWindow(QMainWindow, Ui_MainWindow):
    """Create main window from template"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)


def selectArduino():
    ports = serial.tools.list_ports.comports()
    choices = []
    print('PORT\tDEVICE\t\t\tMANUFACTURER')
    for index, value in enumerate(sorted(ports)):
        if (value.hwid != 'n/a'):
            choices.append(index)
            print(index, '\t', value.name, '\t',
                  value.manufacturer)  # https://pyserial.readthedocs.io/en/latest/tools.html#serial.tools.list_ports.ListPortInfo

    choice = -1
    while choice not in choices:
        answer = input("➜ Select your port: ")
        if answer.isnumeric() and int(answer) <= int(max(choices)):
            choice = int(answer)
    print('selecting: ', ports[choice].device)
    return ports[choice].device


def listenToArduino():
    message = b''
    while True:
        incoming = arduino.read()
        if (incoming == b'\n'):
            arduinoQueue.put(message.decode('utf-8').strip().upper())
            message = b''
        else:
            if ((incoming != b'') and (incoming != b'\r')):
                message += incoming


def listenToLocal():
    while True:
        command = sys.stdin.readline().strip().upper()
        localQueue.put(command)


def configureUserInput():
    localThread = threading.Thread(target=listenToLocal, args=())
    localThread.daemon = True
    localThread.start()


def configureArduino():
    global arduinoPort
    arduinoPort = selectArduino()
    global arduino
    arduino = serial.Serial(arduinoPort, baudrate=baudRate, timeout=.1)
    arduinoThread = threading.Thread(target=listenToArduino, args=())
    arduinoThread.daemon = True
    arduinoThread.start()


def handleLocalMessage(aMessage):
    print("=> [" + aMessage + "]")
    arduino.write(aMessage.encode('utf-8'))
    arduino.write(bytes('\n', encoding='utf-8'))


def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


# ---- MAIN CODE -----
baudRate = 460800
arduinoQueue = queue.Queue()
localQueue = queue.Queue()
configureArduino()  # will reboot AVR based Arduinos
configureUserInput()  # handle stdin

# pglive code
app = QApplication(sys.argv)
running = True
plot_widget = LivePlotWidget(title="Arduino Stream")
plot_curve = LiveLinePlot()
plot_widget.addItem(plot_curve)
# DataConnector holding 600 points and plots @ 100Hz
data_connector = DataConnector(plot_curve, max_points=600)


def plotter(connector, arduinoQueue):
    x = 0
    while running:
        if not arduinoQueue.empty():
            dat_in = arduinoQueue.get()
            if is_number(dat_in):
                x += 0.01
                data_point = float(dat_in)
                connector.cb_append_data_point(data_point, x)
        sleep(.001)


plot_widget.show()
Thread(target=plotter, args=(data_connector, arduinoQueue)).start()
app.exec()
running = False
