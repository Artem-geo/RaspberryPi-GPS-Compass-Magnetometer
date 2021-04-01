import serial
import time
import datetime
import RPi.GPIO as GPIO


# В аннотации к работе функции в скобках указано время исполнения
def cur_date(m):
    """
    Текущая дата (300 ms)
    """
    m.write(b'date\x00')
    m.read_until(b'\x00')
    flush_cash(m)


def set_date(m, date):
    """
    Установка времени и даты (300 ms)
    """
    date = ('date ' + date + '\x00').encode('ascii')
    m.write(date)
    m.read_until(b'\x00')
    flush_cash(m)


def cur_time(m):
    """
    Время  (300 ms)
    """
    m.write(b'time\x00')
    m.read_until(b'\x00')
    flush_cash(m)


def set_time(m, time):
    """
    Установка времения (300 ms)
    """
    time = ('time ' + time + '\x00').encode('ascii')
    m.write(time)
    m.read_until(b'\x00')
    flush_cash(m)


def set_mode(m, t):
    """
    Режим работы магнитометра (300 ms)
    """
    m.write(('mode ' + t + '\x00').encode('ascii'))
    m.read_until(b'\x00')
    flush_cash(m)


def set_range(m, d):
    """
    Диапазон измерения(300 ms)
    """
    m.write(('range ' + d + '\x00').encode('ascii'))
    m.read_until(b'\x00')
    flush_cash(m)


def run(m):
    """
    Выполнить единичное измерение (4000 ms)
    """
    m.write(b'run\x00')
    m.read_until(b'\x00')
    flush_cash(m)


def auto(m, filename, freq=-2):
    """
    Начало автоматической записи магнитометра (5000 ms)
    m - объект магнитометра
    filename - название файла
    freq - частота (по умолчанию, -2 (2 Гц))
    """
    m.write(b'auto ' + to_hexadecimal(freq, 32) + b'\x00')
    # m.write(b'auto 1\x00')
    time.sleep(4)
    flush_cash(m)
    with open('/home/pi/Documents/data/' + filename, 'wb') as f:

        GPIO.output(LED_yellow, GPIO.HIGH)

        while True:
            s = m.read_until(b'\x00')

            # При потере связи с магнитометром или при нажатии кнопки
            # программа будет остановлена
            if len(s) == 0 or GPIO.event_detected(button):
                stop(m)
                f.close()
                m.close()

                GPIO.output(LED_yellow, GPIO.LOW)
                GPIO.cleanup()
                break
            cur_time = str(datetime.datetime.now().timestamp()).split('.')
            f.write(s + to_hexadecimal(int(cur_time[0]), 64) + to_hexadecimal(int(cur_time[1]), 64) + b'\n')


def flush_cash(m):
    """
    Очистка буфера обмена (<1000 ms)
    """
    m.reset_input_buffer()
    m.reset_output_buffer()


def about(m):
    """
    Получение информации о магнитометре(300 ms)
    """
    m.write(b'about\x00')
    m.read_until(b'\x00')
    flush_cash()


def stop(m):
    """
    Остановка измерений (1500 ms)
    """
    m.write(b'\x05\x00')
    m.read_until(b'\x00')
    flush_cash(m)


def to_hexadecimal(val, nbits):
    """
    Конвертирование целых чисел в шестнадцетеричные
    """
    hex_str = hex((val + (1 << nbits)) % (1 << nbits)).replace('0x', '')
    if len(hex_str) % 2 != 0:
        hex_str = '0' + hex_str
    return bytes.fromhex(hex_str)


# присвоение pin-ов переменным
LED_yellow = 31
button = 16

GPIO.setmode(GPIO.BOARD)  # установка режима нумерации pin-ов от 1 - 40
GPIO.setwarnings(False)  # отлючение всех предупреждений

# инициализация pin-ов
GPIO.setup(LED_yellow, GPIO.OUT, initial=GPIO.LOW)  # green_led - output channel
GPIO.setup(button, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # button - input channel

# при нажатии на кнопку цикл/программа должна прекращить работу
GPIO.add_event_detect(button, GPIO.FALLING)

# ожидание подключения магнитометра
while True:
    try:
        m = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)
        break
    except:
        pass
# инициализация параметров работы магнитометра
flush_cash(m)
set_mode(m, 'text')

# дата и время
set_date(m, datetime.datetime.now().strftime('%m-%d-%y'))
set_time(m, datetime.datetime.now().strftime('%H:%M:%S'))

# значение поля
set_range(m, '55000')

set_mode(m, 'binary')

# считывание параметров с файла param
with open('/home/pi/Documents/scripts/param', 'r') as prs:
    freq = int(prs.readline())  # частота работы магнитометра
    decl = float(prs.readline())  # склонение магн.поля в районе работ (dd.mm)
    prs.close()

while True:
    time.sleep(0.5)
    GPIO.output(LED_yellow, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(LED_yellow, GPIO.LOW)
    if GPIO.event_detected(button):
        break

time.sleep(1.5)
filename = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_MG.txt')
auto(m, filename, freq)
