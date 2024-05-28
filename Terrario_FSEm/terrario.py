import rp2
import machine
from utime import sleep, sleep_ms, sleep_us
import ustruct
import _thread
from lcd_api import LcdApi
from pico_i2c_lcd import I2cLcd

# I2C_ADDR     = 39
# I2C_NUM_ROWS = 2
# I2C_NUM_COLS = 16
# 
# i2c = machine.I2C(1, sda=machine.Pin(2), scl=machine.Pin(3), freq=400000)
# lcd = I2cLcd(i2c, I2C_ADDR, I2C_NUM_ROWS, I2C_NUM_COLS)

vref = 3.3
temperature = 0
desiredTemp = 25.0
KP = 21
KI = 2.1
KD = 0.21
error = [0,0,0]
ciclos = 0
#freq = machine.PWM.freq(1000)
#duty = machine.PWM.duty_u16(0)

zCrossPin = machine.Pin(0, machine.Pin.IN)
dimmerPin = machine.Pin(1, machine.Pin.OUT)
pump = machine.Pin(10, machine.Pin.OUT)
fan = machine.PWM(22,freq=1000,duty_u16=0)
peltier = machine.Pin(23, machine.Pin.OUT)
tempMinusLeft = machine.ADC(0)        
tempPlusLeft = machine.ADC(1)
tempInput = machine.ADC(2)
hygro = machine.ADC(3)

def setup():
    global pio
    #lcd.clear()
    pio = rp2.StateMachine(0, Dimmer, freq=10000, in_base=zCrossPin, set_base=dimmerPin)
    pio.active(1)
    pump.value(0)
    _thread.start_new_thread(HumidityControl,())
    
# def CustomCharacter():
#     lcd.custom_char(0, bytearray([
#       0x07,
#       0x05,
#       0x07,
#       0x00,
#       0x00,
#       0x00,
#       0x00,
#       0x00 
#             ]))

#     lcd.custom_char(1, bytearray([
#        0x00,
#        0x18,
#        0x19,
#        0x02,
#        0x04,
#        0x08,
#        0x13,
#        0x03 
#             ])) 

# def WriteLCD():
#     CustomCharacter()
#     lcd.clear()
#     lcd.move_to(0,0)
#     lcd.putstr("DESIRED TEMP:")
#     lcd.move_to(0,1)
#     lcd.putstr(str(desiredTemp))
#     lcd.putchar(chr(0))
#     lcd.putstr("C")
#     sleep(3)
#     lcd.clear()
#     lcd.putstr("CURRENT TEMP:")
#     lcd.move_to(0,1)
#     lcd.putstr(str(temperature))
#     lcd.putchar(chr(0))
#     lcd.putstr("C")
#     sleep(3)
#     lcd.clear()
#     lcd.putstr("HUMIDITY:")
#     lcd.move_to(0,1)
#     lcd.putstr(str(humidityPercent))
#     lcd.putchar(chr(1))
#     sleep(3)

def HumidityControl():
    global humidityPercent
    while True:
        try:
            humidity = hygro.read_u16()
            humidityPercent = 100*humidity/65536
            
            if humidityPercent <= 30:
                pump.value(1)
            else:
                pump.value(0)
            sleep(1)
            
        except:
            print("\tHumidity Error")
            pump.value(0)

def ReadDesiredTemp():
    voltDesiredTemp = tempInput.read_u16()
    return round(0.00015*voltDesiredTemp+20,2)

def ReadTemp():
    global temperature
    voltDiffLeft = 0
    voltDiffRight = 0
    for i in range(20):
        voltPlusLeft  = tempPlusLeft.read_u16()
        voltMinusLeft = tempMinusLeft.read_u16()
        voltDiffLeft  += (voltPlusLeft - voltMinusLeft)
    
    voltDiffLeft = voltDiffLeft/20
    temperature = round((voltDiffLeft * vref)/655.36,2)
    return temperature

def sumError(errorArray):
    totalError = 0
    for i in errorArray:
        totalError += i
    return totalError

def TemperatureControl():
    global error, desiredTemp
    while True:
        try:
            desiredTemp = ReadDesiredTemp()
            print("  Desired temperature: {:0.2f}°C".format(desiredTemp), end="")
            currentTemp = ReadTemp()
            print("\r  Current temperature: {:0.2f}°C".format(currentTemp), end="")
            print("\t  Humidity: {:0.2f}%".format(humidityPercent), end="")
            currentError = desiredTemp - currentTemp
            error.append(currentError)
            if len(error) > 3:
                error.pop(0)
            totalError = sumError(error)
            diffError = error[2] - error[1]
            control = KP * currentError + KI * totalError + KD * diffError
            if control < 0:
                delays = 31
                if control >= -30 and control < 0:
                    duty = int(-655.36*control)
                    fan.duty_u16(duty)
                    peltier.value(0)
                if control >= -100 and control < -30:
                    duty = int(-655.36*control)
                    fan.duty_u16(duty)
                    peltier.value(1)
                if control < -100:
                    fan.duty_u16(65536)
                    peltier.value(1)
            if control >= 0 and control <= 100:
                delays = -0.31*control+31
                peltier.value(0)
            if control > 100:
                delays = 0
                peltier.value(0)
            pio.put(int(delays))
            sleep(1)
        except:
            print("\tTemperature Error!")
            pio.put(31)

def ReceiveDesiredTemp():
    pass
            
def EmergencyStop():
    print('error')

@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def Dimmer():
    set(pins,0)
    pull()
    mov(x,osr)
    wrap_target
    label('cruce')
    wait(0,pin,0)
    pull(noblock)
    mov(x,osr)
    mov(y,x)
    wait(1,pin,0)
    nop() [11]
    label('delay')
    nop()
    jmp(y_dec,'delay')
    set(pins,1)
    set(pins,0)
    jmp('cruce')
    wrap()

def main():
    setup()
    print("-= PID controller =-")
    TemperatureControl()
#     while True:
#         try:
#             print("HI")
#             sleep_ms(3000)
#         except:
#             print("\tError!")
#             pio.put(31)
    
if __name__ == '__main__':
    main()
