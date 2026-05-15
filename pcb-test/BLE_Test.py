# BLE Test

from machine import UART
import time

# Initialize UART
uart = UART(3, baudrate=9600)

active = 0


def parseRX():
    header = ""
    message = ""
    state = ""

    while uart.any():
        retVal = uart.read(1)

        if retVal == b'\x01':
            state = "SOH"
        elif retVal == b'\x02':
            state = "STX"
        elif retVal == b'\x03':
            state = "ETX"
            break
        else:
            char = retVal.decode('utf-8')
            if (state == "SOH"):
                header += char
            elif (state == "STX"):
                message += char

    return [header, message]


def uart_rx_handler(uart):
    command, state = parseRX()

    if (command == "PWR"):
        state = int(state)
        if (state == 0 or state == 1):
            global active
            active = state


uart.irq(handler=uart_rx_handler, trigger=UART.IRQ_RXIDLE)


while True:
    if active:
        print("Driving")
    else:
        print("Not Driving")

    time.sleep(0.1)
