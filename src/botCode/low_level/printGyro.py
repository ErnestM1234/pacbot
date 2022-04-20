from motorsTesting import *

# goes forwards and stops at a wall
def main():
    
    bot = ArduinoMotors()
    bot.stop()
    while True:
        bot.arduino.read()
        # print("heading:" + str(bot.arduino.getHeading()))
        # print(str(bot.arduino.getHeading()))
main()