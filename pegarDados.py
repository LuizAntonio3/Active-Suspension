import serial
from datetime import datetime


def main():
    
    ser = serial.Serial(port="COM9", baudrate=500000)

    now = datetime.now()
    datetimeString = now.strftime("%d-%m-%Y %H-%M")

    data = open(f"dados/testes/susp-{datetimeString}.csv", "w")

    # data.write("CONTROLLER_FLAG,SL_PWM,U,Motor_speed,SL_position,SL_speed,ARM_position,ARM_speed,Deflection,Deflection_speed,Zlin_position,Zlin_speed,Unsprung_acceleration,Sprung_acceleration_x,Sprung_acceleration_y,Sprung_acceleration_z\n")
    data.write("CONTROLLER_FLAG,SL_PWM,U,SL_position,SL_speed,ARM_position,Distance,Sprung_acceleration_x\n")
    # data.write("SL_PWM,SL_position\n")

    i = 0
    print(ser.name)

    while(1):
        try:
            
            line = ser.readline()
            
            try:
                line = str(line, encoding='utf8')
            except Exception:
                pass

            # print(f"{i} -> {line}")
            # i = i + 1
            try:
                if not "requestFrom()" in line and line.count(",") >= 3:
                    print(f"{i} -> {line}", end="")
                    data.write(line)
                    i = i + 1
                else:
                    pass
            except Exception:
                pass

            if(i == 130000):
                ser.close()
                data.close()
                print("End of data")
                exit(0)

        except KeyboardInterrupt:
            ser.close()
            data.close()    
            print("Stopped by CTRL-C")
            exit(0)

if __name__ == "__main__":
    main()