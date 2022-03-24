if __name__ == "__main__":
    import serial
    import argparse
    from datetime import datetime

    ser = serial.Serial()
    ser.port = "COM5"
    ser.baudrate = 115200
    ser.timeout = 1

    ser.open()

    parser = argparse.ArgumentParser()
    parser.add_argument("--start", action="store_true")
    parser.add_argument("--stop", action="store_true")
    parser.add_argument("--set-timer", action="store")
    args = parser.parse_args()

    if args.start:
        try:
            ser.write(bytes([0xA0, 0, 0, 0, 0]))
            f = open(f"log {datetime.now().strftime('%d_%m_%y_%H_%M_%S')}.txt", "w")

            while True:
                uart_data = ser.readline().decode().replace("\r", "").replace("\n", "")
                if uart_data:
                    print(uart_data)
                    f.write(f"{datetime.now().strftime('%d-%m-%y %H:%M:%S')} {uart_data[15:]}\n")

        except KeyboardInterrupt:
            print("\nTemperature measurement terminated")
            f.close()

    if args.stop:
        ser.write(bytes([0xB0, 0, 0, 0, 0]))
        print("\nMeasurement paused")

    if args.set_timer:
        try:
            sec = int(args.set_timer, 0)
        except ValueError:
            print("Enter timer period in seconds")
        else:
            if sec <= 0:
                print("Timer cannot be less than 1 second")
            else:
                ser.write((bytes([0xC0]) + sec.to_bytes(4, byteorder='big')))
                print(f"Timer was set to {sec}")

    ser.close()