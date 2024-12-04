import serial
import time
import matplotlib.pyplot as plt

arduino_port = 'COM3'
baud_rate = 9600
ser = serial.Serial(arduino_port, baud_rate)
time.sleep(2)  
timestamps = []
sensor_values = []

def runForceMode():
    while ser.in_waiting < 0:
        time.sleep(1)
        # wait
    data = ser.readline().decode('utf-8').strip()
    print("calibrated angle: " + data)
    print("place object onto platform A and wait for the scale to come to equilibrium, then press ENTER to continue.")
    inputparam = input()
    if inputparam == '':
        ser.write(b"f\n")
        ForceMode()
    else:
        start()
    ser.write(b"r\n")

def ForceMode():
    #plotting
    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot(timestamps, sensor_values, '-o')
    plt.xlabel("Time (s)")
    plt.ylabel("Sensor Value")
    plt.title("Real-Time Sensor Data")

    start_time = time.time()

    try:
        while True:
            if ser.in_waiting > 0:  # Check if there is data to read
                data = ser.readline().decode('utf-8').strip()
                if data == "end":
                    stop()
                # reads from serial at baud rate set
                try:
                    value = float(data)
                    current_time = time.time() - start_time
                    timestamps.append(current_time)
                    sensor_values.append(value)

                    line.set_xdata(timestamps)
                    line.set_ydata(sensor_values)
                    ax.relim()
                    ax.autoscale_view()
                    plt.draw()
                    plt.pause(0.01)
                except ValueError:
                    print(data)
                    stop()
    except:
        stop()

def stop():
    print("stopping")
    ser.write(b"r\n")
    ser.close()
    plt.ioff()
    plt.show()

def start():

    print("\n\nTo start FORCE MODE, press ENTER.")
    print("ensure the scale is at equilibrium before starting.")
    inputparam = input()
    if inputparam == '':
        ser.write(b"s\n")
        runForceMode();
    else:
        start()

if __name__ == "__main__":
    start()