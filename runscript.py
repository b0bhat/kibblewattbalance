import serial
import time
import matplotlib.pyplot as plt

arduino_port = 'COM3'
baud_rate = 9600
ser = serial.Serial(arduino_port, baud_rate)
time.sleep(2)  

timestamps = []
sensor_values = []

ser.write(b"START\n")

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
            # reads from serial at baud rate set
            try:
                value = int(data)  # string to int
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
                print(f"error {data}")
except KeyboardInterrupt:
    print("stopping")
    ser.close()
    plt.ioff()
    plt.show()