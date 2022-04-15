import csv
import matplotlib.pyplot as plt

measurements = ['roll', 'pitch', 'yaw', 'xaccel', 'yaccel', 'zaccel']

data_dict = { measurement: [] for measurement in measurements }

with open('data.txt', 'rb') as data_file:
    data = data_file.readline()
    counter = int(data)
    for measurement in measurements:
        data = data_file.readline()
        for _ in range(counter):
            data = data_file.readline()
            data_dict[measurement].append(float(data))

plot1_measurements = measurements[:3]
for measurement in plot1_measurements:
    plt.plot(data_dict[measurement])
plt.legend(plot1_measurements)

plt.figure()
plot2_measurements = measurements[3:]
for measurement in plot2_measurements:
    plt.plot(data_dict[measurement])
plt.legend(plot2_measurements)
plt.show()
