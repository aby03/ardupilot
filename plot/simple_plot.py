import matplotlib.pyplot as plt
import csv

x = []
y = []

with open('custom.log','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=' ')
    for row in plots:
    	# print('T: ', row[1], ' C: ', row[3])
        x.append(float(row[1]));
        y.append(float(row[3]));
counter = range(len(x))
plt.plot(counter,x,label='Target')
plt.plot(counter,y,label='Current')
plt.xlabel('Iterations')
plt.ylabel('Value')
plt.title('Yaw PID Graph')
plt.legend()
plt.show()