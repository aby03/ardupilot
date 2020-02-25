import os
import matplotlib.pyplot as plt
import csv

# all_subdirs = [d for d in os.listdir('.') if os.path.isdir(d)]
def all_subdirs_of(b='.'):
  result = []
  for d in os.listdir(b):
    bd = os.path.join(b, d)
    if os.path.isdir(bd): result.append(bd)
  return result

# Ubuntu /tmp/ path from windows
tmppath = 'C:\\Users\\Asus Pc\\AppData\\Local\\Packages\\CanonicalGroupLimited.UbuntuonWindows_79rhkp1fndgsc\\LocalState\\rootfs\\tmp'
#tmppath = 'C:\\Users\\Asus Pc\\AppData\\Local\\Packages\\CanonicalGroupLimited.UbuntuonWindows_79rhkp1fndgsc\\LocalState\\rootfs\\tmp'

all_subdirs = all_subdirs_of(tmppath)
latest_subdir = max(all_subdirs, key=os.path.getmtime)
fpath = latest_subdir+'\\custom.log'

x = []
y = []

with open(fpath,'r') as csvfile:
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