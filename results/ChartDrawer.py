import matplotlib.pyplot as plt
import numpy as np

file_python = '../resources/pythontime.txt'
# file_c = '../resources/ctime.txt'

python_times = np.loadtxt(file_python)
# c_times = np.loadtxt(file_c)

plt.plot(python_times, label="Python time")
# plt.plot(c_times, label="C time")
plt.xlabel("Wielkość danych")
plt.ylabel("Czas")
plt.legend()
plt.show()
