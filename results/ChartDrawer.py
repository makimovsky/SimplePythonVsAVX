import matplotlib.pyplot as plt
import numpy as np

file_python = 'pythontime.txt'
file_c = 'ctime.txt'

python_time = np.loadtxt(file_python)
c_time = np.loadtxt(file_c)
colors = ['skyblue', 'orange']

times = np.array([python_time[-1], c_time[-1]])
labels = np.array(["Python", "C with AVX2"])
plt.barh(labels, times, color=colors, log=True)
plt.xlim(right=100)
plt.xlabel("Time (logarithmic scale)")

for index, value in enumerate(list(times)):
    plt.text(value, index, f'{value:.2f}', va='center', ha='left', color='black', fontsize=10)

plt.savefig('result.png', bbox_inches='tight')

plt.show()
