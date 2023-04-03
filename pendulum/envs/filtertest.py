import numpy as np
from scipy.signal import savgol_filter

# Generate some noisy data
x = np.linspace(0, 10, 101)
y = np.sin(x) + np.random.normal(0, 0.1, 101)

# Apply the Savitzky-Golay filter
y_filtered = savgol_filter(y, window_length=11, polyorder=2)

# Plot the results
import matplotlib.pyplot as plt
plt.plot(x, y, label='Noisy data')
plt.plot(x, y_filtered, label='Filtered data')
plt.legend()
plt.show()