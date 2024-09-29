import numpy as np
import matplotlib.pyplot as plt

# Data points (swapped)
z_values = [179.69122462116238, 63.18818064992596, -21.90408503780731, -160.33396836092808]
z_offsets = [-0.02989050802506199, 0.12244655476871458, 0.3132287246823928, 0.514999778740389]

# Convert lists to numpy arrays
z_values = np.array(z_values)
z_offsets = np.array(z_offsets)

# Calculate the line of best fit
coefficients = np.polyfit(z_values, z_offsets, 1)
slope, intercept = coefficients

# Print the line of best fit equation
print(f"Line of best fit: z_offset = {slope:.6f} * z + {intercept:.6f}")

# Define the line of best fit
line_of_best_fit = slope * z_values + intercept

# Create scatter plot
plt.scatter(z_values, z_offsets, color='blue', label='Data points')

# Plot the line of best fit
plt.plot(z_values, line_of_best_fit, color='red', label=f'Line of best fit: z_offset = {slope:.2f} * z + {intercept:.2f}')

# Add labels and title
plt.xlabel('z')
plt.ylabel('z_offset')
plt.title('Scatter Plot and Line of Best Fit for z_offset vs z')
plt.legend()

# Show grid
plt.grid(True)

# Show the plot
plt.show()
