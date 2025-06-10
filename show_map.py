import numpy as np
import matplotlib.pyplot as plt

# Load the CSV file
map_path = 'build/map.csv'
grid = np.loadtxt(map_path, delimiter=',')

# Binary thresholded map (clear obstacle vs. free space)
binary_grid = grid > 0.5  # Adjust threshold as needed (e.g. 0.5 or 0.7)

# Create side-by-side visualizations
fig, axs = plt.subplots(1, 2, figsize=(12, 6))

# 1. Grayscale original map
axs[0].imshow(grid, cmap='gray', origin='lower', vmin=0, vmax=1)
axs[0].set_title("Grayscale Occupancy Map")
axs[0].set_xlabel("X")
axs[0].set_ylabel("Y")
fig.colorbar(axs[0].images[0], ax=axs[0], label='Occupancy probability')

# 2. Binary threshold map
axs[1].imshow(binary_grid, cmap='gray', origin='lower')
axs[1].set_title("Binary Threshold Map (> 0.5)")
axs[1].set_xlabel("X")
axs[1].set_ylabel("Y")

# Save figure
plt.tight_layout()
plt.savefig("build/map_visualized.png", dpi=300)
print("Saved enhanced map visualization to build/map_visualized.png")
