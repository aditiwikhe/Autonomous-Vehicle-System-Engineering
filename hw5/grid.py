import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Load CSV file into Pandas DataFrame
df = pd.read_csv('xyz_lidar.csv')
df.columns = ('x','y','z')
print(df)

# Define the number of desired grids
num_grids = 20
look_radius = 6

# clip z co-ordiantes
df = df[(df['z'] >= -1) & (df['z'] <= 1)]

df = df[(df['x'] <= look_radius) & ((df['x'] >= (-1) * look_radius)) \
        & (df['y'] <= look_radius) & (df['y'] >= (-1) * look_radius)]

# Calculate the grid size based on the number of desired grids
grid_size = np.ceil(np.max([df['x'].max(), df['y'].max()]) / num_grids)


# Create a grid by rounding off the x and y coordinates to the nearest multiple of grid_size
df['x_grid'] = np.floor(df['x'] / grid_size) * grid_size
df['y_grid'] = np.floor(df['y'] / grid_size) * grid_size

# Group the data by grid and count the number of points in each grid
counts = df.groupby(['x_grid', 'y_grid']).size().reset_index(name='count')

# Calculate the mean x and y coordinates for each grid
mean_coords = df.groupby(['x_grid', 'y_grid']).agg({'x': 'mean', 'y': 'mean'}).reset_index()

# Merge the count and mean coordinates dataframes
result = pd.merge(counts, mean_coords, on=['x_grid', 'y_grid'])

# Sort the data by the count of points in each grid in descending order
result = result.sort_values(by='count', ascending=False)
result = result.head(10)

arr = list(zip(result['x'], result['y']))
# Display the results
print(result)
print(arr)

cmap = 'YlOrRd'

# Create a 2D histogram for the grid points and plot it as a heatmap
fig, ax = plt.subplots(figsize=(7,7))


# Plot the original points
ax.scatter(df['x'], df['y'], s=2, alpha=0.5)

# Plot the mean x and y for each grid
ax.scatter(result['x'], result['y'], s=50, marker='x', color='red')

# Set the x and y axis labels
ax.set_xlabel('X Coordinates')
ax.set_ylabel('Y Coordinates')



# Show the plot
plt.show()




# # Plot the original points, the created grids, and the mean coordinates for each grid
# fig, ax = plt.subplots(figsize=(10, 10))

# # Plot the original points
# ax.scatter(df['x'], df['y'], s=10, alpha=0.5)

# # Plot the grids colored by counts
# ax.scatter(result['x_grid'], result['y_grid'], s=result['count'], c=result['count'], cmap=cmap, alpha=0.8)

# # Plot the mean coordinates for each grid
# for i in range(len(result)):
#     ax.annotate(f'{result.iloc[i]["x"]:.2f}, {result.iloc[i]["y"]:.2f}', 
#                 xy=(result.iloc[i]["x_grid"], result.iloc[i]["y_grid"]), 
#                 ha='center', va='center', color='black', fontsize=8)

# # Set the x and y axis labels
# ax.set_xlabel('X Coordinates')
# ax.set_ylabel('Y Coordinates')

# # Create a colorbar for the count
# cbar = fig.colorbar(ax.collections[1], ax=ax, label='Number of Points')

# # Show the plot
# plt.show()