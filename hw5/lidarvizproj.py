import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import colors

# load csv file containing x and y coordinates
df = pd.read_csv('xyz_lidar.csv')
df.columns = ('x','y','z')
print(df)

# set number of grid cells in x and y direction
num_x_cells = 5
num_y_cells = 5
look_radius = 4

# clip z co-ordiantes
df = df[(df['z'] >= -1) & (df['z'] <= 1)]

df = df[(df['x'] <= look_radius) & ((df['x'] >= (-1) * look_radius)) \
        & (df['y'] <= look_radius) & (df['y'] >= (-1) * look_radius)]

# determine grid cell size based on range of x and y coordinates
x_range = df['x'].max() - df['x'].min()
y_range = df['y'].max() - df['y'].min()
print(x_range, y_range)
cell_size_x = x_range / num_x_cells
cell_size_y = y_range / num_y_cells
print(cell_size_x, cell_size_y)
# create grid
x_coords = np.linspace(df['x'].min(), df['x'].max(), num=num_x_cells+1)
y_coords = np.linspace(df['y'].min(), df['y'].max(), num=num_y_cells+1)


# count number of points in each grid cell
counts = np.zeros((num_x_cells, num_y_cells))
for i in range(num_x_cells):
    for j in range(num_y_cells):
        x_min = x_coords[i]
        x_max = x_coords[i+1]
        y_min = y_coords[j]
        y_max = y_coords[j+1]
        counts[i,j] = ((df['x'] >= x_min) & (df['x'] < x_max) & 
                       (df['y'] >= y_min) & (df['y'] < y_max)).sum()

print(counts.shape)

x = np.repeat(np.arange(num_x_cells), num_x_cells)
y = np.tile(np.arange(num_y_cells), num_y_cells)

# create dataframe with x, y, and values as columns
counts_df = pd.DataFrame({'x': x, 'y': y, 'values': counts.flatten()})
counts_df = counts_df.sort_values(by='values', ascending=False)
counts_df['real_x'] = ((counts_df['x'] - 1) * cell_size_x + counts_df['x'] * cell_size_x) / 2
counts_df['real_y'] = ((counts_df['y'] - 1) * cell_size_y + counts_df['y'] * cell_size_y) / 2
print(counts_df)


# plot coordinates and grid
fig, ax = plt.subplots(figsize=(8,8))
ax.scatter(df['x'], df['y'], s=2)
for i in range(num_x_cells+1):
    ax.axvline(x=x_coords[i], color='green', linewidth=0.5)
for i in range(num_y_cells+1):
    ax.axhline(y=y_coords[i], color='green', linewidth=0.5)
plt.xlim(df['x'].min()-0.1*x_range, df['x'].max()+0.1*x_range)
plt.ylim(df['y'].min()-0.1*y_range, df['y'].max()+0.1*y_range)
plt.title('Grid Count')
plt.xlabel('X')
plt.ylabel('Y')



# print counts for each grid cell
for i in range(num_x_cells):
    for j in range(num_y_cells):
        ax.text(x_coords[i]+0.5*cell_size_x, y_coords[j]+0.5*cell_size_y, 
                int(counts[i,j]), ha='center', va='center')

plt.show()
