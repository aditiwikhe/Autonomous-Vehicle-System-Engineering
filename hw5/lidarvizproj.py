import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# load csv file containing x and y coordinates
df = pd.read_csv('xyz_lidar.csv')
df.columns = ('x','y','z')
print(df)


# set number of grid cells in x and y direction
num_x_cells = 20
num_y_cells = 20

# determine grid cell size based on range of x and y coordinates
x_range = df['x'].max() - df['x'].min()
y_range = df['y'].max() - df['y'].min()
cell_size_x = x_range / num_x_cells
cell_size_y = y_range / num_y_cells

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
