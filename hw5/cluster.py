import pandas as pd
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans

# load data from csv file
df = pd.read_csv('xyz_lidar.csv')
df.columns = ('x','y','z')
print(df)

# set number of grid cells in x and y direction
num_x_cells = 5
num_y_cells = 5
look_radius = 5
k=4

# clip z co-ordiantes
df = df[(df['z'] >= -1) & (df['z'] <= 1)]

df = df[(df['x'] <= look_radius) & ((df['x'] >= (-1) * look_radius)) \
        & (df['y'] <= look_radius) & (df['y'] >= (-1) * look_radius)]

# create KMeans object with k clusters
kmeans = KMeans(n_clusters=k)

# fit the data
kmeans.fit(df)

# get cluster labels and centroids
labels = kmeans.labels_
centroids = kmeans.cluster_centers_

# plot the clusters
fig, ax = plt.subplots()
colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k']
for i in range(k):
    # plot data points of each cluster with a different color
    ax.scatter(df[labels == i]['x'], df[labels == i]['y'], c=colors[i%len(colors)],s=2)
    # plot centroid of each cluster
    ax.scatter(centroids[i][0], centroids[i][1], marker='x', s=500, c='black')
plt.show()


# import pandas as pd
# import numpy as np
# from sklearn.cluster import KMeans
# import matplotlib.pyplot as plt

# # Load the data from CSV file
# df = pd.read_csv('xyz_lidar.csv')
# df.columns = ('x','y','z')
# print(df)

# # set number of grid cells in x and y direction
# num_x_cells = 5
# num_y_cells = 5
# look_radius = 4

# # clip z co-ordiantes
# df = df[(df['z'] >= -1) & (df['z'] <= 1)]

# df = df[(df['x'] <= look_radius) & ((df['x'] >= (-1) * look_radius)) \
#         & (df['y'] <= look_radius) & (df['y'] >= (-1) * look_radius)]

# # Select x and y columns
# X = df[['x', 'y']]

# # Define the number of clusters
# k = 4

# # Create k-means model
# kmeans = KMeans(n_clusters=k)

# # Fit the model to the data
# kmeans.fit(X)

# # Add the cluster labels to the original data
# df['cluster'] = kmeans.labels_

# # Plot the clusters
# colors = ['r', 'g', 'b', 'y', 'c', 'm']
# fig, ax = plt.subplots()
# for i in range(k):
#     cluster = df[df['cluster'] == i]
#     ax.scatter(cluster['x'], cluster['y'], c=colors[i], label=f'Cluster {i+1}')
# ax.legend()
# plt.show()

