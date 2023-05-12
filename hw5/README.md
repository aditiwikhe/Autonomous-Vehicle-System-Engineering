## Using Lidar to position the vehicle.

For this HW, we used lidar to get the position of the boxes. We divided the area around the car into grids and counted the number of points in each grid. Assumption being the two most populous grids are the ones with boxes contained within the grid.
Once we have identified the grids, we take the mean_x and mean_y of all the points in the grid as the co-orinates of the boxes. We find the mid-point of the boxes.
To move the car towards the midpoint, we modified the pure pursuit from HW4 to meet our requirements. 
Once midpoint is reached we make the car follow a circle with radius equal to distance between midpoint and one of the boxes.
We make the car stop once it reaches the midpoint again.
