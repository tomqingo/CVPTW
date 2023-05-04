The first line contains the following data:
	vehicle capacity

The remaining lines contain the data of each node.
For each node, the line contains the following data:
	i x y d b e s
where
	i = node ID (0: represents the depot; the others are customer IDs) 
	x = x coordinate
	y = y coordinate
	d = demand
	b = start of the pre-specified time period
	e = end of the pre-specified time period
	s = duration of the visit

REMARKS
-	The end of the pre-specified time period of node 0 is the deadline.
-	The Euclidian distance is used.
-	The travel time is assumed to be proportional to the distance between two nodes.
