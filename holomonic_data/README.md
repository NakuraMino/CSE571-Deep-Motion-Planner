# My README:
EDIT: I realized no one else can run this code because I didn't upload the AStarPlanner or MapEnvironment code onto github.Let me know if you want me to change anything!

createData.py is the file used to generate data. run

```
python createData.py
```

If you want to change the number of maps used to generate data, change the number on line 88 to whatever you want to:

```
if image_num == 10:
```

Under images/, you will see random croppings of images. Here's a quick rundown:
- They were originally 500x500 images, resized to be 128x128.
- the amount of free space should be at least half the image. 
  - If that wasn't possible, the free space should be > 33%, > 25% etc until it became possible 
  - (This might bias our results, so I can change if need be)

data.csv contains:
curr_y, curr_x, goal_y, goal_x, map_file_path, action

Here are a few things to note about the data file:
- for each A* path generated, I create len(path) data points, with each point along the path used as start_y, start_x
  - I'm not too confident this will give us good results, so I plan on asking a TA what they think about it
- for some reason, all the files we got use the 0th index for the y value and 1st index as the x value, so the csv file reflects this too
- action is a value 1-8 specifying which direction the robot moved in (the robot has 8 possible actions)
- each number maps to a next state: (sorry markdown gives me weird formatting but hopefully you get the point) 

| 3 | 5 | 8 |
|---|---|---|
| 2 | x | 7 |
| 1 | 4 | 6 |


## TODO:

- generate full training dataset
- also generate test dataset too!

# Original README: 

## Map representation

We use the standard cartesian coordinate system (x-axis points right and y-axis points upwards). 
Unit length corresponds to one meter.
     
### Related files  
Each map consists of two files:
- `floor_trav_0_v2.png`. This is an image representing the occupancy grid. Note that the image is vertically flipped so 
  what you see in plots will be a flipped version of this image.
- `floorplan.yaml` contains metadata of the map
  - `image` filename of the image.
  - `origin` a 2-tuple [x, y] representing the origin w.r.t to the image coordinate system. It is set to [0, 0] for all maps.
    This means that the top left corner of the image corresponds to the origin of the global coordinate system.
  - `resolution` the metric width of one pixel. For example, 0.01 means that a pixel represents a 1cm<sup>2</sup> square.

## Utilities
`map_utils.py` contains utilities for handling occupancy maps.
- `Map`. Use this class to load maps, transform coordinates and get laser scan data.
- `Visualizer`. Use this class to visualize maps and laser scans.

You can run `python map_utils.py` get an idea of how these utilities work.

Feel free to modify the code to make it tailor to your project.
