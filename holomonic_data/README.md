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
