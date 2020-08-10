# ncrl_waffle_develop
##### Date : 2018 June
##### Author : Chun-Jung Lin
---
### Target
* using 3D lidar(VLP-16) to avoid the obstacles on an UGV.

### Method
* potenial method

### Node
1. `force_nd`
    * the node to compute the repulsive and visualize the points in ROI
	* **Input** : point cloud from VLP-16
	* **Output** : the repulsive force in XYZ-axis
2. `pf_nd`
    * the node to make the vehicle move to the destination without hitting any obstacles.
	* **Input** : the repulsive force from `force_nd`
	* **Output** : the velocity command in X-axis and angular velocity in Z-axis
3. `offb_ctrl`
    * the example node to control the waffle with the keyboard
	* **Output** : the velocity command in X-axis and angular velocity in Z-axis
