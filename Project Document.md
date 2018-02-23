;## Project: Search and Sample Return
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping** 

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./misc/rover_image.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg 

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  


### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.

```python
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    navigable = np.zeros_like(img[:,:,0])
    obstacle = np.zeros_like(img[:,:,0])
    rock = np.zeros_like(img[:,:,0])
    
    
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    # Navigable area is lighter, above the threshold. 
    navigable[above_thresh] = 1
    # Obstacle area is darker, below the threshold. 
    obstacle[np.invert(above_thresh)] = 1
    
    # Find the rock sample
    # define the RGB color range of the rock sample 
    rock_lower = np.array([20,90,100])
    rock_upper = np.array([30,255,255])
    img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    rock = cv2.inRange(img_hsv, rock_lower, rock_upper)
    
    # Return the binary image
    return navigable, obstacle, rock
```

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 

1. In `process_image(img)` function, the perspective of input image `img` was first transformed into top-down view image. `warped = perspect_transform(img, source, destination)`

2. Then in the second step, another function will identify the the navigable terrain, obstacles and rock samples based on their colors. `navigable, obstacle, rock = color_thresh(warped)`

3. Transform the coordinates into rover centric. `navigable_xpix, navigable_ypix = rover_coords(navigable)`

4. Transform the rover centric coordinates into the world coordinates
    ```python
        navigable_x_world, navigable_y_world = 
            pix_to_world(navigable_xpix, navigable_ypix, 
                        data.xpos[data.count], 
                        data.ypos[data.count], 
                        data.yaw[data.count],
                        data.worldmap.shape[0], 10)
    ```

5. Add it in the worldmap `data.worldmap[navigable_y_world, navigable_x_world] = [255, 255, 255]`


### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

##### Perception
In function `perception_step()` of script `perception.py`, following steps are modified in order to identify the vision from rover's camera.  
1. Transform the perspective to a top-down view, based on the `source` and `destination` points defined in the function. 
2. A `color_thresh()` function will process the input image, identify the obstacles, rock samples and navigable terrain based on their colors, dark, golden and bright. 
  The identified objects in the image is then marked in different color channel of `Rover.vision_image`
3. In this step the vision image is transformed into rover-centric coordinates. Based on this vision, the position and yaw of the rover, `pix_to_world()` will convert them to the pixels in the worldmap. 
  Eventually, it will be placed in the 3 color channels of worldmap view on the screen by setting the values of `Rover.worldmap`.  
4. In final step, `to_polor_coords()` converts the vision into polar coordinates. The position (distance and angle in polar coordinates) of navigable terrain and rock sample are updated in the `Rover` object.
  This allows the rover to make decisions on where to go and what rock samples to pick up.   
5. Update the Rover properties so it can be used in decision making step. 

##### Decision
The purpose function `decision_step()` is to make decisions about how the rover moves based on vision data identified by `perception_step()`. The decision process are mainly as follow:
 
 1. In *forward* mode, the function first detects if there is any rock in the vision.  
   1.1. If there is a rock sample, and the rover is close enough to the rock, i.e. the average distance, the average value of `Rover.sample_dists` is less than 24, then stop the rover and pick it up  
   1.2. If there is a rock sample but the rover is not close enough. Then the cover steers towards the rock.  
 
2. In *forward* mode, if there is no sample rock in the vision.   
  2.1. When there is enough navigable terrain, then continues to move forward. The key of this step is to determine the steer angle. In order to map as much as possible the world map, the strategy is to always drive close to the left edge of navigable terrain in the rover vision.   
  It first calculates the mean of all navigable angles `avg_nav_angle = np.mean(Rover.nav_angles)`, then filters the navigable terrain on the left side of average navigable angle `left_nav_angles = Rover.nav_angles[Rover.nav_angles >= avg_nav_angle]`.  Eventually the steer angle is the mean of all navigable terrain on the left side `steer_angle = np.clip(np.mean(left_nav_angles) * 180 / np.pi, -15, 15)`. This makes sure that the rover moves on the left side of navigable terrain.   
  2.2. If there is not enough navigable terrain in front of the rover. The Rover stops.   

3. When the rover in *stop* mode, there are mainly 2 possibilities:  
  3.1. It stopped because it finds a rock sample. In this case it starts the process of picking up the rock sample.  
  3.2. It stopped because it hits the obstacle. In this case it turns right for 15 degrees and if there is enough navigable terrain. It drives forward again.   


#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

##### The simulator settings 
* Screen resolution: 1024x768
* Graphics quality: Good
* FPS: 14

##### General approach  
* The strategy is to always drive the rover through the left side of navigable terrain so that it can map as much as possible the worldmap.
* When the rover sees a rock sample, it will slow down the speed and move towards the rock sample. Then it stops when close enough. 

##### Some techniques
In class *Rover*, properties are defined to track the status of the rover. 
For example 
* *Rover.sample_angles* and *Rover.sample_dists* are the sample pixels found in the perception step. 
* *Rover.found_sample* stores if the rover sees a rock sample. If it is true, it means there is a rock sample nearby. The rover will slow down and move towards the sample. 
* etc. 

Also in class *Rover*, several methods are defined to simplify the decision making code.
* *Rover.move()* and *Rover.stop()*, move the rover and stop the rover. 
* *Rover.calc_obstacle_dist()*, calculate the distance between the rover and obstacle in the rover vision. 
* *Rover.check_stuck()*, check if the rover is stuck somewhere. 

##### Results and how to improve
Generally the program can reach the target fidelity and mapped worldmap terrain. But here are some problems:  
* In many cases if the rover drive closely to some obstacle rocks, the rock may block the rover. But in rover's vision image, it does not see this rock when it get stuck or there are still enough navigable terrain. So the rover keeps trying to move forward. 
  The solution to this problem is a little bit difficult because in this simulator, the rover does not have extra sensors to detect the environment around it. The only perception sensor it can use is the camera.   
* Another problem is that when the rover sees the sample rock, it will move towards the sample directly, even if there are some obstacles between the rover and rock sample. 
  The solution requires more complicated algorithm to go around the obstacle, then approach the sample from different angle. 
* Last problem is that the fidelity might go very low in some cases, especially when it hits obstacles or it is stuck in some rocks. I may need to adjust the threshold to fix it. 

