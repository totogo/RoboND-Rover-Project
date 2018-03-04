import numpy as np
import cv2


# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(150, 150, 150)):
    # Create an array of zeros same xy size as img, but single channel
    navigable = np.zeros_like(img[:, :, 0])
    obstacle = np.zeros_like(img[:, :, 0])
    rock = np.zeros_like(img[:, :, 0])
    
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:, :, 0] > rgb_thresh[0]) \
                & (img[:, :, 1] > rgb_thresh[1]) \
                & (img[:, :, 2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    navigable[above_thresh] = 1
    obstacle[np.invert(above_thresh)] = 1
    
    # Find the rock
    rock_lower = np.array([20, 90, 100])
    rock_upper = np.array([30, 255, 255])
    img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    rock = cv2.inRange(img_hsv, rock_lower, rock_upper)
    # Return the binary image
    return navigable, obstacle, rock


# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles


# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated


def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world


# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))  # keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5
    bottom_offset = 6
    source = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
                      [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                      ])
    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    navigable, obstacle, rock = color_thresh(warped)
    
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:, :, 0] = obstacle * 100
    Rover.vision_image[:, :, 1] = rock * 100
    Rover.vision_image[:, :, 2] = navigable * 100

    # 5) Convert map image pixel values to rover-centric coords
    obstacle_xpix, obstacle_ypix = rover_coords(obstacle)
    rock_xpix, rock_ypix = rover_coords(rock)
    navigable_xpix, navigable_ypix = rover_coords(navigable)

    # 6) Convert rover-centric pixel values to world coordinates
    navigable_x_world, navigable_y_world = pix_to_world(navigable_xpix, navigable_ypix, 
                                                        Rover.pos[0], Rover.pos[1], Rover.yaw,
                                                        Rover.worldmap.shape[0], 10)
    obstacle_x_world, obstacle_y_world = pix_to_world(obstacle_xpix, obstacle_ypix, 
                                                      Rover.pos[0], Rover.pos[1], Rover.yaw, 
                                                      Rover.worldmap.shape[0], 10)
    rock_x_world, rock_y_world = pix_to_world(rock_xpix, rock_ypix, 
                                              Rover.pos[0], Rover.pos[1], Rover.yaw, 
                                              Rover.worldmap.shape[0], 10)
    
    # 7) Update Rover worldmap (to be displayed on right side of screen)
    if abs(Rover.roll) <= 1 and abs(Rover.pitch) <= 1:
        Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(navigable_xpix, navigable_ypix)
    Rover.nav_angles = Rover.nav_angles * 180 / np.pi
    if len(Rover.nav_angles) > 1:
        Rover.max_nav_angle = np.max(Rover.nav_angles)
        Rover.min_nav_angle = np.min(Rover.nav_angles)
    else:
        Rover.max_nav_angle = 0
        Rover.min_nav_angle = 0
    print('max nav angle', Rover.max_nav_angle)
    print('min nav angle', Rover.min_nav_angle)

    Rover.obstacle_dists, Rover.obstacle_angles = to_polar_coords(obstacle_xpix, obstacle_ypix)
    Rover.obstacle_angles = Rover.obstacle_angles * 180 / np.pi
    # 9) Update sample info
    Rover.sample_dists, Rover.sample_angles = to_polar_coords(rock_xpix, rock_ypix)
    Rover.sample_angles = Rover.sample_angles * 180 / np.pi
    # print(to_polar_coords(rock_xpix, rock_ypix))

    return Rover