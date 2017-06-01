import numpy as np
import cv2
import matplotlib.image as mpimg
import matplotlib.pyplot as plt

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    color_select_obst = np.zeros_like(img[:,:,0])
    
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    #obstacles
    
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    color_select_obst = np.invert(color_select)
    # Return the binary image
    return color_select, color_select_obst
def color_threshr(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    #hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #lower_blue = np.array([176,100,100])
    #upper_blue = np.array([176,100,40])
    lower = np.array([100,100,0])
    upper = np.array([255,255,85])
    mask = cv2.inRange(img, lower, upper)
    res = cv2.bitwise_and(img,img, mask = mask)
    above_thresh = (res[:,:,0] > 0) \
                & (res[:,:,1] > 0) \
                & (res[:,:,2] > 0)
   
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select
# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
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

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    # TODO:
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                 
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # TODO:
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
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    image = Rover.img
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    bottom_offset = 6
    dst_size = 3 
    if (Rover.pitch>0.5 and Rover.pitch<359.5) or (Rover.roll>1.2 and Rover.roll<358):
        source = np.float32([[14, 140], [301 ,140],[200, 82], [118, 82]])
        
    else:
        source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])
    # perform threshold on rocks
    threshrock= color_threshr(image)
    # perform perspective transform on the rock
    warpedrock = perspect_transform(threshrock, source, destination)
    # 2) Apply perspective transform
    warped = perspect_transform(image, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    if (Rover.pitch >3 and Rover.pitch < 355) or (Rover.roll>2 and Rover.roll<358):
        rgb_thresh=(240, 240, 240)
    else:
        rgb_thresh=(160, 160, 160)    
    threshed, obst = color_thresh(warped, rgb_thresh)
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,0] = obst
    Rover.vision_image[:,:,2] = threshed
    Rover.vision_image[:,:,1] = warpedrock
    #Rover.vision_image[:,:,0] = threshed.astype(np.uint8)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    
    # 5) Convert map image pixel values to rover-centric coords
    #navigable terrain
    xpix, ypix = rover_coords(threshed)
    # obstacles
    xpixo, ypixo = rover_coords(obst)
    #rock
    xpixr, ypixr = rover_coords(warpedrock)
    print("ROver Coords =", xpixr)
    print("ROver Coords y=", ypixr)
    # 6) Convert rover-centric pixel values to world coordinates
    #navigable
    ypos, xpos = Rover.pos
    yaw = Rover.yaw
    x_world, y_world = pix_to_world(xpix, ypix, xpos, 
                                ypos, yaw, 
                                Rover.ground_truth.shape[0], 15)
    #obstacles
    x_worldo, y_worldo = pix_to_world(xpixo, ypixo, xpos, 
                                ypos, yaw, 
                                Rover.ground_truth.shape[0], 15)
    #rock
    x_worldr, y_worldr = pix_to_world(xpixr, ypixr, xpos, 
                                ypos, yaw, 
                                Rover.ground_truth.shape[0], 15)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
    Rover.worldmap[x_world, y_world, 2] += 1
    Rover.worldmap[x_worldr, y_worldr, 1] += 1
    Rover.worldmap[x_worldo, y_worldo, 0] += 1
    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    Rover.nav_dists, Rover.nav_angles  = to_polar_coords(xpix, ypix)
    Rover.nav_distsr, Rover.nav_anglesr  = to_polar_coords(xpixr, ypixr)
    
 
    
    
    return Rover
