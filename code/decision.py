import numpy as np
import time


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    rock_world_pos = Rover.worldmap[:,:,1].nonzero()
##    if Rover.near_sample == 0:
##        Rover.send_pickup = False
##        Rover.picking_up = 0
##    
##      if not Rover.nav_angles is None:
##        print("rover mode", Rover.mode )
##        if rock_world_pos[0].any():
##            Rover.mode = 'rocks'
        
        
        # Check for Rover.mode status
##        if Rover.mode == 'rocks':
##            if Rover.vel < Rover.max_vel:
##                # Set throttle value to throttle setting
##                Rover.throttle = Rover.throttle_set
##            else: # Else coast
##                Rover.throttle = 0
##                Rover.brake = 0
##                # Set steering to average angle clipped to the range +/- 15
##                Rover.steer = np.clip(np.mean(Rover.nav_anglesr*2 * 180/np.pi), -15, 15)

    if Rover.mode == 'forward': #or not rock_world_pos[0].any() 
        if (Rover.vel < 0.01 and Rover.vel>-0.01) and 0.4<Rover.throttle<0:
            Rover.throttle = -0.9
            Rover.brakes = 10
            Rover.throttle = 0
            
            Rover.steer = 15
            Rover.brakes=0
            
        else:
            if Rover.vel < 0.1 and Rover.throttle > 0.1:
                Rover.mode = 'stop'
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                    
                       
                else: # Else coast
                    Rover.throttle = 0
                    Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles/2 * 180/np.pi), -15, 12)
                           # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'
           
                    

        # If we're already in "stop" mode then make different decisions
    elif Rover.mode == 'stop':
##            if rock_world_pos[0].any():
##                print("sample visible", rock_world_pos[0].any() )
##                Rover.mode = 'rocks'
          
            # If we're in stop mode but still moving keep braking
        if Rover.vel > 0.2:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0
        # If we're not moving (vel < 0.2) then do something else
        elif Rover.vel <= 0.2:
            # Now we're stopped and we have vision data to see if there's a path forward
            


            if len(Rover.nav_angles) < Rover.go_forward:
                Rover.throttle = 0
                # Release the brake to allow turning
                Rover.brake = 0
                # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                if np.mean(Rover.nav_angles * 180/np.pi) > 180 :
                    Rover.steer = -15 # Could be more clever here about which way to turn
                else:
                    Rover.steer = 15
            # If we're stopped but see sufficient navigable terrain in front then go!
            
            

            if len(Rover.nav_angles) >= Rover.go_forward:
                            
                # Set throttle back to stored value
                Rover.throttle = Rover.throttle_set
                # Release the brake
                Rover.brake = 0
                # Set steer to mean angle
                Rover.steer = np.clip(np.mean(Rover.nav_angles/2 * 180/np.pi), -15, 12)
                Rover.mode = 'forward'
##            if Rover.vel < 0.03  and Rover.throttle > 0.4 and (np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15) > -10 or np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15) < 10):
##                Rover.throttle = 0
##                
##                if np.mean(Rover.nav_angles * 180/np.pi) > 180 :
##                    Rover.steer = -15
##                else:
##                    Rover.steer = 15
    # Just to make the rover do something 
    # even if no modifications have been made to the code
        
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    return Rover

