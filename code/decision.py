import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    print('Rover mode:', Rover.mode)
    if Rover.nav_angles is not None:
        obstacle_dist = Rover.calc_obstacle_dist()
        Rover.check_stuck()

        if Rover.mode == 'start':
            if Rover.total_time >= 15:
                Rover.mode = 'forward'

        # Check for Rover.mode status
        if Rover.mode == 'forward' or Rover.mode == 'start':
            if Rover.stuck:
                Rover.stop()
                return Rover
            # Check if there is any rocks in the vision
            if len(Rover.sample_angles) >= 10 or Rover.found_sample:
                print('sample angles', Rover.sample_angles)
                print('sample distance', np.mean(Rover.sample_dists))
                # if Rover.near_sample:
                if len(Rover.sample_angles) >= 10:
                    Rover.found_sample = True
                    Rover.found_sample_dist = np.mean(Rover.sample_dists)
                    Rover.found_sample_angle = np.mean(Rover.sample_angles) * 180 / np.pi

                if Rover.found_sample_dist <= 15:
                    print('stopping near rock')
                    Rover.stop()
                else:
                    # Slowly approach the sample
                    if Rover.vel > 0.8:
                        Rover.brake = Rover.brake_set / 2
                    else:
                        print('moving to the sample')
                        # Set throttle value to throttle setting
                        Rover.move(angle=np.clip(Rover.found_sample_angle, -5, 5),
                                   vel=0.6)

            # Check the extent of navigable terrain
            elif len(Rover.nav_angles) >= Rover.stop_forward and obstacle_dist >= Rover.go_forward_dist:
                # If mode is forward, navigable terrain looks good
                # and velocity is below max, then throttle

                # Set steering to average angle clipped to the range +/- 15
                avg_nav_angle = np.mean(Rover.nav_angles)
                max_nav_angle = np.max(Rover.nav_angles)

                left_nav_angles = Rover.nav_angles[Rover.nav_angles >= avg_nav_angle]
                # left_nav_dists = Rover.nav_angles
                if Rover.mode == 'start':
                    # steer_angle = np.clip(avg_nav_angle * 180 / np.pi, -15, 15)
                    steer_angle = 0
                    steer_vel = 1.5
                elif Rover.mode == 'forward':
                    if len(left_nav_angles) >= Rover.stop_forward:
                        steer_angle = np.clip(np.mean(left_nav_angles) * 180 / np.pi, -10, 10)
                    else:
                        steer_angle = np.clip(avg_nav_angle * 180 / np.pi, -15, 15)
                    steer_vel = Rover.max_vel

                Rover.move(steer_angle, steer_vel)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            # elif len(Rover.nav_angles) < Rover.stop_forward
            else:
                Rover.stop()

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            print('sample angle', np.mean(Rover.sample_angles * 180 / np.pi))
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.stop()
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Stopped, see if there is any rock
                if len(Rover.sample_angles) > 2 and not Rover.stuck:
                    print('near sample, distance', np.mean(Rover.sample_dists))
                    # It should be very close already, maybe the angle is not right, turn a little bit
                    if not Rover.near_sample and not Rover.picking_up:
                        Rover.throttle = 0
                        Rover.brake = 0
                        Rover.steer = np.clip(np.mean(Rover.sample_angles * 180 / np.pi), -5, 5)
                else:
                    # Now we're stopped and we have vision data to see if there's a path forward
                    if len(Rover.nav_angles) < Rover.go_forward or obstacle_dist < Rover.go_forward_dist or Rover.stuck:
                        # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                        # TODO: Could be more clever here about which way to turn
                        Rover.move(angle=-15, vel=0)
                        Rover.stuck = False
                        Rover.start_stuck_time = None
                    # If we're stopped but see sufficient navigable terrain in front then go!
                    elif len(Rover.nav_angles) >= Rover.go_forward and obstacle_dist >= Rover.go_forward_dist:
                        # Set steer to mean angle
                        Rover.move(angle=np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15),
                                   vel=Rover.max_vel)
                        Rover.mode = 'forward'

    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        # Reset
        Rover.found_sample = False
        Rover.found_sample_angle = None
        Rover.found_sample_dist = None
    
    return Rover

