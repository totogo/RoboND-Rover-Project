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
        Rover.check_stuck()

        # Check for Rover.mode status
        if Rover.mode == 'forward':
            if Rover.stuck:
                Rover.stop()
                Rover.found_sample = False
            # Check if there is any rocks in the vision
            elif len(Rover.sample_angles) >= 5 or Rover.found_sample:
                if len(Rover.sample_angles) >= 5:
                    print('sample distance', np.min(Rover.sample_dists))

                # if Rover.near_sample:
                if len(Rover.sample_angles) >= 5:
                    Rover.found_sample = True
                    Rover.found_sample_dist = np.min(Rover.sample_dists)
                    Rover.found_sample_angle = \
                        np.mean(Rover.sample_angles[Rover.sample_dists == Rover.found_sample_dist])

                if Rover.found_sample_dist <= 10:
                    print('stopping near rock')
                    Rover.stop()
                else:
                    # Slowly approach the sample
                    if Rover.vel > 0.8:
                        Rover.brake = Rover.brake_set / 2
                    else:
                        print('moving to the sample')
                        # Set throttle value to throttle setting
                        Rover.move(angle=np.clip(Rover.found_sample_angle, Rover.min_nav_angle, Rover.max_nav_angle),
                                   vel=0.7)

            # Check the extent of navigable terrain
            elif len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good
                # and velocity is below max, then throttle

                # Set steering to average angle clipped to the range +/- 15
                avg_nav_angle = np.mean(Rover.nav_angles)
                left_nav_angles = Rover.nav_angles[Rover.nav_angles > avg_nav_angle]
                right_nav_angles = Rover.nav_angles[Rover.nav_angles < avg_nav_angle]

                avg_left_angle = np.mean(left_nav_angles)
                avg_right_angle = np.mean(right_nav_angles)
                # if Rover.calc_nav_terrains(avg_right_angle) > Rover.calc_nav_terrains(avg_nav_angle):
                #     if Rover.calc_nav_terrains(avg_right_angle) > Rover.calc_nav_terrains(avg_left_angle):
                #         steer_angle = np.clip(avg_right_angle, -15, 15)
                #     else:
                #         steer_angle = np.clip(avg_left_angle, -15, 15)
                # else:
                steer_angle = np.clip(avg_nav_angle, -15, 15)

                steer_vel = Rover.max_vel

                Rover.move(steer_angle, steer_vel)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            # elif len(Rover.nav_angles) < Rover.stop_forward
            else:
                Rover.stop()

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            print('sample angle', np.mean(Rover.sample_angles))
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
                        if np.mean(Rover.sample_dists) > 5:
                            Rover.throttle = Rover.throttle_set
                            Rover.mode = 'forward'
                        else:
                            Rover.throttle = 0
                        Rover.brake = 0
                        Rover.steer = np.clip(np.mean(Rover.sample_angles), -15, 15)
                else:
                    # Now we're stopped and we have vision data to see if there's a path forward
                    if len(Rover.nav_angles) < Rover.go_forward or Rover.stuck:
                        # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                        # TODO: Could be more clever here about which way to turn
                        Rover.move(angle=-15, vel=0)
                        Rover.stuck = False
                        Rover.start_stuck_time = None
                    # If we're stopped but see sufficient navigable terrain in front then go!
                    elif len(Rover.nav_angles) >= Rover.go_forward:
                        # Set steer to mean angle
                        Rover.move(angle=np.clip(np.mean(Rover.nav_angles), -15, 15),
                                   vel=Rover.max_vel)
                        Rover.mode = 'forward'
                        Rover.stuck = False
                        Rover.start_stuck_time = None

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

