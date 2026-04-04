import time
from robotica import Coppelia, P3DX

BASE_SPEED = 0.5   
TARGET_DIST = 0.3
P_GAIN = 1.0
D_GAIN = 2.0


def main():
    print("Connecting to CoppeliaSim...")
    coppelia = Coppelia()
    robot = P3DX(coppelia.sim, 'PioneerP3DX')

    coppelia.start_simulation()
    time.sleep(1)

    try:
        prev_error_left = 0.0
        prev_error_right = 0.0
        prev_state = "UNKNOWN"
        
        is_avoiding_front = False
        turn_left_to_avoid = False
        avoid_label = ""
        reverse_counter = 0
        turn_counter = 0
        wrap_counter = 0
        position_history = []
        
        while coppelia.is_running():
            # Perception
            sonar = robot.get_sonar()
            left_dist = min(sonar[0], sonar[1], sonar[2], sonar[15])
            front_dist = min(sonar[3], sonar[4])
            right_dist = min(sonar[5], sonar[6], sonar[7], sonar[8])
            rear_dist = min(sonar[8:])
            
            # Odometry for Stall Detection
            current_pos = robot.get_position()
            
            # SOLO guardamos historial si NO estamos ejecutando una maniobra de escape
            if reverse_counter == 0 and turn_counter == 0:
                position_history.append(current_pos)
                if len(position_history) > 6:
                    position_history.pop(0) # Keep 0.6 seconds of history
                    
                if len(position_history) == 6:
                    old_pos = position_history[0]
                    dist_moved = ((current_pos[0] - old_pos[0])**2 + (current_pos[1] - old_pos[1])**2)**0.5
                    
                    # Crash detected
                    if dist_moved < 0.03 and prev_state not in ["AVOID_FRONT", "DEAD_END", "REVERSE", "ESCAPE_TURN", "UNKNOWN"]:
                        reverse_counter = 5  # Back straight up gracefully
                        turn_counter = 8     # Aumentado un poco para asegurar que no mire a la escalera
                        position_history.clear()
            else:
                # Si estamos escapando, vaciamos el historial para que al terminar 
                # empiece a contar desde cero con la nueva trayectoria limpia.
                position_history.clear()

            # Default to wander speeds
            state = "WANDER"
            left_speed = BASE_SPEED
            right_speed = BASE_SPEED

            # Hysteresis for Front Avoidance & Dead Ends
            if not is_avoiding_front and front_dist < 0.3:
                is_avoiding_front = True
                
                # Check if it's a dead end before picking turn direction
                if left_dist < 0.3 and right_dist < 0.3:
                    avoid_label = "DEAD_END"
                    turn_left_to_avoid = False  # U-turn right
                else:
                    avoid_label = "AVOID_FRONT"
                    turn_left_to_avoid = (left_dist > right_dist)
            elif is_avoiding_front and front_dist > 0.6:
                is_avoiding_front = False

            # Finite State Machine Evaluation
            if reverse_counter > 0:
                state = "REVERSE"
                left_speed = -0.6
                right_speed = -0.6
                reverse_counter -= 1
                
            elif turn_counter > 0:
                state = "ESCAPE_TURN"
                # Pure spin in place
                if left_dist > right_dist:
                    left_speed = -0.6
                    right_speed = 0.6
                else:
                    left_speed = 0.6
                    right_speed = -0.6
                turn_counter -= 1
                is_avoiding_front = False
                
            elif min(sonar[2], sonar[3], sonar[4], sonar[5]) < 0.15:
                # Emergency Bumper: Creates breathing room for tight spaces (preserves memory variables!)
                reverse_counter = 4
                state = "REVERSE"
                left_speed = -0.6
                right_speed = -0.6
                
            elif is_avoiding_front:
                state = avoid_label
                if turn_left_to_avoid:
                    left_speed = -0.6
                    right_speed = 0.6
                else:
                    left_speed = 0.6
                    right_speed = -0.6
                    
            elif left_dist < 0.6:
                state = "FOLLOW_LEFT"
                error = left_dist - TARGET_DIST
                if prev_state != state: 
                    prev_error_left = error 
                
                error_diff = error - prev_error_left
                prev_error_left = error
                
                diff = (error * P_GAIN) + (error_diff * D_GAIN)
                
                diff = min(diff, 0.15)
                left_speed = BASE_SPEED - diff
                right_speed = BASE_SPEED + diff
            
            elif prev_state in ["FOLLOW_LEFT", "WRAP_LEFT"] and left_dist >= 0.6:
                if prev_state == "FOLLOW_LEFT":
                    wrap_counter = 15  # Timeout: 1.5s * 1.05 rad/s = 90 degrees
                
                if wrap_counter > 0:
                    state = "WRAP_LEFT"
                    # Turn radius R=0.3m matching TARGET_DIST! No overshoot.
                    left_speed = 0.1   
                    right_speed = BASE_SPEED 
                    wrap_counter -= 1
                else:
                    # No wall exists here, go straight to escape the loop.
                    state = "WANDER"
                    left_speed = BASE_SPEED
                    right_speed = BASE_SPEED
            
            elif right_dist < 0.6:
                state = "FOLLOW_RIGHT"
                error = right_dist - TARGET_DIST
                if prev_state != state: 
                    prev_error_right = error 
                
                error_diff = error - prev_error_right
                prev_error_right = error
                
                diff = (error * P_GAIN) + (error_diff * D_GAIN)
                
                # Prevent orbiting table legs by capping inward "wall-seeking" turns
                diff = min(diff, 0.15)
                left_speed = BASE_SPEED + diff
                right_speed = BASE_SPEED - diff
            
            elif prev_state in ["FOLLOW_RIGHT", "WRAP_RIGHT"] and right_dist >= 0.6:
                if prev_state == "FOLLOW_RIGHT":
                    wrap_counter = 15  # 90-degree convex corner wrap timeout
                
                if wrap_counter > 0:
                    state = "WRAP_RIGHT"
                    # Turn radius R=0.3m matching TARGET_DIST! No overshoot.
                    left_speed = BASE_SPEED   
                    right_speed = 0.1
                    wrap_counter -= 1
                else:
                    state = "WANDER"
                    left_speed = BASE_SPEED
                    right_speed = BASE_SPEED

            # Prevent reflexive states from shattering FSM state tracking!
            if state not in ["REVERSE", "ESCAPE_TURN"]:
                prev_state = state

            robot.set_speed(left_speed, right_speed)
            print(f"[{state:<12}] F:{front_dist:.2f} L:{left_dist:.2f} R:{right_dist:.2f} | M:({left_speed:+.2f}, {right_speed:+.2f})         ", end="\r")
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    
    finally:
        print("\nCleaning up...")
        robot.set_speed(0.0, 0.0)
        coppelia.stop_simulation()

if __name__ == '__main__':
    main()
