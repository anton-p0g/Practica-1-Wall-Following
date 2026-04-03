import time
from robotica import Coppelia, P3DX

BASE_SPEED = 0.5   
TARGET_DIST = 0.3
P_GAIN = 1.0
D_GAIN = 2.0

class RobotBrain:
    """Clase para manejar el estado y la lógica de un solo robot."""
    def __init__(self, robot_instance, name):
        self.robot = robot_instance
        self.name = name
        
        # Variables de estado individuales
        self.prev_error_left = 0.0
        self.prev_error_right = 0.0
        self.prev_state = "UNKNOWN"
        
        self.is_avoiding_front = False
        self.turn_left_to_avoid = False
        self.avoid_label = ""
        self.escape_counter = 0
        self.position_history = []

    def update(self):
        """Ciclo de percepción, decisión y acción para este robot."""
        # --- 1. Percepción ---
        sonar = self.robot.get_sonar()
        left_dist = min(sonar[0], sonar[1], sonar[2], sonar[15])
        front_dist = min(sonar[3], sonar[4])
        right_dist = min(sonar[5], sonar[6], sonar[7], sonar[8])
        rear_dist = min(sonar[8:])
        
        # Odometría para detección de atascos
        current_pos = self.robot.get_position()
        self.position_history.append(current_pos)
        if len(self.position_history) > 6:
            self.position_history.pop(0) # Mantener 0.6 segundos de historial
            
        if len(self.position_history) == 6 and self.escape_counter == 0:
            old_pos = self.position_history[0]
            dist_moved = ((current_pos[0] - old_pos[0])**2 + (current_pos[1] - old_pos[1])**2)**0.5
            
            # Choque detectado
            if dist_moved < 0.03 and self.prev_state not in ["AVOID_FRONT", "DEAD_END", "ESCAPE", "UNKNOWN"]:
                self.escape_counter = 8
                self.position_history.clear()

        # --- 2. Decisión (Máquina de Estados) ---
        state = "WANDER"
        left_speed = BASE_SPEED
        right_speed = BASE_SPEED

        # Histéresis para evasión frontal y callejones sin salida
        if not self.is_avoiding_front and front_dist < 0.3:
            self.is_avoiding_front = True
            
            if left_dist < 0.3 and right_dist < 0.3:
                self.avoid_label = "DEAD_END"
                self.turn_left_to_avoid = False 
            else:
                self.avoid_label = "AVOID_FRONT"
                self.turn_left_to_avoid = (left_dist > right_dist)
        elif self.is_avoiding_front and front_dist > 0.6:
            self.is_avoiding_front = False

        if self.escape_counter > 0:
            state = "ESCAPE"
            safe_reverse = max(-0.6, -1.5 * (rear_dist - 0.2))
            if rear_dist < 0.2: safe_reverse = 0.0
            
            if left_dist > right_dist:
                left_speed = safe_reverse - 0.4
                right_speed = safe_reverse + 0.4
            else:
                left_speed = safe_reverse + 0.4
                right_speed = safe_reverse - 0.4
                
            left_speed = max(-0.8, min(0.8, left_speed))
            right_speed = max(-0.8, min(0.8, right_speed))
            
            self.escape_counter -= 1
            self.is_avoiding_front = False
            
        elif min(sonar[2], sonar[3], sonar[4], sonar[5]) < 0.15:
            self.escape_counter = 4
            state = "ESCAPE"
            left_speed = -0.6
            right_speed = -0.6
            self.is_avoiding_front = False
            
        elif self.is_avoiding_front:
            state = self.avoid_label
            if self.turn_left_to_avoid:
                left_speed = -0.6
                right_speed = 0.6
            else:
                left_speed = 0.6
                right_speed = -0.6
                
        elif left_dist < 0.6:
            state = "FOLLOW_LEFT"
            error = left_dist - TARGET_DIST
            if self.prev_state != state: 
                self.prev_error_left = error 
            
            error_diff = error - self.prev_error_left
            self.prev_error_left = error
            
            diff = (error * P_GAIN) + (error_diff * D_GAIN)
            diff = min(diff, 0.15)
            left_speed = BASE_SPEED - diff
            right_speed = BASE_SPEED + diff
        
        elif self.prev_state in ["FOLLOW_LEFT", "WRAP_LEFT"] and left_dist >= 0.6:
            state = "WRAP_LEFT"
            left_speed = 0.1   
            right_speed = BASE_SPEED + 0.1
        
        elif right_dist < 0.6:
            state = "FOLLOW_RIGHT"
            error = right_dist - TARGET_DIST
            if self.prev_state != state: 
                self.prev_error_right = error 
            
            error_diff = error - self.prev_error_right
            self.prev_error_right = error
            
            diff = (error * P_GAIN) + (error_diff * D_GAIN)
            diff = min(diff, 0.15)
            left_speed = BASE_SPEED + diff
            right_speed = BASE_SPEED - diff
        
        elif self.prev_state in ["FOLLOW_RIGHT", "WRAP_RIGHT"] and right_dist >= 0.6:
            state = "WRAP_RIGHT"
            left_speed = BASE_SPEED + 0.1   
            right_speed = 0.1

        self.prev_state = state

        # --- 3. Acción ---
        self.robot.set_speed(left_speed, right_speed)
        
        return f"{self.name}: [{state:<12}]"


def main():
    print("Conectando a CoppeliaSim...")
    coppelia = Coppelia()
    
    # Instanciar los dos robots
    robot1_hardware = P3DX(coppelia.sim, 'PioneerP3DX[0]')
    robot2_hardware = P3DX(coppelia.sim, 'PioneerP3DX[1]') 

    # Crear los "cerebros" pasándoles el hardware
    brain1 = RobotBrain(robot1_hardware, "R1")
    brain2 = RobotBrain(robot2_hardware, "R2")

    coppelia.start_simulation()
    time.sleep(1)

    try:
        while coppelia.is_running():
            # Actualizamos ambos robots en cada iteración del bucle principal
            status1 = brain1.update()
            status2 = brain2.update()
            
            # Imprimimos el estado de ambos en la misma línea
            print(f"{status1} | {status2}                  ", end="\r")
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nInterrumpido por el usuario.")
    
    finally:
        print("\nLimpiando...")
        robot1_hardware.set_speed(0.0, 0.0)
        robot2_hardware.set_speed(0.0, 0.0)
        coppelia.stop_simulation()

if __name__ == '__main__':
    main()
