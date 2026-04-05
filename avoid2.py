import time
from robotica import Coppelia, P3DX

BASE_SPEED = 0.5   
TARGET_DIST = 0.3
P_GAIN = 1.0
D_GAIN = 2.0

class WallFollowerController:
    """Clase que encapsula la memoria y la lógica de control para un solo robot."""
    def __init__(self, robot, name):
        self.robot = robot
        self.name = name
        
        # Estado y memoria independiente para este robot
        self.prev_error_left = 0.0
        self.prev_error_right = 0.0
        self.prev_state = "UNKNOWN"
        self.is_avoiding_front = False
        self.turn_left_to_avoid = False
        self.avoid_label = ""
        self.reverse_counter = 0
        self.turn_counter = 0
        self.wrap_counter = 0
        self.position_history = []

    def update(self):
        """Ejecuta un paso de la máquina de estados y actualiza las velocidades."""
        # Percepción
        sonar = self.robot.get_sonar()
        left_dist = min(sonar[0], sonar[1], sonar[2], sonar[15])
        front_dist = min(sonar[3], sonar[4])
        right_dist = min(sonar[5], sonar[6], sonar[7], sonar[8])
        
        # Odometría para detección de atascos
        current_pos = self.robot.get_position()
        
        if self.reverse_counter == 0 and self.turn_counter == 0:
            self.position_history.append(current_pos)
            if len(self.position_history) > 6:
                self.position_history.pop(0)
                
            if len(self.position_history) == 6:
                old_pos = self.position_history[0]
                dist_moved = ((current_pos[0] - old_pos[0])**2 + (current_pos[1] - old_pos[1])**2)**0.5
                
                if dist_moved < 0.03 and self.prev_state not in ["AVOID_FRONT", "DEAD_END", "REVERSE", "ESCAPE_TURN", "UNKNOWN"]:
                    self.reverse_counter = 5 
                    self.turn_counter = 8     
                    self.position_history.clear()
        else:
            self.position_history.clear()

        # Velocidades por defecto (Wander)
        state = "WANDER"
        left_speed = BASE_SPEED
        right_speed = BASE_SPEED

        # Histéresis para evasión frontal
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

        # Evaluación de la Máquina de Estados Finita (FSM)
        if self.reverse_counter > 0:
            state = "REVERSE"
            left_speed = -0.6
            right_speed = -0.6
            self.reverse_counter -= 1
            
        elif self.turn_counter > 0:
            state = "ESCAPE_TURN"
            if left_dist > right_dist:
                left_speed = -0.6
                right_speed = 0.6
            else:
                left_speed = 0.6
                right_speed = -0.6
            self.turn_counter -= 1
            self.is_avoiding_front = False
            
        elif min(sonar[2], sonar[3], sonar[4], sonar[5]) < 0.15:
            self.reverse_counter = 4
            state = "REVERSE"
            left_speed = -0.6
            right_speed = -0.6
            
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
            if self.prev_state == "FOLLOW_LEFT":
                self.wrap_counter = 15
            
            if self.wrap_counter > 0:
                state = "WRAP_LEFT"
                left_speed = 0.1   
                right_speed = BASE_SPEED 
                self.wrap_counter -= 1
            else:
                state = "WANDER"
                left_speed = BASE_SPEED
                right_speed = BASE_SPEED
        
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
            if self.prev_state == "FOLLOW_RIGHT":
                self.wrap_counter = 15
            
            if self.wrap_counter > 0:
                state = "WRAP_RIGHT"
                left_speed = BASE_SPEED   
                right_speed = 0.1
                self.wrap_counter -= 1
            else:
                state = "WANDER"
                left_speed = BASE_SPEED
                right_speed = BASE_SPEED

        if state not in ["REVERSE", "ESCAPE_TURN"]:
            self.prev_state = state

        self.robot.set_speed(left_speed, right_speed)
        
        return f"{self.name}[{state:<12}]"


def main():
    print("Conectando a CoppeliaSim...")
    coppelia = Coppelia()
    
    # Inicializar ambos robots
    robot1 = P3DX(coppelia.sim, 'PioneerP3DX[0]')
    robot2 = P3DX(coppelia.sim, 'PioneerP3DX[1]')

    # Crear los controladores independientes
    controller1 = WallFollowerController(robot1, "R1")
    controller2 = WallFollowerController(robot2, "R2")

    coppelia.start_simulation()
    time.sleep(1)

    try:
        while coppelia.is_running():
            # Actualizar lógica para ambos robots
            log1 = controller1.update()
            log2 = controller2.update()
            
            print(f"{log1} | {log2}        ", end="\r")
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nInterrumpido por el usuario.")
    
    finally:
        print("\nLimpiando...")
        robot1.set_speed(0.0, 0.0)
        robot2.set_speed(0.0, 0.0)
        coppelia.stop_simulation()

if __name__ == '__main__':
    main()