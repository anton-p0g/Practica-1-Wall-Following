import numpy as np
import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class Coppelia():

    def __init__(self):
        print('*** connecting to coppeliasim')
        client = RemoteAPIClient()
        self.sim = client.getObject('sim')

    def start_simulation(self):
        self.default_idle_fps = self.sim.getInt32Param(self.sim.intparam_idle_fps)
        self.sim.setInt32Param(self.sim.intparam_idle_fps, 0)
        self.sim.startSimulation()

    def stop_simulation(self):
        self.sim.stopSimulation()
        while self.sim.getSimulationState() != self.sim.simulation_stopped:
            time.sleep(0.1)
        self.sim.setInt32Param(self.sim.intparam_idle_fps, self.default_idle_fps)
        print('*** done')

    def is_running(self):
        return self.sim.getSimulationState() != self.sim.simulation_stopped


class P3DX():
    num_sonar = 16
    sonar_max = 1.0

    def __init__(self, sim, robot_id):
        self.sim = sim
        print('*** getting handles', robot_id)
        self.robot_base = self.sim.getObject(f'/{robot_id}')
        self.left_motor = self.sim.getObject(f'/{robot_id}/leftMotor')
        self.right_motor = self.sim.getObject(f'/{robot_id}/rightMotor')
        self.sonar = []
        for i in range(self.num_sonar):
            self.sonar.append(self.sim.getObject(f'/{robot_id}/ultrasonicSensor[{i}]'))


    def get_sonar(self):
        readings = []
        for i in range(self.num_sonar):
            res,dist,_,_,_ = self.sim.readProximitySensor(self.sonar[i])
            readings.append(dist if res == 1 else self.sonar_max)
        return readings

    def set_speed(self, left_speed, right_speed):
        self.sim.setJointTargetVelocity(self.left_motor, left_speed)
        self.sim.setJointTargetVelocity(self.right_motor, right_speed)

    def get_position(self):
        # Returns absolute [x, y, z] coordinates of the robot in the world
        return self.sim.getObjectPosition(self.robot_base, -1)


def main(args=None):
    coppelia = Coppelia()
    robot = P3DX(coppelia.sim, 'PioneerP3DX')
    robot.set_speed(+1.2, -1.2)
    coppelia.start_simulation()
    while (t := coppelia.sim.getSimulationTime()) < 3:
        print(f'Simulation time: {t:.3f} [s]')
    coppelia.stop_simulation()


if __name__ == '__main__':
    main()
