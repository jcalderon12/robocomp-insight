import pybullet as p
import json
import argparse
import time

class CausesSimulator:

    def __init__(self):
        # ================ PYBULLET SIMULATION SETUP  ================
        # ============================================================

        self.physicsClient = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setGravity(0, 0, -9.81)
        # p.setRealTimeSimulation(1) # Enable real-time simulation
        p.resetDebugVisualizerCamera(cameraDistance=2.7, cameraYaw=0, cameraPitch=-15,
                                        cameraTargetPosition=[-1.3, -0.5, 0.2])

        self.dt = 1./62. # Simulation time step (60 Hz)
        p.setPhysicsEngineParameter(fixedTimeStep=self.dt, numSubSteps=1)

        # LOAD PLANE IN THE SIMULATION
        self.plane = p.loadURDF("../../etc/URDFs/plane/plane.urdf", basePosition=[0, 0, 0]) 
        

    def simulate(self):
        print("Simulated simulation...")
        p.stepSimulation()
        time.sleep(3) # Sleep for 5 seconds to allow the user to see the simulation
        

def main():
    simulator = CausesSimulator()
    simulator.simulate()

if __name__ == "__main__":
    main()

