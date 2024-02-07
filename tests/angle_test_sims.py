from sims.simulation_source import SourceSim
from agents.test_agent import TestDroneAgent
from vi import Config
import cv2

image_path = '../images/light_source.jpg'
image = cv2.imread(image_path)

"""
This sim requires a position for the agent and can be used to confirm calculations like relative angles.

For it to work you'll need to move this script up one dir, and modify the spawn_agent function in the original
simulation file to also take the position, and remember to change it back after testing!
"""

# Run the sim
if __name__ == "__main__":
    simulation = (
        SourceSim(image, Config(image_rotation=True))
        .spawn_agent(TestDroneAgent, (200, 100), images=["images/drone.png"])
        .spawn_agent(TestDroneAgent, (200, 90), images=["images/drone.png"])
        .run()
    )
