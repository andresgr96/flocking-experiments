from sims.simulation_source import SourceSim
from sims.simulation_tests import SimTest
from agents.test_agents import TestNHDDAgent
from agents.stat_agent import TestStatAgent
from vi import Config
import cv2

image_path = 'images/light_source.jpg'
image = cv2.imread(image_path)

"""
This sim requires a position for the agent and can be used to confirm calculations like relative angles.

For it to work you'll need to move this script up one dir, and modify the spawn_agent function in the original
simulation file to also take the position, and remember to change it back after testing!
"""

# Run the sim
if __name__ == "__main__":
    simulation = (
        SimTest(image, Config(image_rotation=True))
        .spawn_agent(TestNHDDAgent, (200, 100), images=["images/agent_black3.png"])
        .spawn_agent(TestStatAgent, (198, 100), images=["images/agent.png"])
        .run()
    )