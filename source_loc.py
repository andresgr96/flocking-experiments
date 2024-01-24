from sims.simulation_source import SourceSim
from agents.agent_flock import SourceAgent
import cv2

image_path = 'images/light_source.jpg'
image = cv2.imread(image_path)

# Run the sim
simulation = (
    SourceSim(image)
    .batch_spawn_agents(50, SourceAgent, images=["images/drone2.png"])
    .run()
)
