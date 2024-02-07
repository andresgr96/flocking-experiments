from sims.simulation_source import SourceSim
from agents.agent_flock import SourceAgent
from agents.agent_drone import DroneAgent
from vi import Config
import cv2

image_path = 'images/light_source.jpg'
image = cv2.imread(image_path)

# Run the sim
if __name__ == "__main__":
    simulation = (
        SourceSim(image, Config(image_rotation=True))
        .batch_spawn_agents(1, DroneAgent, images=["images/drone.png"])
        .run()
    )
