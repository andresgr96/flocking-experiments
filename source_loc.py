from sims.simulation_source import SourceSim
from agents.agent_flock import SourceAgent
from agents.agent_NHDD import NHDDAgent
from vi import Config
import cv2

image_path = 'images/light_source.jpg'
image = cv2.imread(image_path)

# Run the sim
if __name__ == "__main__":
    simulation = (
        SourceSim(image, Config(image_rotation=True, radius=2))
        .batch_spawn_agents(10, NHDDAgent, images=["images/agent_black3.png"], sep_distance=0.5)
        .run()
    )
