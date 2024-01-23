from sims.simulation_source import SourceSim
from agents.agent_flock import FlockingAgent

# Run the sim (make sure to change line 433 with: self._background.fill((255, 255, 255)) )
simulation = (
    SourceSim('images/light_source.jpg')
    .batch_spawn_agents(500, FlockingAgent, images=["images/drone2.png"])
    .run()
)
