from vi import Simulation
from agent_flock import FlockingAgent

# Run the sim
simulation = (
    Simulation()
    .batch_spawn_agents(500, FlockingAgent, images=["examples/images/drone2.png"])
    .run()
)
