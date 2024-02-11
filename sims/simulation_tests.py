from __future__ import annotations
from vi import Agent, Config, Simulation
from .simulation_source import SourceSim
from typing import TYPE_CHECKING, Optional, Type, TypeVar
from utils.sensor import Sensor

if TYPE_CHECKING:
    from typing_extensions import Self

    from vi.agent import Agent

    AgentClass = TypeVar("AgentClass", bound=Agent)

__all__ = [
    "Simulation",
    "SourceSim"
]

class SimTest(SourceSim):
    def __init__(self, image, config: Optional[Config] = None):
        super().__init__(image, config)
        self._proximity = Sensor(self._agents, self.config.radius, self._gradient)

    def spawn_agent(
        self,
        agent_class: Agent,
        initial_position: tuple[int, int],
        heading:float,
        images: list[str],
    ) -> 'SimTest':


        agent_class(images=self._load_images(images), simulation=self, initial_position=initial_position, heading=heading)

        return self


