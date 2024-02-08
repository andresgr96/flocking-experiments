from __future__ import annotations
import collections
from typing import TYPE_CHECKING, Callable, Generator, TypeVar
from pygame.sprite import Group
from vi.proximity import ProximityEngine, ProximityIter
from pygame.math import Vector2

if TYPE_CHECKING:
    from vi import Agent

__all__ = [
    "Sensor",
]

AgentClass = TypeVar("AgentClass", bound="Agent")


def relative_angle(focal_agent, target_agent) -> float:
    """
    Calculate the angle between two agents relative to the focal agent's local frame of reference

    """
    relative_vector = target_agent.pos - focal_agent.pos
    rotated_vector = relative_vector.rotate(-focal_agent.heading)
    angle = rotated_vector.angle_to(Vector2(1, 0))
    return angle


def difference_vector(agent1: Agent, agent2: Agent) -> Vector2:
    """
    Calculate the difference vector between two agents

    """
    return agent2.pos - agent1.pos


class Sensor(ProximityEngine):
    __agents: Group

    __chunks: dict[tuple[int, int], set[Agent]]
    """A map between chunk locations and the agents currently in that chunk."""

    chunk_size: int
    """The size of the chunks used for the proximity calculation."""

    radius: int
    """The radius representing the agent's proximity view."""

    def __init__(self, agents: Group, radius: int):
        super().__init__(agents, radius)
        self.__agents = agents
        self.__chunks = collections.defaultdict(set)

        self._set_radius(radius)

    def _set_radius(self, radius: int):
        self.radius = radius
        self.chunk_size = radius * 2

    def __get_chunk(self, coordinates: tuple[int, int]) -> tuple[int, int]:
        """Retrieve the chunk coordinates for an agent's coordinates."""

        x, y = coordinates

        x_chunk = x // self.chunk_size
        y_chunk = y // self.chunk_size

        return (x_chunk, y_chunk)

    def update(self):
        """Update the internal chunk store with the agents' current positions."""

        self.__chunks.clear()

        for sprite in self.__agents.sprites():
            agent: Agent = sprite  # type: ignore

            chunk = self.__get_chunk(agent.center)
            self.__chunks[chunk].add(agent)

    def __fast_retrieval(self, agent: AgentClass) -> Generator[AgentClass, None, None]:
        chunk = self.__get_chunk(agent.center)

        for nearby_agent in self.__chunks[chunk]:
            if nearby_agent.id != agent.id and agent.is_alive():
                yield nearby_agent  # type: ignore

    def __accurate_retrieval(
        self, agent: AgentClass
    ) -> Generator[tuple[AgentClass, float, float, Vector2], None, None]:
        x, y = agent.center

        CHUNK_SIZE = self.chunk_size
        RADIUS = self.radius

        x_chunk, x_offset = divmod(x, CHUNK_SIZE)
        y_chunk, y_offset = divmod(y, CHUNK_SIZE)

        x_step = 1 if x_offset >= RADIUS else -1
        x_chunk_offset = 0 if x_offset == RADIUS else x_step

        y_step = 1 if y_offset >= RADIUS else -1
        y_chunk_offset = 0 if y_offset == RADIUS else y_step

        for x in range(x_chunk, x_chunk + x_chunk_offset + x_step, x_step):
            for y in range(y_chunk, y_chunk + y_chunk_offset + y_step, y_step):
                for other in self.__chunks[(x, y)]:
                    distance = agent.pos.distance_to(other.pos)
                    if (
                        other.id != agent.id
                        and agent.is_alive()
                        and distance <= self.radius
                    ):
                        relative_angle_value = relative_angle(agent, other)
                        diff_vector = difference_vector(agent, other)
                        yield (other, distance, relative_angle_value, diff_vector)  # type: ignore

    def in_proximity_accuracy(
        self, agent: AgentClass
    ) -> ProximityIter[tuple[AgentClass, float, float, Vector2]]:
        """Retrieve a set of agents that are in the same chunk as the given agent,
        in addition to the agents in the eight neighboring chunks.
        """

        agents = self.__accurate_retrieval(agent)
        return ProximityIter(agents)
