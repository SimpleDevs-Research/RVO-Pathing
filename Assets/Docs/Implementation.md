# Implementation Notes

This implementation of RVO can be divided into three distinct components:

1. **Vision**: How agents "see" other agents around them. You can generally do this two ways: either a) considering all other agents in the simulation, or b) only considering neighboring agents within some empirical bound (i.e. distance). In the real world, we do not have global knowledge about the state of all pedestrians in an urban space. To this end, _Unity-RVO_ considers only agents within a provided distance threshold and visual angle in front of each agent. The former restricts RVO to a truly **local** collision avoidance measure; the latter replicates the vision cone of humans in concept.

2. **Processing**: How agents "process" the optimal velocity to move, based on the states of all neighbors. This would be where VO logic would operate. In our RVO implementation, we iterate across a set of possible pre-determined velocities and observe their suitability as a potential velocity to move in. Those that are "suitable" (AKA they will not lead to a collision with at least one neighbor) will be ranked based on difference from the agent's preferred velocity (the "penalty"), and the optimal velocity is the suitable velocity with the smallest penalty.

3. **Action**: Given an optimal velocity from the **Processing** step, nudge the agent to face towards the optimal direction and adjust their current velocity to match the optimal velocity. Rather than directly set the current velocity to the optimal velocity, agents will accelerate to the optimal velocity; this enables smoother movement and the removal of jittering as agents transition between different optimal velocities per frame.

These steps (**Vision** to **Processing** to **Action**) mimics the conceptual framework established by spatial cognition literature. Spatial cognition can be briefly described as the flow of information from sensory inputs into mental representations of the physical space; this mental representation will affect how individuals adjust their movement to move and avoid collisions. Note that this implementation **DOES NOT IMPLEMENT PATH-FINDING** - the RVO algorithm does not consider pathfinding within a provided space.

## External Dependencies:

1. Unity KDTree Implementation: [https://github.com/viliwonka/KDTree](https://github.com/viliwonka/KDTree)

## Components

