using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;
using RVO;

namespace Routing {
    public class Pedestrian : Robot
    {
        [Header("=== Routing Settings ===")]
        public Node current_destination_node;
        public Node goal_node;

        // Unlike the typical Robot, which only does OnDrawGizmos, a pedestrian:
        //  1. Keeps track of a major "goal" (the Generator script only keeps the current destination along its path)
        //  2. Recalculates the path (and next destination) depending on whether the current robot has reached its current destination
        protected virtual void Start() {
            // To be safe, let's repath
            CalculatePath();
        }

        protected virtual void Update() {
            if (Generator.current.reached_destination[agent_index]) {
                // We're within the acceptable radius of our current_destination_node
                // If the current_destination_node is the same as our goal node already...
                //      ... then that means we should despawn
                if (current_destination_node == goal_node) {
                    // Despawn... by disengaginge ourselves as inactive
                    // We do this via our Generator singleton though. This is to make sure
                    // that the Generator has control over what happens when we despawn
                    Generator.current.ToggleRobot(agent_index, false);
                    return;
                }
                // Since current_destination_node != goal_node, then we have to repath
                CalculatePath();
            }
        }

        // Repath calculation
        protected virtual void CalculatePath() {
            if (RouteManager.Instance == null) {
                current_destination_node = goal_node;
            }
            else {
                current_destination_node = RouteManager.Instance.getNextNode(
                    current_destination_node, 
                    goal_node,
                    personality
                );
            }
            Generator.current.destinations[agent_index] = current_destination_node.GetRandomPoint();
        } 
    }
}
