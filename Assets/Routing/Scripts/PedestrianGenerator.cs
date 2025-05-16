using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;
using Unity.Mathematics;
using RVO;

namespace Routing {
    // This one still carries a LOT of the same operations as the base `Generator` class
    // The only distinctions here are that we want to update our Pedestrians, which already extend the `Robot` class...
    //      ... into considering goal nodes and current goal nodes
    // To do this, we overwrite some base functions in the Generator class

    public class PedestrianGenerator : Generator
    {
        // The first thing to note is that the spawn behavior must be overwritten to account for the new route system
        // Here, we provide an alternative spawn behavior enum
        public enum PedSpawnStyle { 
            Random,         // Choose a random start and end node 
            StartToEnd      // The start/end positions must be among those dictated as start/end nodes
        }

        // This is a unique list to our system. We keep track of all our pedestrians using it.
        [Header("=== Pedestrian Routing System ===")]
        public PedSpawnStyle ped_spawn_style = PedSpawnStyle.Random;
        public List<SpawnRate<Node>> start_nodes;
        public List<SpawnRate<Node>> end_nodes;
        [Space]
        public List<Pedestrian> pedestrians = new List<Pedestrian>();

        // This function overwrites the original `GenerateAgent` function by...
        // ... update it with a current destination node and a goal node system
        protected override void GenerateAgent(int agent_index, ref Transform[] _transforms) {
            // Step 1a: Generate the start and end positions of our agent's trajectory via randomization
            // UPDATED PART: we look at start and end nodes instead of start and end positions intiially (we do that later).
            Node start_node, end_node;
            int r;
            switch(ped_spawn_style) {
                case PedSpawnStyle.StartToEnd:
                    // Choose a start node
                    r = (int)(Random.value * 100f);
                    start_node = start_nodes[0].value;
                    for(int i = 0; i < start_nodes.Count; i++) {
                        Vector2Int chance = start_nodes[i].spawn_chance;
                        if (chance.x <= r && r < chance.y) start_node = start_nodes[i].value;
                    }
                    // Choose an end node
                    do {
                        r = (int)(Random.value * 100f);
                        end_node = end_nodes[0].value;
                        for(int i = 0; i < end_nodes.Count; i++) {
                            Vector2Int chance = end_nodes[i].spawn_chance;
                            if (chance.x <= r && r < chance.y) end_node = end_nodes[i].value;
                        }
                    } while (start_node == end_node);
                    break;
                default:
                    // Default case = randomly choose
                    // Choose an end node
                    r = Random.Range(0,RouteManager.Instance.nodes.Length);
                    start_node = RouteManager.Instance.nodes[r];
                    // Choose an end node
                    do {
                        r = Random.Range(0,RouteManager.Instance.nodes.Length);
                        end_node = RouteManager.Instance.nodes[r];
                    } while (start_node == end_node);
                    break;
            }
            // Step 1b. Given the start and end nodes, generate the start and end positions... randomly
            Vector3 pos = start_node.GetRandomPoint();
            Vector3 dest = end_node.GetRandomPoint();

            // Step 1c. Calculate additional properties based on the start and end positions.
            Vector3 diff = dest - pos;
            Vector3 forward = (diff.sqrMagnitude == 0f) ? Vector3.right : diff.normalized;

            // Step 2: Populate our native arrays with these details. 
            //      Note that we default velocity as a zero vector. 
            //      We also assume all agents have the same spatial radius
            positions[agent_index] = pos;
            velocities[agent_index] = Vector3.zero;
            destinations[agent_index] = dest;
            // We use a nested for loop because neighbor_indices occupy a set range of spaces in the `neighbor_indices` nativearray buffer.
            for(int j = 0; j < max_neighbors; j++) {
                neighbor_indices[agent_index*max_neighbors+j] = agent_index;
                //is_colliding[agent_index*max_neighbors+j] = false;
            }
            is_colliding[agent_index] = false;
            num_neighbors[agent_index] = 0;
            new_velocities[agent_index] = Vector3.zero;
            penalties[agent_index] = new float2(0f);
            reached_destination[agent_index] = false;
            active[agent_index] = true;

            // Step 3: Use demographics to generate the next pedestrian parameters
            Personality p = demographics.GetRandomPersonality();
            responsibility_factors[agent_index] = p.responsibility_factor;
            safety_factors[agent_index] = p.safety_factor;
            inertia_factors[agent_index] = p.inertia_factor;
            radii[agent_index] = p.spatial_radius;
            max_speeds[agent_index] = p.max_speed;
            accelerations[agent_index] = p.acceleration;

            // Step 4: Generate agents to represent these in physical world space
            // UPDATED PART: given the current positions (positions[agent_index]) and destination (destinations[agent_index])...
            //  ... Let's determine the closest node to our current position as the current destination_node...
            //  ... and the goal node as the closest to our current destination
            GameObject go = Instantiate(agent_prefab, pos, Quaternion.LookRotation(forward));
            Transform t = go.transform;
            Pedestrian ped = t.GetComponent<Pedestrian>();
            t.parent = agent_parent;
            agent_positions[agent_index] = pos;
            _transforms[agent_index] = t;
            pedestrians.Add(ped);
            ped.personality = p;
            ped.current_destination_node = start_node;
            ped.goal_node = end_node;
        }
    }
}
