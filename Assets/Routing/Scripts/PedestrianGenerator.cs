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
        protected override void GenerateAgent(int agent_index) {
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
            Personality p = demographics.GetRandomPersonality();

            // Step 2: Instantiate the agent itself
            GameObject go = Instantiate(agent_prefab, pos, Quaternion.LookRotation(forward));
            Transform t = go.transform;
            t.parent = agent_parent;
            Pedestrian ped = t.GetComponent<Pedestrian>();
            pedestrians.Add(ped);
            ped.agent_index = agent_index;
            ped.personality = p;
            ped.current_destination_node = start_node;
            ped.goal_node = end_node;

            // Step 3: Inform our agent data in vo_op
            vo_op.AddAgent(agent_index, pos, dest, t, p);

            // Step 4: Miscellaneous. For Robot components and KDTree stuff.
            agent_positions[agent_index] = pos;
        }
    }
}
