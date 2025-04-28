using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

using DataStructures.ViliWonka.KDTree;


public class GenerateAgentsInCircle: GenerateAgentsWithTrajectories
{
    [Header("=== Circular Arrangement Properties ===")]
    public float arrangement_radius = 10f;

    public override void Generate() {
        // In this one, the number of agents is equal to the number of stored trajectories
        num_agents = agent_trajectories.Length;
        
        // Initialize the lists for KDTree
        agent_positions = new Vector3[num_agents];
        agent_components = new Pedestrian[num_agents];
        agent_data = new AgentData[num_agents];

        // Want to build a ciruclar arrangement here
        List<float2> arrangement_starts = GenerateDirections(num_agents, arrangement_radius);
        Vector3 arrangement_centroid = new Vector3(bounds.x, 0f, bounds.y) / 2f;

        // Generate each agent individually
        for(int i = 0; i < agent_trajectories.Length; i++) {

            StartDestinationPair sdp = agent_trajectories[i];
            float2 arrangement_start_direction = arrangement_starts[i];
            Vector3 arr_direction = new Vector3(arrangement_start_direction[0], 0f, arrangement_start_direction[1]);

            // Generate random position as start point
            Vector3 start_point = arrangement_centroid + arr_direction;
            Vector3 end_point = arrangement_centroid - arr_direction;
            
            // Instantiate agent. If the agent wants to move themselves, then we leave it up to the agent prefab instance itself.
            Pedestrian ps = Instantiate(agent_prefab, start_point, Quaternion.identity) as Pedestrian;
            ps.transform.parent = agent_parent;
            ps.agent_index = i;
            ps.gameObject.name = $"Agent {i}";
            ps.generate_destination_on_start = false;
            ps.destination = end_point;

            // Initialize pedestrian data
            AgentData ped = new AgentData(i, start_point, Vector3.zero, ps.spatial_radius);

            // Add to agent_positions and agent_data
            agent_positions[i] = start_point;
            agent_components[i] = ps;
            agent_data[i] = ped;
        }

        // Initialize our KDTree and Query
        tree = new KDTree(agent_positions, 32);
        query = new KDQuery();
    }
}
