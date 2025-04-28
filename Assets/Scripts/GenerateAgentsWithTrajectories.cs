using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using DataStructures.ViliWonka.KDTree;


public class GenerateAgentsWithTrajectories : GenerateAgents
{
    [System.Serializable]
    public class StartDestinationPair {
        public Vector3 start;
        public Vector3 end;
        public Color color;
        public List<Vector3> points = new List<Vector3>();
    }

    public bool draw_trajectory_gizmos = false;
    public StartDestinationPair[] agent_trajectories;

    #if UNITY_EDITOR
    protected override void OnDrawGizmos() {
        base.OnDrawGizmos();
        if (!draw_trajectory_gizmos) return;

        for (int i = 0; i < agent_trajectories.Length; i++) {
            StartDestinationPair sdp = agent_trajectories[i];
            Gizmos.color = sdp.color;
            if (!Application.isPlaying) {
                Gizmos.DrawSphere(sdp.start, 0.25f);
                Gizmos.DrawWireSphere(sdp.end, 0.25f);
                Gizmos.DrawLine(sdp.start, sdp.end);
            } else {
                if (sdp.points.Count >= 2) {
                    for(int j = 0; j < sdp.points.Count-1; j++) {
                        Gizmos.DrawLine(sdp.points[j], sdp.points[j+1]);
                    }
                }
            }
        }
    }
    #endif

    public override void Generate() {
        // In this one, the number of agents is equal to the number of stored trajectories
        num_agents = agent_trajectories.Length;
        
        // Initialize the lists for KDTree
        agent_positions = new Vector3[num_agents];
        agent_components = new Pedestrian[num_agents];
        agent_data = new AgentData[num_agents];

        // Generate each agent individually
        for(int i = 0; i < agent_trajectories.Length; i++) {

            StartDestinationPair sdp = agent_trajectories[i];

            // Generate random position as start point
            Vector3 start_point = sdp.start;
            Vector3 end_point = sdp.end;
            
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

    protected override void LateUpdate() {
        // Update each agent's data
        // We do it here to enable the update loop in each independent agent to conduct Observation and Processing
        for(int i = 0; i < agent_positions.Length; i++) {
            agent_components[i].current_velocity = agent_components[i].velocity;
            agent_positions[i] = agent_components[i].position;
            agent_data[i].Update(agent_components[i].position, agent_components[i].velocity);
            if (agent_components[i].initialized) agent_trajectories[i].points.Add(new Vector3(agent_positions[i].x, agent_components[i].current_velocity.magnitude, agent_positions[i].z));
        }

        // We update the KDTree here and now
        tree.Rebuild();
    }
}
