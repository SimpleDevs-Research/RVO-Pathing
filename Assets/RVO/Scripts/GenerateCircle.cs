using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

using DataStructures.ViliWonka.KDTree;

namespace RVO {
    public class GenerateCircle: Generator {
        
        [Header("=== Circular Arrangement Properties ===")]
        public float arrangement_radius = 10f;

        protected override void GenerateAgent(int agent_index, ref Transform[] _transforms) {
            
            // This overrides the base `GenerateAgent` method of the original `Generator` script.
            // In this implementation, we pre-calculate the vector of the start point based on agent index, then we set the destination to the opposite side of the circular arrangement.
            // The only inspector parameter unique to this is the `arrangement radius`, which just dictates how big the circular arrangement is.

            // Based on bounds, determine the origin point of the circular arrangement
            Vector3 centroid = new Vector3(bounds.x, 0f, bounds.y) / 2f;

            // Calculate the directional ray from the centroid to the start position of this agent
            float angle_step = 2f * Mathf.PI / num_agents;
            float theta = agent_index * angle_step;
            float x = Mathf.Sin(theta);
            float y = Mathf.Cos(theta);
            Vector3 start_ray = new Vector3(arrangement_radius*x, 0f, arrangement_radius*y);

            // Determine the start and end point based on start_ray
            Vector3 pos = centroid + start_ray;
            Vector3 dest = centroid - start_ray;

            // Now we proceed with the original script
            Vector3 diff = dest - pos;
            Vector3 forward = (diff.sqrMagnitude == 0f) ? Vector3.right : diff.normalized;

            // Step 2: Populate our native arrays with these details. Note that we default velocity as a zero vector. 
            //          We also assume all agents have the same spatial radius
            positions[agent_index] = pos;
            velocities[agent_index] = Vector3.zero;
            //radii[agent_index] = spatial_radius;
            destinations[agent_index] = dest;
            // We use a nested for loop because neighbor_indices occupy a set range of spaces in the `neighbor_indices` nativearray buffer.
            for(int j = 0; j < max_neighbors; j++) {
                neighbor_indices[agent_index*max_neighbors+j] = agent_index;
                //is_colliding[agent_index*max_neighbors+j] = false;
            }
            num_neighbors[agent_index] = 0;
            new_velocities[agent_index] = Vector3.zero;
            reached_destination[agent_index] = false;

            // Step 3: Generate agents to represent these in physical world space
            GameObject go = Instantiate(agent_prefab, pos, Quaternion.LookRotation(forward));
            Transform t = go.transform;
            t.parent = agent_parent;
            _transforms[agent_index] = t;
            agent_positions[agent_index] = pos;
        }
    }
}
