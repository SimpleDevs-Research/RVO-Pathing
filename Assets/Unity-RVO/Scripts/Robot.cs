using System.Collections;
using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace RVO {
    public class Robot : MonoBehaviour
    {
        [Header("=== Robot Settings ===")]
        public int agent_index;
        public Generator generator;
        public Personality personality;
        Color neighbor_color = Color.blue;
        Color new_velocity_color = Color.red;

        #if UNITY_EDITOR
        protected void OnDrawGizmosSelected() {
            if (!Application.isPlaying) return;
            if (generator == null) return;
            // Draw ourselves
            Vector3 pA = transform.parent.TransformPoint(generator.vo_op.positions[agent_index]);
            Gizmos.color = Color.black;
            Gizmos.DrawWireSphere(pA, personality.spatial_radius);
            // show neighbors
            Gizmos.color = generator.vo_op.colliding[agent_index] ? Color.red : Color.blue;
            for(int i = 0; i < generator.vo_op.num_neighbors[agent_index]; i++) {
                int neighbor_index = generator.vo_op.neighbor_indices[agent_index*generator.max_neighbors+i];
                Vector3 pB = transform.parent.TransformPoint(generator.vo_op.positions[neighbor_index]);
                float neighbor_radius = generator.vo_op.radii[neighbor_index];
                Gizmos.DrawLine(pA,pB);
                Gizmos.DrawWireSphere(pB, neighbor_radius);
            }
            // Show new velocity
            Gizmos.color = new_velocity_color;
            Gizmos.DrawRay(pA,generator.vo_op.new_velocities[agent_index]);
            //GUI.color = new_velocity_color;
            //Handles.Label(pA+new Vector3(1f,0f,1f), generator.penalties[agent_index].ToString());

        }
        #endif
    }
}
