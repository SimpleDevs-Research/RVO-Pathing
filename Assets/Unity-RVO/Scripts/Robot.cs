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
        public Personality personality;
        Color neighbor_color = Color.blue;
        Color new_velocity_color = Color.red;

        #if UNITY_EDITOR
        protected void OnDrawGizmosSelected() {
            if (!Application.isPlaying) return;
            if (Generator.current == null) return;
            // show neighbors
            Gizmos.color = neighbor_color;
            Vector3 pA = Generator.current.positions[agent_index];
            for(int i = 0; i < Generator.current.num_neighbors[agent_index]; i++) {
                int neighbor_index = Generator.current.neighbor_indices[agent_index*Generator.current.max_neighbors+i];
                Vector3 pB = Generator.current.positions[neighbor_index];
                Gizmos.DrawLine(pA,pB);
                Gizmos.DrawWireSphere(pB,0.5f);
            }
            // Show new velocity
            Gizmos.color = new_velocity_color;
            Gizmos.DrawRay(pA,Generator.current.new_velocities[agent_index]);
            GUI.color = new_velocity_color;
            Handles.Label(pA+new Vector3(1f,0f,1f), Generator.current.penalties[agent_index].ToString());

        }
        #endif
    }
}
