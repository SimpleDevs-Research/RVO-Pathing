using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using Random = UnityEngine.Random;

namespace Routing {
    [ExecuteInEditMode]
    public class Node : MonoBehaviour
    {
        [Header("=== Node Properties ===")]
        public int node_index;
        public float acceptable_radius;
        public List<Node> neighbors;
        public bool auto_radius = true;
        [Space]
        public Region region = null;

        [Header("=== Debug ===")]
        public bool draw_gizmos = false;
        public Color node_color = Color.blue;
        public Color edge_color = Color.red;
        public Color closest_edge_point_color = Color.black;

        #if UNITY_EDITOR
        private void OnDrawGizmosSelected() {
            if (!draw_gizmos) return;
            Gizmos.color = node_color;
            Gizmos.DrawSphere(transform.position, acceptable_radius);
            if (neighbors.Count > 0) {
                Gizmos.color = edge_color;
                foreach(Node neighbor in neighbors) Gizmos.DrawLine(transform.position, neighbor.transform.position);
           }

            NavMeshHit hit;
            if (NavMesh.FindClosestEdge(transform.position, out hit, NavMesh.AllAreas)) {
                Gizmos.color = closest_edge_point_color;
                Gizmos.DrawSphere(hit.position, 0.1f);
                Gizmos.DrawLine(transform.position, hit.position);
            }
        }
        #endif

        protected virtual void Update() {
            // Adjust acceptable radius based on distance to edge
            if (auto_radius) {
                NavMeshHit hit;
                if (NavMesh.FindClosestEdge(transform.position, out hit, NavMesh.AllAreas)) {
                    acceptable_radius = hit.distance;
                }
            }

            // if our region is defined, then contribute to its size
            if (region != null) {
                region.radius = acceptable_radius;
                region.size = Mathf.PI * acceptable_radius * acceptable_radius;
            }
        }

        protected virtual void OnValidate() {
            // For each neighbor, make sure they themselves reference us as their neighbor
            if (neighbors.Count > 0) {
                foreach(Node neighbor in neighbors) {
                    if (!neighbor.neighbors.Contains(this)) neighbor.neighbors.Add(this);
                }
            }

            // Check if we have a region attached to this
            region = GetComponent<Region>();
        }

        // publicly callable function to generate a random point
        public Vector3 GetRandomPoint() {
            return transform.position + new Vector3(
                Random.Range(-acceptable_radius, acceptable_radius),                
                0f,
                Random.Range(-acceptable_radius, acceptable_radius)
            );
        }
    }
}