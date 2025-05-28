using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

namespace Routing {
    public class TestRouteManager : MonoBehaviour
    {
        public Transform destinationRef;
        public Personality personality;

        public Color startColor = Color.blue;
        public Color currentColor = Color.green;
        public Color nextColor = Color.yellow;
        public Color endColor = Color.red;
        public Color pathColor = Color.black;
        
        // Comes with a current node, a next node, and an end node.
        private Node startNode;
        private Node currentNode;
        private Node nextNode;
        private Node endNode;
        public List<Vector3> path = new List<Vector3>();

        void OnDrawGizmos() {
            if (!Application.isPlaying) return;
            if (startNode != null) {
                Gizmos.color = startColor;
                Gizmos.DrawCube(startNode.transform.position, Vector3.one);
            }
            if (currentNode != null) {
                Gizmos.color = currentColor;
                Gizmos.DrawCube(currentNode.transform.position, Vector3.one);
            }
            if (nextNode != null) {
                Gizmos.color = nextColor;
                Gizmos.DrawCube(nextNode.transform.position, Vector3.one);
            }
            if (endNode != null) {
                Gizmos.color = endColor;
                Gizmos.DrawCube(endNode.transform.position, Vector3.one);
            }
            if (path != null && path.Count > 0) {
                Gizmos.color = pathColor;
                foreach(Vector3 p in path) Gizmos.DrawSphere(p, 0.25f);
            }
        }

        private void Start() {
            startNode = RouteManager.Instance.GetClosestNode(transform.position);
            endNode = RouteManager.Instance.GetClosestNode(destinationRef.position);
            // Given these three nodes, check the path to the current
            StartCoroutine(CalculatePathToCurrentNode(startNode));
        }

        private IEnumerator CalculatePathToCurrentNode(Node fromNode) {
            // What's the current node to head to?
            currentNode = RouteManager.Instance.getNextNode(fromNode, endNode, personality);
            Vector3 destination = currentNode.GetRandomPoint();

            // Get NavMesh path. Do it over multiple frames if needed.
            NavMeshPath navPath = new NavMeshPath();
            bool pathFound = false;
            do {
                pathFound = NavMesh.CalculatePath(
                    transform.position, 
                    currentNode.transform.position, 
                    NavMesh.AllAreas, 
                    navPath
                );
                yield return null;
            } while(!pathFound);

            // Check each point along the provided path
            NavMeshHit hit;
            List<Vector3> positions = new List<Vector3>();
            for(int i = 0; i < navPath.corners.Length; i++) {
                Vector3 p = navPath.corners[i];
                if (NavMesh.FindClosestEdge(p, out hit, NavMesh.AllAreas)) {
                    Vector3 p2 = (hit.distance < personality.spatial_radius) 
                        ? hit.position + hit.normal * personality.spatial_radius
                        : p;
                    positions.Add(p2);
                }
            }

            // Cache the path data from the modified positions
            path = positions;
        }

        private void Update() {

            // Ensure that we're still in the navmesh
            KeepInMesh();
            
            // At least a subpath exists
            if (path.Count >= 2 && CheckCurrentNode()) {
                // Must adjust either path or node
                path.RemoveAt(0);
                if (path.Count <= 1) {
                    // we've reached our current node. Have we reached our destination node?
                    if (currentNode == endNode) {
                        gameObject.SetActive(false);
                        return;
                    }
                    // Need to calculate the next current node
                    StartCoroutine(CalculatePathToCurrentNode(currentNode));
                }
            }
        }

        private void KeepInMesh() {
            NavMeshHit hit;
            if (NavMesh.SamplePosition(transform.position, out hit, 1f, NavMesh.AllAreas)) transform.position = hit.position;
        }

        private bool CheckCurrentNode() {
            Vector3 fromPath = path[1] - path[0];
            Vector3 fromPosition = path[1] - transform.position;
            float dot = Vector3.Dot(fromPath, fromPosition);
            return dot <= 0f;
        }
    }
}
