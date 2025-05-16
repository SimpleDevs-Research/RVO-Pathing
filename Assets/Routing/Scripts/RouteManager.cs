using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RVO;
using DataStructures.ViliWonka.KDTree;

namespace Routing {
    public class RouteManager : MonoBehaviour
    {
        public static RouteManager Instance;

        [Header("=== Route Definitions ===")]
        [Tooltip("All nodes")]
        public Node[] nodes;
        private List<Vector3> node_positions;
        public List<Region> regions;
        private List<Vector3> region_positions;

        [Tooltip("All route segments in this scene")]
        public List<Segment> route_segments;

        [Header("=== Levers ===")]
        public bool consider_risk;
        public bool consider_crowding;
        public bool consider_dirtiness;
        public bool use_dijkstras;

        [Header("=== Global Weights ===")]
        public float distance_weight = 1;
        public float complexity_weight = 1;
        public float risk_weight = 1;
        public float density_weight = 1;
        public float condition_weight = 1;

        private KDTree regionTree, nodeTree;
        private KDQuery query;

        protected void Awake() {
            // Singleton Logic
            Instance = this;

            // Update our nodes with their indices
            node_positions = new List<Vector3>();
            for(int i = 0; i < nodes.Length; i++) {
                nodes[i].node_index = i;
                node_positions.Add(nodes[i].transform.position);
            }

            // Adjust at least the distance data for each segment
            // Also accrue region data, for implementation in the KDTree
            region_positions = new List<Vector3>();
            foreach(Segment segment in route_segments) {
                segment.UpdateSegment();
                foreach(Region region in segment.regions) {
                    if (!regions.Contains(region)) {
                        regions.Add(region);
                        region_positions.Add(region.transform.position);
                    }
                }
                if (segment.node1.region != null && !regions.Contains(segment.node1.region)) {
                    regions.Add(segment.node1.region);
                    region_positions.Add(segment.node1.transform.position);
                }
                if (segment.node2.region != null && !regions.Contains(segment.node2.region)) {
                    regions.Add(segment.node2.region);
                    region_positions.Add(segment.node2.transform.position);
                }
            }
            // Create KDTree for searching all regions and nodes
            regionTree = new KDTree(region_positions.ToArray(), 16);
            nodeTree = new KDTree(node_positions.ToArray(), 16);
            query = new KDQuery();
        }

        // Update loop, we update all our segments
        protected void Update() {
            foreach(Segment segment in route_segments) segment.UpdateSegment();
        }

        // Public function. Can be called by anyone.
        // Use this to query the closest region
        public Region getClosestRegion(Vector3 query_position) {
            List<int> results = new List<int>();
            query.ClosestPoint(regionTree, query_position, results);
            return regions[results[0]];
        }

        // Helper: Get the closest node
        public Node GetClosestNode(Vector3 query_position) {
            List<int> results = new List<int>();
            query.ClosestPoint(nodeTree, query_position, results);
            return nodes[results[0]];
        }

        // Public function. Can be called by anyone.
        // Given a personality of an agent, we can compute specific route details
        // In truth, this is legacy code. But I'm keeping it here for posterity
        public float[] computeSegmentCosts(Personality personality) {
            float[] costs = new float[route_segments.Count];
            for(int i = 0; i < route_segments.Count; i++) {
                costs[i] = route_segments[i].ComputePersonalCost(personality);
            }
            return costs;
        }

        // When we calculate the Dijkstra's algo, we need to compute edge costs
        public float[,] computeEdgeCosts(Personality personality) {
            // Initialize return array
            float[,] edges = new float[nodes.Length, nodes.Length];
            // Start filling out our edges with default values
            for (int i = 0; i < nodes.Length; i++) {
                for (int ii = 0; ii < nodes.Length; ii++) {
                    edges[i, ii] = -1;
                }
            }

            // Compute our segment costs now, and fill `edges` with the appropriate values
            for(int i = 0; i < route_segments.Count; i++) {
                Segment segment = route_segments[i];
                float cost = segment.ComputePersonalCost(personality);
                int ind1 = segment.node1.node_index;
                int ind2 = segment.node2.node_index;
                edges[ind1, ind2] = cost;
                edges[ind2, ind1] = cost;
            }

            // Return
            return edges;
        }

        // This is the function that gets called by an agent if they want to re-path
        public Node getNextNode(Node start, Node end, Personality personality) {
            // If we toggle off dijkstra's, just give the start node
            if (!use_dijkstras) return end;

            // We won't return the next best path. But we will return the next node along that best path
            List<Node> bestPath = new List<Node>();
            float[,] edges = computeEdgeCosts(personality);
            
            // Compute the filler float arrays with precomputed values. These represent the unvisited set
            float[] minimumDistance = new float[nodes.Length];
            float[] distances = new float[nodes.Length];
            int[] prevNode = new int[nodes.Length];
            for (int i = 0; i < nodes.Length; i++) {
                minimumDistance[i] = int.MaxValue;
                distances[i] = int.MaxValue;
                prevNode[i] = -1;
            }
            // Note that we need at least one in the set that isn't just a max value
            distances[start.node_index] = 0;
            // Conduct Dijkstra's. If it takes too long... we failsafe out
            int failsafe1 = 0;
            while (infCount(minimumDistance) > 0 && failsafe1 < 99) {
                failsafe1++;
                float currentBestDistance = int.MaxValue;
                int currentBestNodeInd = -1;
                for (int i = 0; i < distances.Length; i++) {
                    if (distances[i] < currentBestDistance && minimumDistance[i] == int.MaxValue) {
                        currentBestNodeInd = i;
                        currentBestDistance = distances[i];
                    }
                }
                int currentNode = currentBestNodeInd;
                minimumDistance[currentNode] = distances[currentNode];
                for(int i = 0; i < nodes.Length; i++) {
                    if (edges[currentNode, i] == -1) continue;
                    float possibleNewDistance = distances[currentNode] + edges[currentNode, i];
                    if(possibleNewDistance < distances[i]) {
                        distances[i] = possibleNewDistance;
                        prevNode[i] = currentNode;
                    }
                }
            }

            // We reconstruct our path starting from the end.
            bestPath.Add(end);
            int currentNodeOnBestPath = prevNode[end.node_index];

            // Like before, we failsafe out if needed
            int failsafe2 = 0;
            while(currentNodeOnBestPath != -1 && failsafe2 < 10) {
                failsafe2++;
                bestPath.Insert(0, nodes[currentNodeOnBestPath]);
                currentNodeOnBestPath = prevNode[currentNodeOnBestPath];
            }

            // We ALWAYS return the next node, not the entire path.
            return bestPath[1];
        }
    
        // Helper. Given an array of floats, tell us how many are infinite values
        private int infCount(float[] array) {
            int count = 0;
            for(int i = 0; i < array.Length; i++) {
                if (array[i] == int.MaxValue) count++;
            }
            return count;
        }

    }
}
