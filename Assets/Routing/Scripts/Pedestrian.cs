using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using Random = UnityEngine.Random;
using RVO;

namespace Routing {
    public class Pedestrian : Robot
    {
        [Header("=== Routing Settings ===")]
        public Node start_node;
        public Node next_node;
        public Node goal_node;
        public Region current_region;
        public List<Vector3> path_points = new List<Vector3>();
        public bool calculating_path = false;

        // Unlike the typical Robot, which only does OnDrawGizmos, a pedestrian:
        //  1. Keeps track of a major "goal" (the Generator script only keeps the current destination along its path)
        //  2. Recalculates the path (and next destination) depending on whether the current robot has reached its current destination
        protected virtual void Start()
        {
            // At the beginning, the start and end nodes are provided to this pedestrians
            // We need to initialize the path though
            StartCoroutine(CalculatePath(start_node));
        }

        // Callable from Generator or PedestrianManager.
        protected virtual void LateUpdate()
        {
            // Ensure that we're still in the navmesh
            KeepInMesh();
            CheckCurrentRegion();

            // Don't do anything if we're waiting for a path
            if (calculating_path) return;

            // At least a subpath exists
            if (path_points.Count >= 2 && CheckNextNode())
            {
                // Must adjust either path or node
                path_points.RemoveAt(0);
                if (path_points.Count <= 1)
                {
                    // we've reached our current node. Have we reached our destination node?
                    if (next_node == goal_node)
                    {
                        Generator.current.ToggleRobot(agent_index, false);
                        //gameObject.SetActive(false);
                        return;
                    }
                    // Need to calculate the next current node
                    StartCoroutine(CalculatePath(next_node));
                }
                else
                {
                    Generator.current.vo_op.destinations[agent_index] = path_points[1];
                    Generator.current.vo_op.reached_destination[agent_index] = false;
                }
            }
            else {
                Generator.current.vo_op.reached_destination[agent_index] = false;
            }
        }

        // Repath calculation
        protected virtual IEnumerator CalculatePath(Node from_node)
        {
            // Set flag
            calculating_path = true;

            // Query route manager for the next node destination
            next_node = RouteManager.Instance.getNextNode(from_node, goal_node, personality);
            Vector3 destination = next_node.GetRandomPoint();

            // We got a destination, but it's not the one we'll set in Generator.
            // We need to calculate the sub-path to our destination. We do this via NavMesh
            NavMeshPath nav_path = new NavMeshPath();
            bool path_found = false;
            do
            {
                path_found = NavMesh.CalculatePath(
                    transform.position,
                    destination,
                    NavMesh.AllAreas,
                    nav_path
                );
                yield return null;
            } while (!path_found);

            // Check each point along the provided path
            NavMeshHit hit;
            List<Vector3> positions = new List<Vector3>();
            for (int i = 0; i < nav_path.corners.Length; i++)
            {
                Vector3 p = nav_path.corners[i];
                if (NavMesh.FindClosestEdge(p, out hit, NavMesh.AllAreas))
                {
                    Vector3 p2 = (hit.distance < personality.spatial_radius)
                        ? hit.position + hit.normal * personality.spatial_radius
                        : p;
                    positions.Add(p2);
                }
            }

            // Cache the path data from the modified positions
            path_points = positions;

            // Set the SECOND item in `path_points` as our destination
            Generator.current.vo_op.destinations[agent_index] = path_points[1];

            // Reset flag
            calculating_path = false;
        }

        // We want pedestrians to stay in the mesh, to make sure navmesh works
        protected virtual void KeepInMesh()
        {
            try
            {
                NavMeshHit hit;
                if (NavMesh.SamplePosition(transform.position, out hit, 1f, NavMesh.AllAreas))
                {
                    transform.position = hit.position;
                    Generator.current.vo_op.positions[agent_index] = hit.position;
                }
            }
            catch (Exception e) {
                Debug.LogError(e);
            }   
        }

        // What node are we currently in?
        protected virtual void CheckCurrentRegion()
        {
            Region r = RouteManager.Instance.getClosestRegion(transform.position);
            if (r != current_region)
            {
                if (current_region != null) current_region.RemoveRobot(this);
                current_region = r;
                current_region.AddRobot(this);
            }
        }
        
        // Do we need to repath? If so, return true.
        // Need to repath depends on either 1. Generator has detected that we reached our destination, or 2) we've passed our current destination point
        protected virtual bool CheckNextNode()
        {
            //if (Generator.current.vo_op.reached_destination[agent_index]) return true;
            Vector3 from_path = path_points[1] - path_points[0];
            Vector3 from_position = path_points[1] - transform.position;
            return
                Vector3.Dot(from_path, from_position) <= 0f
                || Vector3.Distance(transform.position, path_points[1]) <= personality.spatial_radius;
        }
    }
}
