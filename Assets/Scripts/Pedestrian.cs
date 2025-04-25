using System.Collections;
using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

public class Pedestrian : MonoBehaviour
{
    [Header("=== Agent Settings ===")]
    public Vector3 destination;
    public bool initialized = false;
    public int agent_id;
    public Transform mesh_transform;

    [Header("=== Neighbor Querying ===")]
    public List<int> neighbor_indices = new List<int>();
    public Color neighbor_color = Color.blue;
    public float angle_threshold = 90f;

    [Header("=== RVO ===")]
    public Vector3 current_velocity;
    public Vector3 desired_direction; // normalized
    public Vector3 desired_velocity;
    public float max_speed = 2.5f;
    public float speed = 2f;
    public float radius = 0.5f;
    public float neighborhood_radius = 3f;

    #if UNITY_EDITOR
    void OnDrawGizmos() {
        // Return early if not even playing
        if (!Application.isPlaying) return;

        if (!initialized) {
            // Gizmos to know that we're done
            Gizmos.color = Color.black;
            Gizmos.DrawCube(mesh_transform.position, mesh_transform.localScale * 1.2f);
        } else {
            // GIzmos to indicate activeness
            Gizmos.color = Color.black;
            Gizmos.DrawWireSphere(mesh_transform.position, neighborhood_radius);
        }
    }
    void OnDrawGizmosSelected() {
        // Return early if not even playing
        if (!Application.isPlaying) return;

        // If we have neighbors, we must query.
        if (neighbor_indices.Count > 0) {
            Gizmos.color = neighbor_color;
            for (int i = 0; i < neighbor_indices.Count; i++) {
                Vector3 neighbor_pos = PedestrianManager.current.agent_positions[neighbor_indices[i]];
                Gizmos.DrawLine(transform.position, neighbor_pos);
            }
        }
    }
    #endif

    public void Init(int agent_id, Vector3 destination) {
        this.agent_id = agent_id;
        this.destination = destination;
        this.desired_direction = (destination - transform.position).normalized;
        this.desired_velocity = this.desired_direction * max_speed;
        this.current_velocity = this.desired_velocity;
        
        this.initialized = true;
    }

    void Update() {
        // Don't do anything if not initialized
        if (!initialized) return;

        Observation();      // Find Neighbors
        //UpdateRVO();        // Conduct RVO for local collision avoidance
        current_velocity = (destination - transform.position).normalized * max_speed;
        //UpdatePosition();   // Update position
        
    }

    private void Observation() {
        // Query neighbors using KDTree
        List<int> tree_indices = new List<int>();
        PedestrianManager.current.QueryNeighbors(transform.position, ref tree_indices);

        // Filter based on angles
        neighbor_indices = new List<int>();
        for(int i = 0; i < tree_indices.Count; i++) {
            // Get angle between position and forward
            float angle = Vector3.Angle(transform.forward, PedestrianManager.current.agent_positions[tree_indices[i]]-transform.position);
            if (angle <= angle_threshold/2f) neighbor_indices.Add(tree_indices[i]);
        }
    }

    private void UpdateRVO() {
        desired_direction = (destination - transform.position).normalized;
        desired_velocity = desired_direction * max_speed;

        Vector3 adjusted_direction = desired_direction;
        float least_distance = Mathf.Infinity;        

        // Iterate through these
        for(int i = 0; i < neighbor_indices.Count; i++) {
            Transform other_t = PedestrianManager.current.agent_transforms[neighbor_indices[i]];
            Pedestrian other_p = PedestrianManager.current.agent_pedestrians[neighbor_indices[i]];
            
            // Ignore if we're looking at ourselves
            if (other_t == this.transform) continue;
            
            // Calculate some relative vectors and values
            Vector3 posA = transform.position;
            Vector3 posB = other_t.position;
            Vector3 dirToOther = posB - posA;
            float dist = dirToOther.magnitude;

            // Ignore if the distance between ourselves and our neighbor is outside the neighborhood range
            if (dist >= neighborhood_radius) continue;

            // Calculate relative velocities
            Vector3 velA = desired_velocity;
            Vector3 velB = other_p.desired_velocity;
            Vector3 relVel = velA - velB;

            // Direction to the other agent. Note that omega == VO cone angle
            float headingToOther = Mathf.Atan2(dirToOther.z, dirToOther.x);
            float omega = Mathf.Asin(Mathf.Clamp(radius / dist, -1f, 1f)); 
            float relHeading = Mathf.Atan2(relVel.z, relVel.x);

            // VO angular cone: [center - omega, center + omega]
            float minAngle = headingToOther - omega;
            float maxAngle = headingToOther + omega;

            // Pseudo-RVO: blend your heading with theirs
            float blendedHeading = Mathf.Atan2(velA.z + velB.z, velA.x + velB.x);
            float rvoHeading = (minAngle + maxAngle + blendedHeading) / 3f; // weighted between min angle, max angle, and blended heading

            // Check if relative heading is inside VO cone
            if (relHeading > minAngle && relHeading < maxAngle) {
                // Nudge away from VO
                adjusted_direction += new Vector3(Mathf.Cos(rvoHeading), 0f, Mathf.Sin(rvoHeading)) * -1f;
                if (dist < least_distance) least_distance = dist;
            }
        }

        // Normalize and adjust current velocity
        adjusted_direction.Normalize();
        float adjusted_speed = (least_distance < neighborhood_radius) ? Mathf.Lerp(0.1f, max_speed, least_distance / neighborhood_radius) : max_speed;
        current_velocity = adjusted_direction * adjusted_speed;
    }

    private void FixedUpdate() {
        // Don't update if the distance between our position and the destination meets a min distance threshold
        initialized = Vector3.Distance(transform.position, destination) > 0.01f;
        if (!initialized) return;

        transform.forward = current_velocity.normalized;
        transform.position += current_velocity * PedestrianManager.current.deltaTime;
    }
    
}
