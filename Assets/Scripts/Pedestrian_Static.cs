using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;

#if UNITY_EDITOR
using UnityEditor;
#endif

public class Pedestrian_Static : MonoBehaviour
{
    [System.Serializable]
    public struct CandidateDirection {
        public int index;
        public float penalty;
        public CandidateDirection(int index, float penalty=0f) {
            this.index = index;
            this.penalty = penalty;
        }
    }

    [Header("=== Agent Settings ===")]
    public bool generate_destination_on_start = true;
    public Vector3 destination;
    public bool initialized = false;

    [Header("=== Neighbor Querying ===")]
    public float angle_threshold = 60f;

    [Header("=== RVO ===")]
    [HideInInspector] public int agent_index;
    public int num_candidate_directions = 32;
    public float max_speed = 2.5f;
    public float visual_radius = 0.5f;
    public float spatial_radius = 3f;
    public float aggression = 1f;
    [HideInInspector]   public Vector3 current_velocity;
    [HideInInspector]   public Vector3 desired_direction; // normalized
    [HideInInspector]   public Vector3 desired_velocity;
    [HideInInspector]   public Vector3 position => this.transform.position;
    [HideInInspector]   public Vector3 velocity => this.current_velocity;
    [HideInInspector]   public float radius => this.spatial_radius;

    [Header("=== Non-RVO ===")]
    public float acceleration = 1.5f;
    public float angular_speed = 60f;

    /* =========
    Jobification
    ========= */
    private NativeArray<float2> candidate_directions;
    private NativeArray<CandidateDirection> candidate_direction_results;
    private NativeArray<GenerateAgents.AgentData> neighbors;
    private DirectionJob direction_job;
    private JobHandle direction_job_handler;
    public CandidateDirection[] candidate_rankings;

    /*
    #if UNITY_EDITOR
    void OnDrawGizmosSelected() {
        // Return early if not even playing
        if (!Application.isPlaying) return;

        Gizmos.color = Color.blue;
        for(int i = 0; i < num_candidate_directions; i++) {
            float2 candidate_dir = candidate_directions[candidate_rankings[i].index];
            float penalty = candidate_rankings[i].penalty;
            Gizmos.DrawRay(transform.position, candidate_dir.ToVector3() * (1f-penalty));
        }
        Gizmos.color = Color.red;
        Gizmos.DrawRay(transform.position, current_velocity);
    }
    #endif
    */

    private void Start() {
        // Confirm destination
        if (generate_destination_on_start) 
            this.destination = (GenerateAgents.current != null) ? GenerateAgents.current.GetRandomPointInBounds() : transform.position;

        // Initialize candidate directions that can be jobified
        float2[] candidate_directions_template = GenerateAgents.current.GenerateDirections(num_candidate_directions, max_speed);
        candidate_directions = new NativeArray<float2>(num_candidate_directions, Allocator.Persistent);
        candidate_direction_results = new NativeArray<CandidateDirection>(num_candidate_directions, Allocator.Persistent);
        for(int i = 0; i < num_candidate_directions; i++) {
            candidate_directions[i] = candidate_directions_template[i];
            candidate_direction_results[i] = new CandidateDirection(i);
        }
    }

    private void Update() {
        Observation();
        Processing();
    }

    private void Observation() {
        // KDTree Query
        List<int> result_indices = new List<int>();
        GenerateAgents.current.QueryRadius(transform.position, 5f, ref result_indices);

        List<GenerateAgents.AgentData> neighbor_data = new List<GenerateAgents.AgentData>();
        for(int i = 0; i < result_indices.Count; i++) {
            GenerateAgents.AgentData other = GenerateAgents.current.agent_data[result_indices[i]];
            Vector2Int a = new Vector2Int(Mathf.RoundToInt(transform.forward.x*10), Mathf.RoundToInt(transform.forward.z*10));
            Vector2Int b = new Vector2Int(Mathf.RoundToInt((other.position[0] - transform.position.x)*10), Mathf.RoundToInt((other.position[1] - transform.position.z) * 10));
            int dot = a.x * b.x + a.y * b.y;
            if (dot * 4 > -1 * (a.magnitude * b.magnitude)) {
                neighbor_data.Add(other);
            }
        }

        // Updating our neighbor_indices nativearray
        neighbors = new NativeArray<GenerateAgents.AgentData>(neighbor_data.ToArray(), Allocator.TempJob);
    }

    private void Processing() {
        // Designate the desired direction and velocity
        desired_direction = (destination - transform.position).normalized;
        desired_velocity = desired_direction * max_speed;

        // If we have neighbors, identify optimal velocity using RVO
        if (neighbors.Length > 0) {
             // Initialize the job
            direction_job = new DirectionJob() {
                candidate_directions = candidate_directions,
                neighbors = neighbors,
                agent_index = agent_index,
                position = (float2)transform.position.ToVector2(),
                current_velocity = (float2)current_velocity.ToVector2(),
                desired_velocity = (float2)desired_velocity.ToVector2(),
                radius = spatial_radius,
                max_speed = max_speed,
                aggressiveness = aggression,
                candidate_direction_results = candidate_direction_results
            };
            direction_job_handler = direction_job.Schedule(num_candidate_directions, 16);
            JobHandle.ScheduleBatchedJobs();
            direction_job_handler.Complete();

            // Get the penalties as a new array
            candidate_rankings = direction_job.candidate_direction_results.ToArray();
            Array.Sort(candidate_rankings, (v1,v2)=>v1.penalty.CompareTo(v2.penalty));
            desired_velocity = candidate_directions[candidate_rankings[0].index].ToVector3();
        }
    }

    private void FixedUpdate() {
        Vector3 diff_pos = destination - transform.position;
        initialized = diff_pos.magnitude > 0.1f;
        if (!initialized) return;

        // Rotate the agent to face the direction of the optimal velocity,. but only if the optimal velocity isn't Vector3.zero
        Quaternion target_rotation = (desired_velocity != Vector3.zero) 
            ? Quaternion.LookRotation(desired_velocity)
            : Quaternion.LookRotation(diff_pos);
        float angle_difference = Quaternion.Angle(transform.rotation, target_rotation);
        float angular_step = angular_speed * Time.fixedDeltaTime;
        // Rotate towards the target rotation but do not overshoot
        if (angular_step > angle_difference) transform.rotation = target_rotation;
        else transform.rotation = Quaternion.RotateTowards(transform.rotation, target_rotation, angular_step);

        // Calcualte the difference between our current velocity and the optimal velocity
        Vector3 diff = desired_velocity - current_velocity;

        // As long as there is a different in the two velocities, we HAVE to translate.
        if (diff.sqrMagnitude > 0f) {
            // Calculate the step needed to add to the current velocity
            Vector3 vel_step = diff.normalized * acceleration * Time.fixedDeltaTime;
            // Increment current velocity based on velStep, except in the case that the velocity step overshoots the optimal velocity
            if (vel_step.sqrMagnitude > diff.sqrMagnitude) current_velocity = desired_velocity;
            else current_velocity += vel_step;
        }

        // Translate the agent
        transform.position += current_velocity * Time.fixedDeltaTime;
    }

    private void OnDestroy() {
        if (candidate_directions.IsCreated) candidate_directions.Dispose();
        if (candidate_direction_results.IsCreated) candidate_direction_results.Dispose();
        if (neighbors.IsCreated) neighbors.Dispose();
    }



    /* ===================================================
    WE'RE IN JOB TERRITORY NOW. BURST COMPILATION YEAHHHH
    =================================================== */
    [BurstCompile(CompileSynchronously = true)]
    public struct DirectionJob : IJobParallelFor {
        // Inputs
        // - Array data that we iterate over
        [ReadOnly] public NativeArray<float2> candidate_directions;
        [ReadOnly] public NativeArray<GenerateAgents.AgentData> neighbors;
        // - Properties about this specific agent
        [ReadOnly] public int agent_index;  // Unique id for this agent
        [ReadOnly] public float2 position;  // 2D position in world space
        [ReadOnly] public float2 current_velocity;  // Thecurrent 2D velocity in world space
        [ReadOnly] public float2 desired_velocity;  // The desired velocity this agent wants to move towards
        [ReadOnly] public float radius;     // Radius of this agent
        [ReadOnly] public float max_speed;  // Maximum speed of this agent
        [ReadOnly] public float aggressiveness; // Aggressiveness of this agent
        // Outputs
        [WriteOnly] public NativeArray<CandidateDirection> candidate_direction_results;

        // Execute function. We're provided an index.
        // This index is the item index of candidate_directions and candidate_direction_results
        public void Execute(int index) {
            // Primers
            float2 candidate_direction = candidate_directions[index];
            float2 potential = (2f * candidate_direction) - current_velocity;
            float base_penalty = math.length(math.normalize(desired_velocity) - math.normalize(candidate_direction));
            float penalty = 0f;

            // iterate through all neighbors.
            for(int i = 0; i < neighbors.Length; i++) {
                GenerateAgents.AgentData other = neighbors[i];
                if (agent_index == other.agent_index) continue; // Skip if ourselves

                // RVO requires us to add a minkowski sum of radii between this agent and the other
                float minkowski_radius = radius + other.radius;

                // Let's calculate the left and right angle bounds of this agent
                float2 diff_pos = other.position - position;                        // Calculate vector from here to neighbor
                float distance = math.max(math.length(diff_pos), minkowski_radius); // We cap the possible distance to the minkowski radius
                float theta = math.atan2(diff_pos[1], diff_pos[0]);                 // Relative to X+ axis, get the angle of this pos. diff. vector
                float theta_ort = math.asin(minkowski_radius / distance);           // Get the orthogonal angle to consider left and right 
                float theta_left = math.degrees(theta + theta_ort) + 360f;                 // Calculate the left angle bound.
                float theta_right = math.degrees(theta - theta_ort) + 360f;                // Calculate the right angle bound.

                // Let's calculate the potential diff and its own theta
                float2 translate_pos = position + other.velocity;   // RVO is different - we estimate the positional translation given the other's velocity
                float time_cost = distance / max_speed;             // Time cost is how fast it'll take for us to translate to the other, given our max speed
                float2 diff = potential + position - translate_pos; // Hard to describe in a single line, just know it's RVO-specific
                float theta_diff = math.degrees(math.atan2(diff[1], diff[0])) + 360f;    // Given diff, let's calculate its angle

                // Validity check!
                if (theta_right <= theta_diff && theta_diff <= theta_left) {
                    // This is an invalid candidate, as it lies within VO of collision. What is its penalty?
                    //penalty = math.max(penalty, (1f+aggressiveness)/time_cost);
                    penalty = 1000f;
                }
            }
            
            // Output penalty to results. Just make sure to not adjust the order in the CPU later.
            candidate_direction_results[index] = new CandidateDirection(index, base_penalty + penalty);
        }
    }
}
