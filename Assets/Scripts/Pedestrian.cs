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

public class Pedestrian : MonoBehaviour
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
    [HideInInspector]   public Vector3 current_velocity;
    [HideInInspector]   public Vector3 desired_direction; // normalized
    [HideInInspector]   public Vector3 desired_velocity;
    [HideInInspector]   public Vector3 optimal_velocity;
    [HideInInspector]   public Vector3 prev_current_velocity;
    [HideInInspector]   public Vector3 position => this.transform.position;
    [HideInInspector]   public Vector3 velocity => this.optimal_velocity;
    [HideInInspector]   public float radius => this.spatial_radius;

    [Header("=== Non-RVO ===")]
    public bool draw_gizmos = false;
    public float acceleration = 1.5f;
    public float angular_speed = 60f;

    /* =========
    Jobification
    ========= */
    private List<float2> candidate_directions_template;
    private NativeArray<float2> candidate_directions;
    private NativeArray<CandidateDirection> candidate_direction_results;
    private NativeArray<GenerateAgents.AgentData> neighbors;
    private DirectionJob direction_job;
    private JobHandle direction_job_handler;
    private CandidateDirection[] candidate_rankings;

    #if UNITY_EDITOR
    void OnDrawGizmos() {
        // Return early if not even playing or if we disabled gizmos
        if (!draw_gizmos || !Application.isPlaying) return;

        Gizmos.color = Color.black;
        Gizmos.DrawRay(transform.position, current_velocity);
        Gizmos.DrawLine(transform.position, destination);

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

    private void Start() {
        // Confirm destination
        if (generate_destination_on_start) 
            this.destination = (GenerateAgents.current != null) ? GenerateAgents.current.GetRandomPointInBounds() : transform.position;

        // Initialize candidate directions that can be jobified
        candidate_directions_template = GenerateAgents.current.GenerateDirections(num_candidate_directions, 1f, max_speed, 0.5f);
        //candidate_directions_template = GenerateAgents.current.GenerateDirections(num_candidate_directions, max_speed);
        candidate_directions = new NativeArray<float2>(candidate_directions_template.Count+1, Allocator.Persistent);
        candidate_direction_results = new NativeArray<CandidateDirection>(candidate_directions_template.Count+1, Allocator.Persistent);
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
        neighbors = new NativeArray<GenerateAgents.AgentData>(neighbor_data.ToArray(), Allocator.Persistent);
    }

    private void Processing() {
        // Designate the desired direction and velocity
        desired_direction = (destination - transform.position).normalized;
        desired_velocity = desired_direction * max_speed;
        optimal_velocity = desired_velocity;

        // If we have neighbors, identify optimal velocity using RVO
        if (neighbors.Length == 0) return;

        // Update candidate directions with the current desired velocity
        for (int i = 0; i < candidate_directions_template.Count; i++) {
                candidate_directions[i] = candidate_directions_template[i];
                candidate_direction_results[i] = new CandidateDirection(i);
            }
            candidate_directions[candidate_directions_template.Count] = (float2)desired_velocity.ToVector2();
            candidate_direction_results[candidate_directions_template.Count] = new CandidateDirection(candidate_directions_template.Count);
            // Initialize the job
            direction_job = new DirectionJob() {
                candidate_directions = candidate_directions,
                neighbors = neighbors,
                agent_index = agent_index,
                position = (float2)transform.position.ToVector2(),
                current_velocity = (float2)current_velocity.ToVector2(),
                desired_velocity = (float2)desired_velocity.ToVector2(),
                radius = spatial_radius,
                candidate_direction_results = candidate_direction_results
            };
        direction_job_handler = direction_job.Schedule(candidate_directions_template.Count+1, 128);
        JobHandle.ScheduleBatchedJobs();
        direction_job_handler.Complete();

        // Get the penalties as a new array
        candidate_rankings = direction_job.candidate_direction_results.ToArray();
        Array.Sort(candidate_rankings, (v1,v2)=>v1.penalty.CompareTo(v2.penalty));
        optimal_velocity = candidate_directions[candidate_rankings[0].index].ToVector3();
    }

    private void FixedUpdate() {
        Vector3 diff_pos = destination - transform.position;
        initialized = diff_pos.magnitude > 0.1f;
        if (!initialized) return;

        // Rotate the agent to face the direction of the optimal velocity,. but only if the optimal velocity isn't Vector3.zero
        Quaternion target_rotation = (current_velocity != Vector3.zero) 
            ? Quaternion.LookRotation(current_velocity)
            : Quaternion.LookRotation(diff_pos);
        float angle_difference = Quaternion.Angle(transform.rotation, target_rotation);
        float angular_step = angular_speed * Time.fixedDeltaTime;
        // Rotate towards the target rotation but do not overshoot
        if (angular_step > angle_difference) transform.rotation = target_rotation;
        else transform.rotation = Quaternion.RotateTowards(transform.rotation, target_rotation, angular_step);

        // Calcualte the difference between our current velocity and the optimal velocity
        Vector3 vel_diff = current_velocity - prev_current_velocity;
        Vector3 translate_velocity;
        if (vel_diff.sqrMagnitude > 0f) {
            // Calculate the step needed to add to the current velocity
            Vector3 vel_step = vel_diff.normalized * acceleration * Time.fixedDeltaTime;
            // Increment current velocity based on velStep, except in the case that the velocity step overshoots the optimal velocity
            if (vel_step.sqrMagnitude > vel_diff.sqrMagnitude) translate_velocity = current_velocity;
            else translate_velocity = prev_current_velocity + vel_step;
        }
        else {
            translate_velocity = current_velocity;
        }

        // Translate the agent
        transform.position += translate_velocity * Time.fixedDeltaTime;
        prev_current_velocity = translate_velocity;
    }

    private void OnDestroy() {
        direction_job_handler.Complete();
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
        // Outputs
        [WriteOnly] public NativeArray<CandidateDirection> candidate_direction_results;

        // Execute function. We're provided an index.
        // This index is the item index of candidate_directions and candidate_direction_results
        public void Execute(int index) {

            // What's the direction we're checking now?
            float2 candidate_direction = candidate_directions[index];
            
            // Every candidate direction comes with a cost of 0
            float cost = 0f;

            // We have to iterate through all neighbors
            for(int i = 0; i < neighbors.Length; i++) {
                GenerateAgents.AgentData other = neighbors[i];
                if (agent_index == other.agent_index) continue; // Skip if ourselves

                // Calculate translation of VB to VA. RVO-specific
                float2 translate_VB_VA = position + 0.5f * (other.velocity - current_velocity);

                // Calculate diff and its theta
                float2 diff = candidate_direction + position - translate_VB_VA;
                float theta_diff = math.atan2(diff[1], diff[0]);

                // Calculate distance
                float2 pos_diff = other.position - position;
                float distance = math.length(pos_diff);
                float minkowski_radius = radius + other.radius;
                if (minkowski_radius > distance) distance = minkowski_radius;

                // Calculate theta of BA
                float theta_BA = math.atan2(pos_diff[1], pos_diff[0]);

                // Calculate orthogonal of that_BA
                float theta_BAort = math.asin(minkowski_radius / distance);

                // Calculate angular bounds based on theta_BA and theta_BAort
                float theta_left = theta_BA + theta_BAort;
                float theta_right = theta_BA - theta_BAort;
                
                // We need to check if theta_diff is between theta_left and theta_right. If so, this is an unsuitable candidate
                // In this case, we apply a max penalty of 1000
                if (math.abs(theta_right - theta_left) <= math.PI) {
                    if (theta_right <= theta_diff && theta_diff <= theta_left) cost = 1000f;
                } else {
                    // We need to consider the case where the signs of theta_left and theta_right are smaller than 0
                    if (theta_left < 0f) theta_left += 2f * math.PI;
                    if (theta_right < 0f) theta_right += 2f * math.PI;
                    if (theta_diff < 0f) theta_diff += 2f * math.PI;

                    if (theta_left < theta_right) {
                        if (theta_left <= theta_diff && theta_diff <= theta_right) cost = 100f;
                    }
                    else {
                        if (theta_right <= theta_diff && theta_diff <= theta_left) cost = 100f;
                    }
                }

                // Determine the final penalty cost
                cost = math.max(math.length(candidate_direction - desired_velocity), cost);
                if (cost == 100f) break;
            }

            // output result
            candidate_direction_results[index] = new CandidateDirection(index, cost);
            
            /*
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
            */
        }
    }
}
