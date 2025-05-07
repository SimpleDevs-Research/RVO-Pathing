using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using DataStructures.ViliWonka.KDTree;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace RVO {
    public class Agent : MonoBehaviour {

        [Header("=== Agent Settings ===")]
        public bool generate_destination_on_start = true;
        public Vector3 destination;

    [Header("=== RVO ===")]
    public int agent_index;
    public int num_candidate_directions = 32;
    public bool multi_speed_candidates = true;
    public float min_speed = 0.5f;
    public float max_speed = 2.5f;
    public float speed_step = 0.5f;
    public float visual_radius = 0.5f;
    public float spatial_radius = 3f;
    public float stopping_distance = 0.05f;
    public float safety_factor = 1f;
    public int max_neighbors = 8;
    [HideInInspector]   public Vector3 current_velocity;
    [HideInInspector]   public Vector3 desired_direction; // normalized
    [HideInInspector]   public Vector3 desired_velocity;
    [HideInInspector]   public Vector3 optimal_velocity;
    [HideInInspector]   public Vector3 prev_current_velocity;
    [HideInInspector]   public Vector3 position => this.transform.position;
    [HideInInspector]   public Vector3 velocity => this.optimal_velocity;
    [HideInInspector]   public float radius => this.spatial_radius;

    [Header("=== Non-RVO ===")]
    public bool simulate_vision = true;
    public float acceleration = 1.5f;
    public float angular_speed = 60f;

    [Header("=== Gizmos ===")]
    public bool draw_gizmos = false;

    [Header("=== Read-Only Data ===")]
    public float distance_to_destination = 0f;
    public bool reached_destination = false;
    public bool colliding = false;
    public int num_neighbors = 0;
    public int num_directions = 0;

    /* =========
    Jobification
    ========= */
    private List<float2> candidate_directions_template;
    private NativeArray<float2> candidate_directions;
    private NativeArray<CandidateDirection> candidate_direction_results;
    private NativeArray<AgentData> neighbors;
    private DirectionJob direction_job;
    private JobHandle direction_job_handler;
    [SerializeField] private CandidateDirection[] candidate_rankings;

    #if UNITY_EDITOR
    protected virtual void OnDrawGizmos() {
        // Return early if not even playing or if we disabled gizmos
        if (!draw_gizmos || !Application.isPlaying) return;

        Gizmos.color = Color.black;
        Gizmos.DrawWireSphere(transform.position, this.spatial_radius);

        //Gizmos.color = Color.black;
        //Gizmos.DrawRay(transform.position, current_velocity);
        //Gizmos.DrawLine(transform.position, destination);

        Gizmos.color = Color.blue;
        for(int i = 0; i < num_candidate_directions; i++) {
            float2 candidate_dir = candidate_directions[candidate_rankings[i].index];
            float penalty = candidate_rankings[i].penalty;
            Gizmos.DrawRay(transform.position, candidate_dir.ToVector3() * (1f-penalty));
        }
        Gizmos.color = Color.red;
        Gizmos.DrawRay(transform.position, current_velocity);

        Gizmos.color = Color.black;
        for(int i = 0; i < num_neighbors; i++) {
            Gizmos.DrawLine(transform.position, neighbors[i].position.ToVector3());
        }

    }
    #endif

    protected virtual void Start() {
        // Set our scale
        transform.localScale = Vector3.one * this.spatial_radius;
        // Confirm destination
        if (generate_destination_on_start) 
            this.destination = (Generator.current != null) ? Generator.current.GetRandomPointInBounds() : transform.position;

        // Initialize candidate directions that can be jobified
        if (multi_speed_candidates) {
            candidate_directions_template = Generator.current.GenerateDirections(num_candidate_directions, min_speed, max_speed, speed_step);
        } else {
            candidate_directions_template = Generator.current.GenerateDirections(num_candidate_directions, max_speed);
        }
        num_directions = candidate_directions_template.Count+1;
        candidate_directions = new NativeArray<float2>(num_directions, Allocator.Persistent);
        candidate_direction_results = new NativeArray<CandidateDirection>(num_directions, Allocator.Persistent);
        neighbors = new NativeArray<AgentData>(max_neighbors, Allocator.Persistent);
    }

    protected virtual void Update() {
        CalculateDesiredVelocity();
        Observation();
        Processing();
    }
    
    protected virtual void FixedUpdate() {
        Movement();
    }

    public virtual void CalculateDesiredVelocity() {
        // Designate the desired direction and velocity
        Vector3 dest_diff = destination - transform.position;
        desired_direction = dest_diff.normalized;
        desired_velocity = desired_direction * max_speed;
    }
    
    public virtual void Observation() {
        // The funky thing is that in the RVO library from van der Berg, they consider an additional `rangeSqr` value
        // This value is the minimum of either (this.visual_radius^2) or (max(deltaTime, maxSpeed/maxAcceleration) * maxSpeed * this.spatial_radius)
        float range_sq = Mathf.Min(
            Mathf.Pow(this.visual_radius,2),
            Mathf.Max(Time.unscaledDeltaTime, this.max_speed/this.acceleration) * this.max_speed * this.spatial_radius
        );

        // KDTree Query using radial sort. Results should output DistanceResults, which is a custom class outputted by KDQuery.
        List<KDQuery.DistanceResult<AgentData>> results = new List<KDQuery.DistanceResult<AgentData>>();
        Generator.current.QueryRadiusSort(transform.position, this.visual_radius, ref results);

        // Pre-set our colliding handle to false
        colliding = false;

        //List<AgentData> neighbor_data = new List<AgentData>();
        int n_neighbors = 0;
        foreach(KDQuery.DistanceResult<AgentData> result in results) {
            AgentData other = result.data;
            if (agent_index == other.agent_index) continue;

            // We need to check if we're colliding with this agent.
            // the DistanceResult should tell us how far away the other agent is from us already, in sqr meters
            if (result.distance < Mathf.Pow(other.radius + this.radius, 2)) {
                // We're colliding. Let's do some stuff
                if (!colliding) {
                    colliding = true;
                    n_neighbors = 0;
                }
                // If we don't simulate vision, then do simple stuff
                if (!simulate_vision) {
                    neighbors[n_neighbors] = other;
                    n_neighbors += 1;
                    if (n_neighbors >= 8) break;    // end early if we've achieved 8 closest visible people.
                    continue;
                }
                Vector2Int a = new Vector2Int(Mathf.RoundToInt(transform.forward.x*10), Mathf.RoundToInt(transform.forward.z*10));
                Vector2Int b = new Vector2Int(Mathf.RoundToInt((other.position[0] - transform.position.x)*10), Mathf.RoundToInt((other.position[1] - transform.position.z) * 10));
                int dot = a.x * b.x + a.y * b.y;
                if (dot * 4 > -1 * (a.magnitude * b.magnitude)) {
                    neighbors[n_neighbors] = other;
                    n_neighbors += 1;
                    if (n_neighbors >= 8) break;    // end early if we've achieved 8 closest visible people.
                }
            }
            // else if case: if we aren't colliding yet and we still find someone, then we have to add them as an RVO neighbor
            else if (!colliding) {
                if (!simulate_vision) {
                    neighbors[n_neighbors] = other;
                    n_neighbors += 1;
                    if (n_neighbors >= 8) break;    // end early if we've achieved 8 closest visible people.
                    continue;
                }
                Vector2Int a = new Vector2Int(Mathf.RoundToInt(transform.forward.x*10), Mathf.RoundToInt(transform.forward.z*10));
                Vector2Int b = new Vector2Int(Mathf.RoundToInt((other.position[0] - transform.position.x)*10), Mathf.RoundToInt((other.position[1] - transform.position.z) * 10));
                int dot = a.x * b.x + a.y * b.y;
                if (dot * 4 > -1 * (a.magnitude * b.magnitude)) {
                    neighbors[n_neighbors] = other;
                    n_neighbors += 1;
                    if (n_neighbors >= 8) break;    // end early if we've achieved 8 closest visible people.
                }
            }
        }

        // Updating our neighbor_indices nativearray
        num_neighbors = n_neighbors;

        // By this point, if we detected that we were colliding with someone, then our neighbors are only those we're colliding with.
        // Alternatively, if we haven't detected any collisions, then we should only be considering those that are within our range of detection.
    }

    public virtual void Processing() {
        optimal_velocity = desired_velocity;

        // If we have neighbors, identify optimal velocity using RVO
        if (num_neighbors == 0) return;

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
            max_speed = max_speed,
            radius = spatial_radius,
            safety_factor = safety_factor,
            num_neighbors = num_neighbors,
            colliding = colliding,
            dt = Time.unscaledDeltaTime,
            candidate_direction_results = candidate_direction_results
        };
        /*
        direction_job_handler = direction_job.Schedule(candidate_directions_template.Count+1, 128);
        JobHandle.ScheduleBatchedJobs();
        direction_job_handler.Complete();
        */
        direction_job.Run(candidate_directions.Length);

        // Get the penalties as a new array
        candidate_rankings = direction_job.candidate_direction_results.ToArray();
        Array.Sort(candidate_rankings, (v1,v2)=>v1.penalty.CompareTo(v2.penalty));
        optimal_velocity = candidate_directions[candidate_rankings[0].index].ToVector3();
    }

    public virtual void Movement() {
        Vector3 diff_pos = destination - transform.position;
        distance_to_destination = diff_pos.magnitude;
        reached_destination = distance_to_destination <= stopping_distance;
        if (reached_destination) {
            transform.position = destination;
            return;
        }

        // Rotate the agent to face the direction of the optimal velocity,. but only if the optimal velocity isn't Vector3.zero
        Quaternion target_rotation = (current_velocity != Vector3.zero) 
            ? Quaternion.LookRotation(current_velocity)
            : Quaternion.LookRotation(diff_pos);
        float angle_difference = Quaternion.Angle(transform.rotation, target_rotation);
        float angular_step = angular_speed * Time.fixedDeltaTime;
        // Rotate towards the target rotation but do not overshoot
        if (angular_step > angle_difference) transform.rotation = target_rotation;
        else transform.rotation = Quaternion.RotateTowards(transform.rotation, target_rotation, angular_step);

        // Calculate the difference between our current velocity and the optimal velocity
        Vector3 translate_velocity;
        if (acceleration > 0f) {
            Vector3 vel_diff = current_velocity - prev_current_velocity;
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
        } else {
            translate_velocity = current_velocity;
        }

        // Translate the agent
        transform.position += translate_velocity * Time.fixedDeltaTime;
        prev_current_velocity = translate_velocity;
    }

    protected virtual void OnApplicationQuit() {
        direction_job_handler.Complete();
        if (candidate_directions.IsCreated) candidate_directions.Dispose();
        if (candidate_direction_results.IsCreated) candidate_direction_results.Dispose();
        if (neighbors.IsCreated) neighbors.Dispose();
    }

    public virtual void SetDestination(Vector3 d) {
        this.destination = d;
    }



    /* ===================================================
    WE'RE IN JOB TERRITORY NOW. BURST COMPILATION YEAHHHH
    =================================================== */
    [BurstCompile(CompileSynchronously = true)]
    public struct DirectionJob : IJobParallelFor {
        // Inputs
        // - Array data that we iterate over
        [ReadOnly] public NativeArray<float2> candidate_directions;
        [ReadOnly] public NativeArray<AgentData> neighbors;
        // - Properties about this specific agent
        [ReadOnly] public int agent_index;  // Unique id for this agent
        [ReadOnly] public float2 position;  // 2D position in world space
        [ReadOnly] public float2 current_velocity;  // Thecurrent 2D velocity in world space
        [ReadOnly] public float2 desired_velocity;  // The desired velocity this agent wants to move towards
        [ReadOnly] public float max_speed;      // max speed the agent wants to move
        [ReadOnly] public float radius;         // Radius of this agent
        [ReadOnly] public float safety_factor;  // The agent's willingness to be safe.
        [ReadOnly] public int num_neighbors;    // Number of detected neighbors
        [ReadOnly] public bool colliding;       // Are we currently colliding with any neighbors?
        [ReadOnly] public float dt;
        // Outputs
        [WriteOnly] public NativeArray<CandidateDirection> candidate_direction_results;

        // helper: calcualte determinatne of two float2's
        public float det(float2 a, float2 b) { return a[0]*b[1] - a[1]*b[0]; }
    
        // helper: calculate multiple of two float2's into single float
        public float mult(float2 a, float2 b) { return a[0]*b[0] + a[1]*b[1]; }

        // helper: calculate absolute squear of a float2
        public float absSq(float2 v) { return mult(v, v); }

        // helper: calculate square (not square root) of a float
        public float sq(float v) { return v*v; }

        // helper: calculate time to collision
        public float TimeToCollision(float2 pA, float2 Vab, float2 pB, float rr, bool collision) {
            float2 ba = pB - pA;
            float sq_diam = sq(rr);
            float Vab2 = absSq(Vab);
            float time;

            float discr = -sq(det(Vab, ba)) + sq_diam * Vab2;
            if (discr > 0f) {
                if (collision) {
                    time = (mult(Vab, ba) + math.sqrt(discr)) / Vab2;
                    if (time < 0) time = -1000000f;
                } else {
                    time = (mult(Vab, ba) - math.sqrt(discr)) / Vab2;
                    if (time < 0) time = 1000000f;
                }
            } else {
                if (collision) time = -1000000f;
                else time = 1000000f;
            }
            return time;
        }

        // Execute function. We're provided an index.
        // This index is the item index of candidate_directions and candidate_direction_results
        public void Execute(int index) {

            // What's the direction we're checking now?
            float2 candidate_direction = candidate_directions[index];

            // Calculate the distance cost and time cost
            float distance_cost = 0f;
            if (!colliding) distance_cost = math.length(candidate_direction - desired_velocity);
            float time_cost = 1000000f;

            // We have to iterate through all neighbors
            for(int i = 0; i < num_neighbors; i++) {
                AgentData other = neighbors[i]; 

                // Priming the time to collision for this specific agent
                float ct;

                // calculate time to collision for this agent
                float2 translate_vb_va = 2f * candidate_direction - current_velocity - other.velocity;
                float mink_sum = radius + other.radius;
                float time = TimeToCollision(position, translate_vb_va, other.position, mink_sum, colliding);

                // mod time to collision for this current agent with additional metrics, based on if we're colliding or not
                if (colliding) ct = -math.ceil(time / dt) - (absSq(candidate_direction)/sq(max_speed));
                else ct = time;

                // If the current time to collision is less than the time cost, then we set it
                if (ct < time_cost) time_cost = ct;

                /*
                // Calculate translation of VB to VA. RVO-specific
                float2 translate_VB_VA = position + 2f * (other.velocity - current_velocity);

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

                // Calculate some penalty values. By default, the penalty is the distance between the desired velocity and canidate
                float penalty = math.length(candidate_direction - desired_velocity);
                float distance_cost = 10f / distance;
                
                // We need to check if theta_diff is between theta_left and theta_right. If so, this is an unsuitable candidate
                // In this case, we apply a max penalty of 1000
                if (math.abs(theta_right - theta_left) <= math.PI) {
                    if (theta_right <= theta_diff && theta_diff <= theta_left) penalty += 100f;
                } else {
                    // We need to consider the case where the signs of theta_left and theta_right are smaller than 0
                    if (theta_left < 0f) theta_left += 2f * math.PI;
                    if (theta_right < 0f) theta_right += 2f * math.PI;
                    if (theta_diff < 0f) theta_diff += 2f * math.PI;

                    if (theta_left < theta_right) {
                        if (theta_left <= theta_diff && theta_diff <= theta_right) penalty += 100f;
                    }
                    else {
                        if (theta_right <= theta_diff && theta_diff <= theta_left) penalty += 100f;
                    }
                }

                // Determine the final penalty cost
                cost = math.max(penalty, cost);
                if (cost >= 100f) break;
                */
            }

            // ultimately, after considering all neighbor,s calculate the final penalty cost
            float penalty = safety_factor / time_cost + distance_cost;

            // output result
            candidate_direction_results[index] = new CandidateDirection(index, distance_cost, time_cost, penalty);
        }
    }
}
}
