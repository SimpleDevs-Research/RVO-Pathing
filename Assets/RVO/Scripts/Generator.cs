using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;
using DataStructures.ViliWonka.KDTree;

using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;

/*
This is a script that works to generate a bunch of agents.
It also provides a KDTree implementation for easy querying.
You need to modify some elements on your own too, such as the
agent prefab, the number of agents, etc.
*/

namespace RVO {

    public class Generator : MonoBehaviour
    {
        public static Generator current;

        public enum SpawnStyle { Random, Circular }

        [Header("=== References, Environment Setup ===")]
        [Tooltip("The Transform parent of all agents")]         public Transform agent_parent;
        [Tooltip("The agent prefab that should be spawned")]    public GameObject agent_prefab;
        [Tooltip("The main camera")]                            public Camera scene_cam;
        [Space]
        [Tooltip("The environment bounds from origin")]         public Vector2 bounds = new Vector2(20f,20f);
        [Tooltip("Number of agents to generate")]               public int num_agents = 50;
        [Tooltip("Dist. before agent is considered at dest.")]  public float destination_buffer = 0.1f;
        [Tooltip("Spawn Orientation Setting")]                  public SpawnStyle spawn_orientation = SpawnStyle.Random;
        [Tooltip("Distance between start and end pos. ONLY with a non-random spawn orientation")]   public float spawn_distance = 10f;
        
        [Header("=== RVO Settings ===")]
        [Tooltip("Maximum speed of agents")]                    public float max_speed = 2f;
        [Tooltip("Acceleration of agents")]                     public float acceleration = 5f;
        [Tooltip("How many candidate directions to consider?")] public int num_candidate_directions = 16;
        [Tooltip("How many neighbors to consider?")]            public int max_neighbors = 8;
        [Tooltip("Radius of 'vision' - for neighbor search")]   public float visual_radius = 5f;
        [Tooltip("Radius of agent space")]                      public float spatial_radius = 0.25f;
        [Space]
        [Tooltip("Shared responsibility factor"),Range(0f,1f)]  public float responsibility_factor = 0.5f;
        [Tooltip("Safety factor; TTC weight")]                  public float safety_factor = 1f;
        [Tooltip("Inertia factor; sidedness weight")]           public float inertia_factor = 0f;

        [Header("=== Record-Keeping ===")]
        [Tooltip("End app if all agents reach their destinations")] public bool early_terminate_app = true;
        [Tooltip("Smooth FPS factor"),Range(0f, 1f)]                public float fps_smoothing_factor = 0.25f;
        [Tooltip("Recording toggle")]                               public bool record_data = true;
        //[Tooltip("Recorder for agent data")]                        public CSVWriter agents_writer;
        //[Tooltip("Recorder for FPS")]                               public CSVWriter fps_writer;
        [HideInInspector] public float current_fps;
        [HideInInspector] public float smoothed_fps;

        [Header("=== Gizmos ===")]
        [Tooltip("Draw the boundaries of the simulation space")]    public bool draw_bounds = true;
        [Tooltip("Gizmos color of the bounds")]                     public Color bounds_color = Color.yellow;

        // RVO + JOBS OPERATIONS
        // Native arrays for data-storing
        public NativeArray<Vector2> positions;
        public NativeArray<Vector2> velocities;
        //public NativeArray<float> radii;
        public NativeArray<Vector2> destinations;
        public NativeArray<int> neighbor_indices;
        public NativeArray<int> num_neighbors;
        //public NativeArray<bool> is_colliding;
        public NativeArray<Vector2> new_velocities;
        public NativeArray<bool> reached_destination;
        // Job handles
        JobHandle dependencyJobHandle = default;
        JobHandle rvoJobHandle = default;
        JobHandle velocityJobHandle = default;
        // KDTree and KDQuery for Neighbor Search
        protected KDTree tree;
        protected KDQuery query;
        // You can expose these in the inspector if needed
        [HideInInspector] public Transform[] agent_transforms;
        [HideInInspector] public Vector3[] agent_positions;

        #if UNITY_EDITOR
        protected virtual void OnDrawGizmos() {
            // Draw Bounds
            if (draw_bounds) {
                Vector3 _bounds = new Vector3(bounds.x, 0f, bounds.y);
                Vector3 centroid = _bounds/2f;
                Gizmos.color = bounds_color;
                Gizmos.DrawWireCube(centroid, _bounds);
            }
        }
        #endif

        private void Awake() {
            current = this;

            // If the agent_parent is null, we set to ourselves
            if (agent_parent == null) agent_parent = this.transform;

            // If camera is present, then prep it
            if (scene_cam != null) {
                Vector3 cam_pos = new Vector3(bounds.x/2f, 100f, bounds.y/2f);
                float screen_ratio = (float)Screen.width / (float)Screen.height;
                float target_ratio = bounds.x / bounds.y;
                float ortho_size = (screen_ratio >= target_ratio) 
                    ? bounds.y / 2
                    : bounds.y / 2 * (target_ratio / screen_ratio);
                
                scene_cam.transform.position = cam_pos;
                scene_cam.transform.rotation = Quaternion.LookRotation(Vector3.down, Vector3.up);
                scene_cam.orthographic = true;
                scene_cam.orthographicSize = ortho_size;
            }

            // Initialize our writers
            /*
            if (record_data) {
                agents_writer.Initialize();
                fps_writer.Initialize();
            }
            */

            // We want to generate our agents
            Generate();
        }

        public virtual void Generate() {
            // Step 1: Initialize our native arrays and normal arrays based on `num_agents`
            positions = new NativeArray<Vector2>(num_agents, Allocator.Persistent);
            velocities = new NativeArray<Vector2>(num_agents, Allocator.Persistent);
            //radii = new NativeArray<float>(num_agents, Allocator.Persistent);
            destinations = new NativeArray<Vector2>(num_agents, Allocator.Persistent);
            neighbor_indices = new NativeArray<int>(num_agents*max_neighbors, Allocator.Persistent);
            num_neighbors = new NativeArray<int>(num_agents, Allocator.Persistent);
            //is_colliding = new NativeArray<bool>(num_agents * max_neighbors, Allocator.Persistent);
            new_velocities = new NativeArray<Vector2>(num_agents, Allocator.Persistent);
            reached_destination = new NativeArray<bool>(num_agents, Allocator.Persistent);
            agent_transforms = new Transform[num_agents];
            agent_positions = new Vector3[num_agents];

            // Step 2: Use a FOR loop to instantiate agent values. Set their respective values in our native/normal arrays
            for(int i = 0; i < num_agents; i++) GenerateAgent(i);

             // Initialize our KDTree and Query
            tree = new KDTree(agent_positions, 32);
            query = new KDQuery();
        }

        // By default, the base Generator class will randomize the start and end positions of each agent.
        // if you want to modify this behavior, simply create a child inheritor of this base class and overwrite this with your own `GenerateAgent()` function.
        protected virtual void GenerateAgent(int agent_index) {
            // Step 1a: Generate the start and end positions of our agent's trajectory via randomization
            Vector3 pos, dest;
            switch(spawn_orientation) {
                case SpawnStyle.Circular:
                    Vector3 centroid = new Vector3(bounds.x, 0f, bounds.y) / 2f;
                    float angle_step = 2f * Mathf.PI / num_agents;
                    float theta = agent_index * angle_step;
                    float x = Mathf.Sin(theta);
                    float y = Mathf.Cos(theta);
                    Vector3 start_ray = new Vector3(spawn_distance*x, 0f, spawn_distance*y);
                    pos = centroid + start_ray;
                    dest = centroid - start_ray;
                    break;
                default:
                    // Random
                    pos = Extensions.GenerateRandomVector3(0f, bounds.x, 0f, bounds.y);
                    dest = Extensions.GenerateRandomVector3(0f, bounds.x, 0f, bounds.y);
                    break;
            }
            // Step 1b. Calculate additional properties based on the start and end positions.
            Vector3 diff = dest - pos;
            Vector3 forward = (diff.sqrMagnitude == 0f) ? Vector3.right : diff.normalized;

            // Step 2: Populate our native arrays with these details. Note that we default velocity as a zero vector. 
            //          We also assume all agents have the same spatial radius
            positions[agent_index] = pos.ToVector2();
            velocities[agent_index] = Vector2.zero;
            //radii[agent_index] = spatial_radius;
            destinations[agent_index] = dest.ToVector2();
            // We use a nested for loop because neighbor_indices occupy a set range of spaces in the `neighbor_indices` nativearray buffer.
            for(int j = 0; j < max_neighbors; j++) {
                neighbor_indices[agent_index*max_neighbors+j] = agent_index;
                //is_colliding[agent_index*max_neighbors+j] = false;
            }
            num_neighbors[agent_index] = 0;
            new_velocities[agent_index] = Vector2.zero;
            reached_destination[agent_index] = false;

            // Step 3: Generate agents to represent these in physical world space
            GameObject go = Instantiate(agent_prefab, pos, Quaternion.LookRotation(forward));
            Transform t = go.transform;
            t.parent = agent_parent;
            agent_transforms[agent_index] = t;
            agent_positions[agent_index] = pos;
        }


        // ============================================
        // NOTE: HOW THIS SCRIPT OPERATES
        // This script encompasses 3 distinct levels of a simulation: 
        // 1. OBSERVATION: Agents identify who their closest neighbors are
        // 2. PROCESSING: Agents will determine optimal velocities to move towards based on RVO
        // 3. MOVEMENT: Agents will adjust their positions and current velocities to reflect Step 2.
        // Because this is a base class, we assume that Steps 1 and 2 will be conducted in the Update loop, while Step 3 is done in a LateUpdate loop
        // We provide the base classes for Observation, Processing, and Movement as well.
        // If you want to modify any of these operations, you can create your own inherited child of this script and modify them.
        // ============================================


        // The OBSERVATION step: For each agent, perform a KDTree search.
        protected virtual void Observation () {
            // Iterate through all agents
            for(int i = 0; i < num_agents; i++) {
                // KDTree Sort
                List<KDQuery.DistanceResult<int>> results = new List<KDQuery.DistanceResult<int>>();
                QueryRadiusSort(agent_positions[i], visual_radius, ref results);
                // Process each potential neighbor
                int n_neighbors = 0;
                for(int j = 0; j < results.Count; j++) {
                    int neighbor_index = results[j].data;                   // What's the neighbor index?
                    if (neighbor_index == i) continue;                      // Skip if ourselves
                    // Check if colliding
                    //is_colliding[i*max_neighbors+n_neighbors] = results[j].distance < Mathf.Pow(radii[i] + radii[neighbor_index],2);
                    neighbor_indices[i*max_neighbors+n_neighbors] = neighbor_index;     // Contribute to our neighbor indices
                    n_neighbors += 1;               // increment n_neighbors
                    if (n_neighbors == max_neighbors) break;    // Break immediately if beyond max_neighbors
                }
                // Set our num neighbors
                num_neighbors[i] = n_neighbors;
            }
        }

        // Helper: Query our tree to find all agents within a radius of our original query point, sorted by distance
        public bool QueryRadiusSort(Vector3 query_position, float r, ref List<KDQuery.DistanceResult<int>> results) {
            results = new List<KDQuery.DistanceResult<int>>();
            query.RadiusSort(tree, query_position, r, results);
            return results.Count > 0;
        }


        // The PROCESSING step: Using RVO mechanisms, determine the optimal velocity to move in.
        //                      Note that we require a deltaTime parameter, in case someone is using this in another update cycle
        protected virtual void Processing(float deltaTime) {
            var rvo_job = new RVOJobParallelFor() {
                positions = positions,
                velocities = velocities,
                //radii = radii,
                destinations = destinations,
                neighbor_indices = neighbor_indices,
                num_neighbors = num_neighbors,
                //is_colliding = is_colliding,
                // For now, assume equal radius size among all agents
                deltaTime = deltaTime,
                max_speed = max_speed,
                num_directions = num_candidate_directions,
                max_neighbors = max_neighbors,
                responsibility_factor = responsibility_factor,
                safety_factor = safety_factor,
                inertia_factor = inertia_factor,
                // The output
                new_velocities = new_velocities
            };
            rvoJobHandle = rvo_job.ScheduleByRef(positions.Length, 32, dependencyJobHandle);
            rvoJobHandle.Complete();
        }


        // The MOVEMENT step: Knowing the optimal velocities to move in, adjust the position of each agent.
        protected virtual void Movement(float deltaTime) {
            // Initialize the job data
            var movement_job = new ApplyVelocityJobParallelFor() {
                new_velocities = new_velocities,
                destinations = destinations,
                deltaTime = deltaTime,
                acceleration = acceleration,
                destination_buffer = destination_buffer,
                positions = positions,
                velocities = velocities,
                reached_destination = reached_destination
            };
            velocityJobHandle = movement_job.ScheduleByRef(positions.Length, 32, rvoJobHandle);
            velocityJobHandle.Complete();
        } 


        // In this base class, we call Steps 1 and 2 in the Update loop and Step 3 in the LateUpdate loop.
        protected virtual void Update() {
            Observation();                  // Vision
            Processing(Time.deltaTime);     // Local Collision Avoidance
            Movement(Time.deltaTime);       // Movement
        }
        protected virtual void LateUpdate() {
            // We need an additional step to update our agent_positions and rebuild our KD Tree.
            //      We also consider whether we actually reached all our destinations
            int reached_destination_count = 0;
            for(int i = 0; i < num_agents; i++) {
                Vector3 p = positions[i].ToVector3();
                agent_transforms[i].position = p;
                if (velocities[i].sqrMagnitude > 0f) agent_transforms[i].forward = velocities[i].ToVector3();
                agent_positions[i] = p;
                if (reached_destination[i]) reached_destination_count += 1;
            }
            
            // Rebuild our Tree
            tree.Rebuild();

            // Handle the cse that our `reached_destination_count` matches the total number of agents
            if (early_terminate_app && reached_destination_count == num_agents) {
                #if UNITY_STANDALONE
                    Application.Quit();
                #endif
                #if UNITY_EDITOR
                    UnityEditor.EditorApplication.isPlaying = false;
                #endif
            }
        }

        /*
        protected virtual void LateUpdate() {
            // Calculate current frame count
            int frame = Time.frameCount;
            int destination_count = 0;

            // Update each agent's data
            // We do it here to enable the update loop in each independent agent to conduct Observation and Processing
            for(int i = 0; i < agent_positions.Length; i++) {
                // Let our agent know they can move in the current velocity in their component
                agent_components[i].current_velocity = agent_components[i].velocity;
                
                // Update our arrays
                agent_positions[i] = agent_components[i].position;
                agent_data[i].Update(agent_components[i].position, agent_components[i].velocity);
                //agent_components[i].Movement();
                
                // Record our data
                if (record_data) AddAgentToWriter(frame, i);
                
                // Caching our trajectory data if needed
                if (!agent_components[i].reached_destination) {
                    if (cache_trajectories) agent_trajectories[i].points.Add(new Vector3(agent_positions[i].x, agent_components[i].current_velocity.magnitude, agent_positions[i].z));
                }
                else {
                    destination_count += 1;
                }
            }

            // Recording our FPS
            if (record_data) AddFPSToWriter(frame);

            // We update the KDTree here and now
            tree.Rebuild();

            // Terminate app early if toggled to and if all agents reach their destinations
            if (early_terminate_app && destination_count == agent_positions.Length) {
                #if UNITY_STANDALONE
                    Application.Quit();
                #endif
                #if UNITY_EDITOR
                    UnityEditor.EditorApplication.isPlaying = false;
                #endif
            }
        }
        */


        // ============================================
        // How Multithreading Works in Unity
        // We have two jobs, of `IJobParallelFor` type, that we must execute to achieve parallelization
        // Feel free to modify these jobs at your descrition./
        // ============================================


        // RVO Job
        struct RVOJobParallelFor : IJobParallelFor {
            [ReadOnly] public NativeArray<Vector2> positions;       // List of all positions of all agents
            [ReadOnly] public NativeArray<Vector2> velocities;      // List of all velocities of all agents
            //[ReadOnly] public NativeArray<float> radii;             // List of all radii for all agents
            [ReadOnly] public NativeArray<Vector2> destinations;    // List of all destinations of all agents
            [ReadOnly] public NativeArray<int> neighbor_indices;    // List of, upwards to 8, neighbors of all agents
            [ReadOnly] public NativeArray<int> num_neighbors;       // List of the number of neighbors of all agents
            //[ReadOnly] public NativeArray<bool> is_colliding;       // List of checks for collisions of all agents

            [ReadOnly] public float deltaTime;                      // Time step
            [ReadOnly] public float max_speed;                      // Maximum speed of an agent
            [ReadOnly] public float radius;                         // The expected radius of all agents. Generalize across all agents
            [ReadOnly] public int num_directions;                   // The number of directions we can iterate over
            [ReadOnly] public int max_neighbors;                    // The maximum number of neighbors possible
            [ReadOnly] public float responsibility_factor;          // Weight to control responsibility of an agent
            [ReadOnly] public float safety_factor;                  // Weight to control TTC affect on penalty
            [ReadOnly] public float inertia_factor;                 // Weight to control sidedness preference
            
            // The output
            [WriteOnly] public NativeArray<Vector2> new_velocities;  // The final velocity of an agent

            // helper: calcualte determinatne of two float2's
            public float det(Vector2 a, Vector2 b) { return a.x*b.y - a.y*b.x; }
            // helper: calculate multiple of two float2's into single float
            public float mult(Vector2 a, Vector2 b) { return a.x*b.x + a.y*b.y; }
            // helper: calculate absolute squear of a float2
            public float absSq(Vector2 v) { return mult(v, v); }
            // helper: calculate square (not square root) of a float
            public float sq(float v) { return v*v; }

            // helper: calculate time to collision
            public float TimeToCollision(Vector2 pA, Vector2 Vab, Vector2 pB, float rr) {
                Vector2 ba = pB - pA;
                float sq_diam = sq(rr);
                float Vab2 = absSq(Vab);
                float time;

                float discr = -sq(det(Vab, ba)) + sq_diam * Vab2;
                if (discr > 0f) {
                    /*
                    if (colliding) {
                        time = (mult(Vab, ba) + Mathf.Sqrt(discr)) / Vab2;
                        if (time < 0) time = -100000f;
                    } else {
                        time = (mult(Vab, ba) - Mathf.Sqrt(discr)) / Vab2;
                        if (time < 0f) time = 100000f;
                    }
                    */
                    time = (mult(Vab, ba) - Mathf.Sqrt(discr)) / Vab2;
                    if (time < 0f) time = 100000f;
                } else {
                    /*
                    if (colliding) time = -100000f;
                    else time = 100000f;
                    */
                    time = 100000f;
                }
                return time;
            }

            // helper: process a potential candidate velocity
            public float CalculatePenalty(int index, Vector2 candidate_velocity, Vector2 preferred_velocity, Vector2 pA, Vector2 vA, int n_neighbors) {
                // Initialize the distance cost, time cost, and inertia costs
                float distance_cost = (candidate_velocity - preferred_velocity).sqrMagnitude;
                float time_cost = 100000f;
                float inertia_cost = (candidate_velocity - vA).sqrMagnitude * inertia_factor;
                float ct;
                // Given the candidate velocity, iterate through our neighbors
                for(int j = 0; j < n_neighbors; j++) {
                    // Get the position and velocity of the other agent
                    int neighbor_index = neighbor_indices[index * max_neighbors + j];
                    Vector2 pB = positions[neighbor_index];
                    Vector2 vB = velocities[neighbor_index];
                    // bool colliding = is_colliding[neighbor_index];
                    // calculate time to collision for this agent
                    Vector2 translate_vb_va = (1f/responsibility_factor)*candidate_velocity - (1f-(1f/responsibility_factor))*vA - vB;
                    //float mink_sum = radii[index] + radii[neighbor_index];
                    float mink_sum = radius * 2f;
                    float time = TimeToCollision(pA, translate_vb_va, pB, mink_sum);
                    //float time = TimeToCollision(pA, translate_vb_va, pB, mink_sum, colliding);
                    ct = time;
                    //if (colliding) ct = -(time / deltaTime) - (absSq(candidate_velocity)/sq(max_speed));
                    // If the current time to collision is less than the time cost, then we set it
                    if (ct < time_cost) time_cost = ct;
                }
                // Return the final penalty
                return safety_factor / time_cost + distance_cost + inertia_cost;
            }

            public void Execute(int index) {
                // For agent i, we must determine the preferred velocity
                Vector2 pA = positions[index];
                Vector2 vA = velocities[index];
                int n_neighbors = num_neighbors[index];
                Vector2 preferred_velocity = (destinations[index] - pA).normalized * max_speed;

                // If our n_neighbors is 0... then there's no need to perform the operation.
                if (n_neighbors == 0) {
                    new_velocities[index] = preferred_velocity;
                    return;
                }

                // Let's iterate across potential candidate velocities. For now, intitialize a candiate velocity (Vector2) and minimum penalty
                Vector2 new_velocity = preferred_velocity;
                float min_penalty = CalculatePenalty(index, preferred_velocity, preferred_velocity, pA, vA, n_neighbors);

                // Use a for loop to iterate across multiple possible velocities
                float angleStep = 2f * Mathf.PI / num_directions;
                for (int i = 0; i < num_directions; i++) {
                    // let's increment from max_speed to the closest speed above 0, based on a velocity step
                    float theta = i * angleStep;
                    float x = Mathf.Sin(theta);
                    float y = Mathf.Cos(theta);
                    // increment downwards to at least consider max speed
                    for (float r = max_speed; r > 0f; r -= 0.25f) {
                        // Determine the candidate velocity
                        Vector2 candidate_velocity = new Vector2(r*x, r*y);
                        float est_penalty = CalculatePenalty(index, candidate_velocity, preferred_velocity, pA, vA, n_neighbors);
                        // Override the new velocity if the estimated penalty is smaller than the min penalty
                        if (est_penalty < min_penalty) {
                            min_penalty = est_penalty;
                            new_velocity = candidate_velocity;
                        }
                    }
                }

                // Ultimately, set the new velocity as... the new velocity with the minimum penalty
                new_velocities[index] = new_velocity;

            }
        }

        // Movement Job
        struct ApplyVelocityJobParallelFor : IJobParallelFor {
            // Jobs declare all data that will be accessed in the job
            // By declaring it as read only, multiple jobs are allowed to access the data in parallel
            
            [ReadOnly] public NativeArray<Vector2> new_velocities;
            [ReadOnly] public NativeArray<Vector2> destinations;

            // Delta time must be copied to the job since jobs generally don't have concept of a frame.
            // The main thread waits for the job same frame or next frame, but the job should do work deterministically
            // independent on when the job happens to run on the worker threads.
            [ReadOnly] public float deltaTime;
            [ReadOnly] public float acceleration;
            [ReadOnly] public float destination_buffer;

            public NativeArray<Vector2> positions;   // read and write
            public NativeArray<Vector2> velocities; // read and write
            [WriteOnly] public NativeArray<bool> reached_destination;

            // The code actually running on the job
            public void Execute(int i) {
                // Check early if we reached our destination
                bool at_destination = (positions[i] - destinations[i]).magnitude < destination_buffer;
                if (at_destination) {
                    positions[i] = destinations[i];
                    velocities[i] = Vector2.zero;
                    reached_destination[i] = true;
                    return;
                }

                // Determine the new velocity we should move the agent in
                Vector2 vA = velocities[i];
                Vector2 new_vA = new_velocities[i]; 
                float dv = (new_vA - vA).magnitude;
                if (dv < acceleration * deltaTime) vA = new_vA;
                else vA = (1f - (acceleration * deltaTime / dv)) * vA + (acceleration * deltaTime / dv) * new_vA;

                // Translate our agent. Set to destination if reached our destination
                positions[i] = positions[i] + vA * deltaTime;
                velocities[i] = vA;
                reached_destination[i] = false;
            }
        }



        protected virtual void OnDestroy() {
            //if (agents_writer.is_active) agents_writer.Disable();
            //if (fps_writer.is_active) fps_writer.Disable();

            if (positions.IsCreated) positions.Dispose();
            if (velocities.IsCreated) velocities.Dispose();
            //if (radii.IsCreated) radii.Dispose();
            if (destinations.IsCreated) destinations.Dispose();
            if (neighbor_indices.IsCreated) neighbor_indices.Dispose();
            if (num_neighbors.IsCreated) num_neighbors.Dispose();
            //if (is_colliding.IsCreated) is_colliding.Dispose();
            if (new_velocities.IsCreated) new_velocities.Dispose();
            if (reached_destination.IsCreated) reached_destination.Dispose();
        }

        /*
        protected virtual void AddAgentToWriter(int frame, int i) {
            // Update our writers. Order of columns = frame, agent_index, 2d position, 2d forward, 2d current velocity, 2d optimal_velocity, # neighbors, # candidate_directions
            agents_writer.AddPayload(frame);
            agents_writer.AddPayload(agent_components[i].agent_index);
            agents_writer.AddPayload(agent_positions[i].x);
            agents_writer.AddPayload(agent_positions[i].z);
            agents_writer.AddPayload(agent_components[i].transform.forward.x);
            agents_writer.AddPayload(agent_components[i].transform.forward.z);
            agents_writer.AddPayload(agent_components[i].current_velocity.x);
            agents_writer.AddPayload(agent_components[i].current_velocity.z);
            agents_writer.AddPayload(agent_components[i].optimal_velocity.x);
            agents_writer.AddPayload(agent_components[i].optimal_velocity.z);
            agents_writer.AddPayload(agent_components[i].num_neighbors);
            agents_writer.AddPayload(agent_components[i].num_directions);
            agents_writer.WriteLine();
        }

        protected virtual void AddFPSToWriter(int frame) {
            // Update our FPS writer
            float dt = Time.unscaledDeltaTime;
            current_fps = 1f / dt;
            smoothed_fps = (fps_smoothing_factor * current_fps) + (1f - fps_smoothing_factor) * smoothed_fps;
            fps_writer.AddPayload(frame);
            fps_writer.AddPayload((int)current_fps);
            fps_writer.AddPayload((int)smoothed_fps);
            fps_writer.AddPayload(dt);
            fps_writer.WriteLine();
        }
        
        public Vector3 GetRandomPointInBounds(float y = 0f) {
            return new Vector3(
                Random.Range(0f, bounds.x), 
                y, 
                Random.Range(0f, bounds.y)
            );
        }

        public bool QueryClosest(Vector3 query_position, ref List<int> indices) {
            indices = new List<int>();
            query.ClosestPoint(tree, query_position, indices);
            return indices.Count > 0;
        }
        public bool QueryKNearest(Vector3 query_position, int k, ref List<int> indices) {
            indices = new List<int>();
            query.KNearest(tree, query_position, k, indices);
            return indices.Count > 0;
        }
        public bool QueryRadius(Vector3 query_position, float r, ref List<int> indices) {
            indices = new List<int>();
            query.Radius(tree, query_position, r, indices);
            return indices.Count > 0;
        }
        public bool QueryRadiusSort(Vector3 query_position, float r, ref List<KDQuery.DistanceResult<AgentData>> results) {
            results = new List<KDQuery.DistanceResult<AgentData>>();
            query.RadiusSort<AgentData>(tree, query_position, r, results, agent_data);
            return results.Count > 0;
        }
        public bool QueryInterval(Vector3 query_position, Vector3 interval, ref List<int> indices) {
            indices = new List<int>();
            query.Interval(tree, query_position - interval/2f, query_position + interval/2f, indices);
            return indices.Count > 0;
        }

        public List<float2> GenerateDirections(int n, float r) {
            List<float2> directions = new List<float2>();
            float angleStep = 2f*Mathf.PI / n;
            for(int i = 0; i < n; i++) {
                float theta = i * angleStep;
                directions.Add(new(r * Mathf.Sin(theta), r * Mathf.Cos(theta)));
            }
            return directions;
        }

        public List<float2> GenerateDirections(int n, float min_r, float max_r, float r_iterstep = 0.1f) {
            List<float2> directions = new List<float2>();
            float angleStep = 2f*Mathf.PI / n;
            for(int i = 0; i < n; i++) {
                float theta = i * angleStep;
                float x = Mathf.Sin(theta);
                float y = Mathf.Cos(theta);
                for (float r = min_r; r < max_r; r += r_iterstep) {
                    directions.Add(new(r * x, r * y));
                }
                directions.Add(new(max_r * x, max_r * y));
            }
            return directions;
        }
        */

    }

    public static class Extensions {
        // Helper: Generate N directions with radius R
        public static Vector2[] GenerateDirections(int n, float r) {
            Vector2[] directions = new Vector2[n];
            float angleStep = 2f * Mathf.PI / n;
            for(int i = 0; i < n; i++) {
                float theta = i * angleStep;
                directions[i] = new Vector2(r * Mathf.Sin(theta), r * Mathf.Cos(theta));
            }
            return directions;
        }
        // Helper: generate a random Vector3 within this manager's bounds
        public static Vector3 GenerateRandomVector3(float min_x, float max_x, float min_y, float max_y) { 
            return new Vector3(Random.Range(min_x, max_x), 0f, Random.Range(min_y, max_y));  
        }
        // Helper: Convert a Vector3 to a Vector2 based on the X and Z coordinates
        public static Vector2 ToVector2(this Vector3 v) {   
            return new Vector2(v.x, v.z);
        }
        // Helper: Convert a Vector2 into a Vector3 by setting the X and Z coordinates
        public static Vector3 ToVector3(this Vector2 v, float y=0f) { 
            return new Vector3(v.x, y, v.y); 
        }
        // Helper: convert a float2 into a Vector3 by setting the 0thand 1st coordinates
        public static Vector3 ToVector3(this float2 v, float y = 0f) {
            return new Vector3(v[0], y, v[1]);
        }

    }


}