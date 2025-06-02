using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;
using DataStructures.ViliWonka.KDTree;

using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine.Jobs;
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

        public enum SpawnStyle { Random, Rows, Circular }
        public enum RVOMethod { RVO, HRVO }

        [Header("=== References, Environment Setup ===")]
        [Tooltip("The Transform parent of all agents")]         public Transform agent_parent;
        [Tooltip("The agent prefab that should be spawned")]    public GameObject agent_prefab;
        [Tooltip("The main camera (Optional)")]                 public Camera scene_cam;
        [Tooltip("The floor transform (Optional)")]             public Transform floor;
        [Space]
        [Tooltip("The environment bounds from origin")]         public Vector2 bounds = new Vector2(20f,20f);
        [Tooltip("Number of agents to generate")]               public int num_agents = 50;
        [Tooltip("Dist. before agent is considered at dest.")]  public float destination_buffer = 0.1f;
        [Tooltip("Spawn Orientation Setting")]                  public SpawnStyle spawn_orientation = SpawnStyle.Random;
        [Tooltip("Distance between start and end pos. ONLY with a non-random spawn orientation")]   public float bound_edge_buffer = 10f;
        
        [Header("=== RVO Global Settings ===")]
        [Tooltip("Which RVO style should we use?")]             public RVOMethod rvo_method = RVOMethod.RVO;
        [Tooltip("All demographic groups used when generating agents. Note that the demographic groups' chances must total to 100")]
        public Demographics demographics;
        [Space]
        [Tooltip("How many candidate directions to consider?")] public int num_candidate_directions = 16;
        [Tooltip("How many neighbors to consider?")]            public int max_neighbors = 8;
        [Tooltip("Radius of 'vision' - for neighbor search")]   public float visual_radius = 5f;
        [Space]

        //[Header("=== Non-RVO ===")]
        /*
        [Tooltip("Should we simulate vision?")]                 public bool simulate_vision; 
        [Tooltip("Cone of Vision Hemisphere")]                  public float cone_of_vision_hemisphere = 180f;
        */
        //[Tooltip("End app if all agents reach their destinations")] public bool early_terminate_app = true;
        //[Tooltip("Smooth FPS factor"),Range(0f, 1f)]                public float fps_smoothing_factor = 0.25f;
        //[Tooltip("Recording toggle")]                               public bool record_data = true;
        //[Tooltip("Recorder for agent data")]                        public CSVWriter agents_writer;
        //[Tooltip("Recorder for FPS")]                               public CSVWriter fps_writer;
        //[HideInInspector] public float current_fps;
        //[HideInInspector] public float smoothed_fps;

        [Header("=== Gizmos ===")]
        [Tooltip("Draw the boundaries of the simulation space")]    public bool draw_bounds = true;
        [Tooltip("Gizmos color of the bounds")]                     public Color bounds_color = Color.yellow;

        // JOBS OPERATIONS
        public VO_OP vo_op;
        protected JobHandle velocityJobHandle = default;
        // KDTree and KDQuery for Neighbor Search
        protected KDTree tree;
        protected KDQuery query;

        // Outputs
        protected Vector3[] agent_positions;

        #if UNITY_EDITOR
        // Drawing boundaries
        protected virtual void OnDrawGizmos() {
            if (!draw_bounds) return;
            Vector3 _bounds = new Vector3(bounds.x, 0f, bounds.y);
            Vector3 centroid = _bounds/2f;
            Gizmos.color = bounds_color;
            Gizmos.DrawWireCube(centroid, _bounds);
        }
#endif

        private void Awake()
        {
            current = this;

            // If the agent_parent is null, we set to ourselves
            if (agent_parent == null) agent_parent = this.transform;

            // If camera is assigned, then prep it
            if (scene_cam != null)
            {
                Vector3 cam_pos = new Vector3(bounds.x / 2f, 100f, bounds.y / 2f);
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

            // If the floor is assigned, then prep it
            if (floor != null)
            {
                floor.rotation = Quaternion.Euler(90f, 0f, 0f);
                floor.position = new Vector3(bounds.x / 2f, 0f, bounds.y / 2f);
                floor.localScale = new Vector3(bounds.x, bounds.y, 0f);
            }

            // Initialize our writers
            /*
            if (record_data) {
                agents_writer.Initialize();
                fps_writer.Initialize();
            }
            */

            // Determine the VO operation to use
            if (rvo_method == RVOMethod.RVO) vo_op = new RVO_OP();
            else if (rvo_method == RVOMethod.HRVO) vo_op = new HRVO_OP();
            else vo_op = new VO_OP();
            vo_op.Initialize(this);

            // We want to generate our agents
            Generate();
        }

        public virtual void Generate() {
            // Step 1: Generate some arrays specific to Generator
            this.agent_positions = new Vector3[num_agents];
            // Step 2: Use a FOR loop to instantiate agent values. Set their respective values in our native/normal arrays
            for (int i = 0; i < num_agents; i++) GenerateAgent(i);
            // Step 3: For our VO_OP, inform its transform access array
            vo_op.UpdateTransforms();     
            // Step 4: Initialize our KDTree and Query
            tree = new KDTree(this.agent_positions, 32);
            query = new KDQuery();
        }

        // By default, the base Generator class will randomize the start and end positions of each agent.
        // if you want to modify this behavior, simply create a child inheritor of this base class and overwrite this with your own `GenerateAgent()` function.
        protected virtual void GenerateAgent(int agent_index) {
            // Step 1a: Generate the start and end positions of our agent's trajectory via randomization
            Vector3 pos, dest;
            Vector3 centroid = new Vector3(bounds.x, 0f, bounds.y) / 2f;
            float x, y;
            switch(spawn_orientation) {
                case SpawnStyle.Rows:
                    float bx2 = bounds.x - bound_edge_buffer*2f;
                    float by2 = bounds.y - bound_edge_buffer*2f;
                    int n2 = (int)Mathf.Ceil(num_agents/2);
                    bool on_left = agent_index%2 == 1;
                    y = centroid.z + by2/2f - (by2/(n2+1) * (Mathf.Floor(agent_index/2)+1));
                    pos = new Vector3((on_left)  ? centroid.x - bx2/2f : centroid.x + bx2/2f, 0f, y);
                    dest = new Vector3((on_left) ? centroid.x + bx2/2f : centroid.x - bx2/2f, 0f, y);
                    break;
                case SpawnStyle.Circular:
                    float spawn_distance = Mathf.Min(bounds.x, bounds.y)/2f - bound_edge_buffer;
                    float angle_step = 2f * Mathf.PI / num_agents;
                    float theta = agent_index * angle_step;
                    x = Mathf.Sin(theta);
                    y = Mathf.Cos(theta);
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
            Personality p = demographics.GetRandomPersonality();

            // Step 2: Instantiate the agent itself
            GameObject go = Instantiate(agent_prefab, pos, Quaternion.LookRotation(forward));
            Transform t = go.transform;
            t.parent = agent_parent;

            // Step 3: Inform our agent data in vo_op
            vo_op.AddAgent(agent_index, pos, dest, t, p);

            // Step 4: Miscellaneous. For Robot components and KDTree stuff.
            agent_positions[agent_index] = pos;
            Robot ad = go.GetComponent<Robot>();
            if (ad != null) {
                ad.agent_index = agent_index;
                ad.personality = p;
            }
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
                // Skip ourselves if inactive
                vo_op.active[i] = vo_op.transforms[i].gameObject.activeSelf;
                if (!vo_op.active[i]) {
                    vo_op.num_neighbors[i] = 0;
                    continue;
                }
                // KDTree Sort
                List<KDQuery.DistanceResult<int>> results = new List<KDQuery.DistanceResult<int>>();
                QueryRadiusSort(vo_op.positions[i], visual_radius, ref results);
                // Process each potential neighbor
                int n_neighbors = 0;
                bool colliding = false;
                for(int j = 0; j < results.Count; j++) {
                    int neighbor_index = results[j].data;                   // What's the neighbor index?
                    if (neighbor_index == i) continue;                      // Skip if ourselves
                    if (!vo_op.active[neighbor_index]) continue;            // SKip if inactive neighbor
                    // Check if colliding
                    if (results[j].distance < Mathf.Pow(vo_op.radii[i] + vo_op.radii[neighbor_index], 2))
                    {
                        if (!colliding)
                        {
                            colliding = true;
                            n_neighbors = 0;
                        }
                        vo_op.neighbor_indices[i * max_neighbors + n_neighbors] = neighbor_index;   // Contribute to our neighbor indices
                        vo_op.neighbor_collisions[i * max_neighbors + n_neighbors] = true;
                        n_neighbors += 1;                                                       // Increment n_neighbors
                    }
                    else if (!colliding)
                    {
                        vo_op.neighbor_indices[i * max_neighbors + n_neighbors] = neighbor_index;   // Contribute to our neighbor indices
                        vo_op.neighbor_collisions[i * max_neighbors + n_neighbors] = false;
                        n_neighbors += 1;                                                       // Increment n_neighbors
                    }
                    if (n_neighbors == max_neighbors) break;    // Break immediately if beyond max_neighbors
                }
                // Set our num neighbors
                vo_op.num_neighbors[i] = n_neighbors;
                vo_op.colliding[i] = colliding;
            }
        }

        // Helper: Query our tree to find all agents within a radius of our original query point, sorted by distance
        public bool QueryRadiusSort(Vector3 query_position, float r, ref List<KDQuery.DistanceResult<int>> results) {
            results = new List<KDQuery.DistanceResult<int>>();
            query.RadiusSort(tree, query_position, r, results);
            return results.Count > 0;
        }
        public int QueryRadiusCount(Vector3 query_position, float r) {
            List<int> results = new List<int>();
            query.Radius(tree, query_position, r, results);
            return results.Count;
        }


        // The PROCESSING step: Using RVO mechanisms, determine the optimal velocity to move in.
        //                      Note that we require a deltaTime parameter, in case someone is using this in another update cycle
        protected virtual void Processing(float deltaTime)
        {
            vo_op.Execute(deltaTime);
        }

        // The MOVEMENT step: Knowing the optimal velocities to move in, adjust the position of each agent.
        protected virtual void Movement(float deltaTime) {
            // Initialize the job data
            var movement_job = new ApplyVelocityJobParallelFor() {
                new_velocities = vo_op.new_velocities,
                destinations = vo_op.destinations,
                deltaTime = deltaTime,
                accelerations = vo_op.accelerations,
                active = vo_op.active,
                destination_buffer = destination_buffer,
                positions = vo_op.positions,
                velocities = vo_op.velocities,
                reached_destination = vo_op.reached_destination
            };
            velocityJobHandle = movement_job.Schedule(vo_op.transforms);
            velocityJobHandle.Complete();
        } 


        // In this base class, we call Steps 1 and 2 in the Update loop and Step 3 in the LateUpdate loop.
        protected virtual void Update() {
            Observation();                  // Vision
            Processing(Time.deltaTime);     // Local Collision Avoidance
            Movement(Time.deltaTime);       // Movement
        }
        protected virtual void LateUpdate() {
            // Simpe: Rebuild our Tree after moving data from `positions` into `agent_positions`
            vo_op.positions.Reinterpret<Vector3>().CopyTo(agent_positions);
            tree.Rebuild();

            /*
            // Handle the cse that our `reached_destination_count` matches the total number of agents
            if (early_terminate_app && reached_destination_count == num_agents) {
                #if UNITY_STANDALONE
                    Application.Quit();
                #endif
                #if UNITY_EDITOR
                    UnityEditor.EditorApplication.isPlaying = false;
                #endif
            }
            */
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

        // Movement Job
        [BurstCompile(CompileSynchronously = true)]
        struct ApplyVelocityJobParallelFor : IJobParallelForTransform {
            // Jobs declare all data that will be accessed in the job
            // By declaring it as read only, multiple jobs are allowed to access the data in parallel
            
            [ReadOnly] public NativeArray<float3> new_velocities;
            [ReadOnly] public NativeArray<float> accelerations;
            [ReadOnly] public NativeArray<float3> destinations;
            [ReadOnly] public NativeArray<bool> active;

            // Delta time must be copied to the job since jobs generally don't have concept of a frame.
            // The main thread waits for the job same frame or next frame, but the job should do work deterministically
            // independent on when the job happens to run on the worker threads.
            [ReadOnly] public float deltaTime;
            [ReadOnly] public float destination_buffer;

            public NativeArray<float3> positions;   // read and write
            public NativeArray<float3> velocities; // read and write
            public NativeArray<bool> reached_destination;

            // The code actually running on the job
            public void Execute(int index, TransformAccess transform) {
                // Skip early if inactive
                if (!active[index] || reached_destination[index]) {
                    // Don't update position or reached destination
                    velocities[index] = new_velocities[index];
                    return;
                }
                // Initialize float3's for new position and velocity
                float3 new_position, new_velocity;
                float acceleration = accelerations[index];

                // Check early if we reached our destination
                float3 diff = destinations[index] - positions[index];
                float dist_to_destination = math.length(diff);
                bool at_destination = dist_to_destination < destination_buffer;

                /*
                // If we reached our destination, just set the new position to our destination and set new velocity to 0.
                // Otherwise, Determine the new velocity we should move the agent in.
                if (at_destination) {
                    new_position = destinations[index];
                    new_velocity = new float3(0f);
                } else {
                    float3 vA = velocities[index];
                    float3 new_vA = new_velocities[index]; 
                    float dv = math.length(new_vA - vA);
                    if (dv < acceleration * deltaTime) new_velocity = new_vA;
                    else new_velocity = (1f - (acceleration * deltaTime / dv)) * vA + (acceleration * deltaTime / dv) * new_vA;
                    new_position = positions[index] + new_velocity * deltaTime;
                }
                */
                new_velocity = new_velocities[index];
                new_position = positions[index] + new_velocity * deltaTime;         

                // We need to update our native arrays
                positions[index] = new_position;
                velocities[index] = new_velocity;
                reached_destination[index] = at_destination;

                // We also need to update our transform
                transform.localPosition = new_position;
                // Rotation is dependent on new velocity
                if (!at_destination && math.length(new_velocity)>0f) {
                    float3 dir_to_destination = math.normalize(diff);
                    transform.rotation = quaternion.LookRotation(math.normalize(new_velocity), new float3(0,1,0));
                }
            }
        }

        protected virtual void OnDestroy() {
            //if (agents_writer.is_active) agents_writer.Disable();
            //if (fps_writer.is_active) fps_writer.Disable();
            vo_op.Terminate();
        }

        // Helper: if we want to toggle specific agents or not, do so here
        public virtual void ToggleRobot(int agent_index, bool to_toggle) {
            vo_op.transforms[agent_index].gameObject.SetActive(to_toggle);
            vo_op.active[agent_index] = to_toggle;
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