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

        [Header("=== Agent Setup ===")]
        [Tooltip("The Transform parent of all agents")]                 public Transform agent_parent;
        [Tooltip("The agent prefab that should be spawned")]            public Agent agent_prefab;
        [Tooltip("Should we randomly generate agent start points?")]    public bool randomize_starts = false;
        [Tooltip("Should we randomly generate agent end points?")]      public bool randomize_ends = false;

        [Header("=== Environment Setup ===")]
        [Tooltip("The main camera")]                            public Camera scene_cam;
        [Tooltip("The environment bounds from origin")]         public Vector2 bounds = new Vector2(20f,20f);
        [Tooltip("Visualize the bounds via Gizmos")]            public Color bounds_color = Color.yellow;

        [Header("=== Trajectories ===")]
        public bool cache_trajectories = true;
        public StartDestinationPair[] agent_trajectories;

        [Header("=== Record-Keeping ===")]
        public bool early_terminate_app = true;
        [Range(0f, 1f)] public float fps_smoothing_factor = 0.25f;
        public bool record_data = true;
        public CSVWriter agents_writer;
        public CSVWriter fps_writer;
        [HideInInspector] public float current_fps;
        [HideInInspector] public float smoothed_fps;

        [Header("=== Gizmos ===")]
        public bool draw_bounds = true;
        public bool draw_start_end = true;
        public bool draw_paths = false;
        public bool draw_trajectories => draw_start_end || draw_paths;

        [Header("=== Read-Only ===")]
        [Tooltip("How many agents were generated?")]    public int num_agents = 50;





        [HideInInspector] public Vector3[] agent_positions;
        [HideInInspector] public Agent[] agent_components;
        [HideInInspector] public AgentData[] agent_data;
        [HideInInspector] public Dictionary<GameObject, int> agent_index_map;
        protected KDTree tree;
        protected KDQuery query;

        #if UNITY_EDITOR
        protected virtual void OnDrawGizmos() {
            // Draw Bounds
            if (draw_bounds) {
                Vector3 _bounds = new Vector3(bounds.x, 0f, bounds.y);
                Vector3 centroid = _bounds/2f;
                Gizmos.color = bounds_color;
                Gizmos.DrawWireCube(centroid, _bounds);
            }

                    
            // Draw trajectories if prompted
            if (draw_trajectories) {
                for (int i = 0; i < agent_trajectories.Length; i++) {
                    StartDestinationPair sdp = agent_trajectories[i];
                    Gizmos.color = sdp.color;
                    if (draw_start_end) {
                        Gizmos.DrawSphere(sdp.start, 0.25f);
                        Gizmos.DrawWireSphere(sdp.end, 0.25f);
                        Gizmos.DrawLine(sdp.start, sdp.end);
                    } 
                    if (draw_paths && sdp.points.Count >= 2) {
                        for(int j = 0; j < sdp.points.Count-1; j++) {
                            Gizmos.DrawLine(sdp.points[j], sdp.points[j+1]);
                        }
                    }
                }
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
            if (record_data) {
                agents_writer.Initialize();
                fps_writer.Initialize();
            }

            // We want to generate our agents
            Generate();
        }

        public virtual void Generate() {
            // In this one, the number of agents is equal to the number of stored trajectories
            num_agents = agent_trajectories.Length;

            // Initialize the lists for KDTree
            agent_positions = new Vector3[num_agents];
            agent_components = new Agent[num_agents];
            agent_data = new AgentData[num_agents];

            // Generate each agent individually
            for(int i = 0; i < num_agents; i++) {
                
                // Get the startdestination pair
                StartDestinationPair sdp = agent_trajectories[i];

                // Generate random position as start point
                Vector3 start_point = (randomize_starts) 
                    ? GetRandomPointInBounds()
                    : sdp.start;
                Vector3 end_point = (randomize_ends) 
                    ? GetRandomPointInBounds()
                    : sdp.end;
                
                // Instantiate agent. If the agent wants to move themselves, then we leave it up to the agent prefab instance itself.
                Agent ps = Instantiate(agent_prefab, start_point, Quaternion.identity) as Agent;
                ps.transform.parent = agent_parent;
                ps.agent_index = i;
                ps.gameObject.name = $"Agent {i}";
                ps.generate_destination_on_start = false;
                ps.SetDestination(end_point);

                // Initialize Agent data
                AgentData ped = new AgentData(i, start_point, Vector3.zero, ps.spatial_radius);

                // Add to agent_positions and agent_data
                agent_positions[i] = start_point;
                agent_components[i] = ps;
                agent_data[i] = ped;
            }

            // Initialize our KDTree and Query
            tree = new KDTree(agent_positions, 32);
            query = new KDQuery();
        }

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

        protected virtual void OnDestroy() {
            if (agents_writer.is_active) agents_writer.Disable();
            if (fps_writer.is_active) fps_writer.Disable();
        }

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
            current_fps = 1f / Time.unscaledDeltaTime;
            smoothed_fps = (fps_smoothing_factor * current_fps) + (1f - fps_smoothing_factor) * smoothed_fps;
            fps_writer.AddPayload(frame);
            fps_writer.AddPayload((int)current_fps);
            fps_writer.AddPayload((int)smoothed_fps);
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

    }

    public static class ExtensionMethods {
        public static Vector2 ToVector2(this Vector3 v) {   return new Vector2(v.x, v.z);       }
        public static Vector3 ToVector3(this float2 v) {    return new Vector3(v[0], 0f, v[1]); }
    }


}