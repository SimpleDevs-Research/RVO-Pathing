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

public class GenerateAgents : MonoBehaviour
{
    public static GenerateAgents current;

    [System.Serializable]
    public struct AgentData {
        public int agent_index;
        public float2 position;
        public float2 velocity;
        public float radius;
        public AgentData(int index, Vector3 position, Vector3 velocity, float radius) {
            this.agent_index = index;
            this.position = (float2)position.ToVector2();
            this.velocity = (float2)velocity.ToVector2();
            this.radius = radius;
        }
        public void Update(Vector3 position, Vector3 velocity) {
            this.position = (float2)position.ToVector2();
            this.velocity = (float2)velocity.ToVector2();
        }
    }

    [Header("=== Agent Setup ===")]
    [Tooltip("The Transform parent of all agents")]         public Transform agent_parent;
    [Tooltip("The agent prefab that should be spawned")]    public Pedestrian_Static agent_prefab;
    [Tooltip("How many agents do you want?")]               public int num_agents = 50;

    [Header("=== Environment Setup ===")]
    [Tooltip("The environment bounds from origin")]         public Vector2 bounds = new Vector2(20f,20f);
    [Tooltip("Visualize the bounds via Gizmos")]            public Color bounds_color = Color.yellow;
    
    [Space]
    [HideInInspector] public Vector3[] agent_positions;
    [HideInInspector] public Pedestrian_Static[] agent_components;
    [HideInInspector] public AgentData[] agent_data;
    [HideInInspector] public Dictionary<GameObject, int> agent_index_map;
    protected KDTree tree;
    protected KDQuery query;

    #if UNITY_EDITOR
    protected virtual void OnDrawGizmos() {
        Vector3 _bounds = new Vector3(bounds.x, 0f, bounds.y);
        Vector3 centroid = _bounds/2f;
        Gizmos.color = bounds_color;
        Gizmos.DrawWireCube(centroid, _bounds);
    }
    #endif

    private void Awake() {
        current = this;

        // If the agent_parent is null, we set to ourselves
        if (agent_parent == null) agent_parent = this.transform;

        // We want to generate our agents
        Generate();
    }

    public virtual void Generate() {
        // Initialize the lists for KDTree
        agent_positions = new Vector3[num_agents];
        agent_components = new Pedestrian_Static[num_agents];
        agent_data = new AgentData[num_agents];

        // Generate each agent individually
        for(int i = 0; i < num_agents; i++) {

            // Generate random position as start point
            Vector3 start_point = GetRandomPointInBounds();
            
            // Instantiate agent. If the agent wants to move themselves, then we leave it up to the agent prefab instance itself.
            Pedestrian_Static ps = Instantiate(agent_prefab, start_point, Quaternion.identity) as Pedestrian_Static;
            ps.transform.parent = agent_parent;
            ps.agent_index = i;
            ps.gameObject.name = $"Agent {i}";

            // Initialize pedestrian data
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
        // Update each agent's data
        // We do it here to enable the update loop in each independent agent to conduct Observation and Processing
        for(int i = 0; i < agent_positions.Length; i++) {
            agent_components[i].current_velocity = agent_components[i].velocity;
            agent_positions[i] = agent_components[i].position;
            agent_data[i].Update(agent_components[i].position, agent_components[i].velocity);
        }

        // We update the KDTree here and now
        tree.Rebuild();
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
    public static Vector2 ToVector2(this Vector3 v) {
        return new Vector2(v.x, v.z);
    }
    public static Vector3 ToVector3(this float2 v) {
        return new Vector3(v[0], 0f, v[1]);
    }
}
