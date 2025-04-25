using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using DataStructures.ViliWonka.KDTree;
using Random = UnityEngine.Random;
#if UNITY_EDITOR
using UnityEditor;
#endif

public class PedestrianManager : MonoBehaviour
{
    public static PedestrianManager current;

    public enum QType {
        ClosestPoint,
        KNearest,
        Radius,
        Interval
    }

    [Header("=== Setup ===")]
    public int num_agents = 50;
    public Vector2 bounds = new Vector2(20f,20f);
    public Color bounds_color = Color.yellow;
    public Transform agent_parent;
    public Transform query_point;
    public Color query_color = Color.blue;
    public Pedestrian agent_prefab;
    public float custom_delta_time = 0f;
    [HideInInspector] public float deltaTime;

    [Header("=== KDTree ===")]
    [HideInInspector] public Vector3[] agent_positions;
    [HideInInspector] public Transform[] agent_transforms;
    [HideInInspector] public Pedestrian[] agent_pedestrians;
    public KDTree tree;
    public KDQuery query;
    public QType query_type = QType.ClosestPoint;
    public int query_nearest = 10;
    public float query_radius = 10f;
    public Vector3 query_interval = new Vector3(10f, 0f, 10f);
    [HideInInspector] List<int> result_indices = new List<int>();


    #if UNITY_EDITOR
    void OnDrawGizmos() {
        Vector3 _bounds = new Vector3(bounds.x, 0f, bounds.y);
        Vector3 centroid = _bounds/2f;
        Gizmos.color = bounds_color;
        Gizmos.DrawWireCube(centroid, _bounds);

        if (query_point != null && result_indices.Count > 0) {
            Gizmos.color = query_color;
            for(int i = 0; i < result_indices.Count; i++) {
                Gizmos.DrawLine(query_point.position, agent_positions[result_indices[i]]);
            }
        }
    }
    #endif

    void Awake() {
        current = this;

        if (agent_parent == null) agent_parent = this.transform;
        GenerateAgentsAndKDTree();
    }

    public void GenerateAgentsAndKDTree() {
        // Initialize list of positions an d transforms
        agent_positions = new Vector3[num_agents];
        agent_transforms = new Transform[num_agents];
        agent_pedestrians = new Pedestrian[num_agents];
        for(int i = 0; i < num_agents; i++) {
            // Generate random position
            Vector3 rand_start = new Vector3(Random.Range(0f, bounds.x), 0f, Random.Range(0f, bounds.y));
            Vector3 rand_end = new Vector3(Random.Range(0f, bounds.x), 0f, Random.Range(0f, bounds.y));
            // Instantiate agent
            GenerateAgent(i, rand_start, rand_end, out Transform t, out Pedestrian p);

            // Add to agent_positions and agent_transforms
            agent_positions[i] = rand_start;
            agent_transforms[i] = t;
            agent_pedestrians[i] = p;
        }
        // Initialize our KDTree and Query
        tree = new KDTree(agent_positions, 32);
        query = new KDQuery();
    }

    public void GenerateAgent(int id, Vector3 start, Vector3 end, out Transform  t, out Pedestrian p) {
        p = Instantiate(agent_prefab, start, Quaternion.identity) as Pedestrian;
        p.Init(id, end);
        t = p.gameObject.transform;
        t.parent = agent_parent;
    }

    void Update() {

        // We want to look at all agents and update their positions. Then we rebuild the tree
        for(int i = 0; i < agent_positions.Length; i++) {
            agent_positions[i] = agent_transforms[i].position;
            agent_transforms[i].localScale = Vector3.one;
        }
        tree.Rebuild();

        // if we have a query point, then we search
        if (query_point == null) return;

        // Query. If our query resuylts are not empty, then we do something
        result_indices = new List<int>();
        if (QueryNeighbors(query_point.position, ref result_indices)) {
            for(int i = 0; i < result_indices.Count; i++) {
                agent_transforms[result_indices[i]].localScale = Vector3.one * 2f;
            }
        }
    }

    void FixedUpdate() {
        // Update delta time
        deltaTime = custom_delta_time > 0f ? custom_delta_time : Time.fixedDeltaTime;
    }

    public bool QueryNeighbors(Vector3 query_position, ref List<int> indices) {
        indices = new List<int>();
        switch(query_type) {
            case QType.ClosestPoint:
                query.ClosestPoint(tree, query_position, indices);
                break;
            case QType.KNearest:
                query.KNearest(tree, query_position, query_nearest, indices);
                break;
            case QType.Radius:
                query.Radius(tree, query_position, query_radius, indices);
                break;
            case QType.Interval:
                query.Interval(tree, query_position - query_interval/2f, query_position + query_interval/2f, indices);
                break;
        }
        return indices.Count > 0;
    }
}
