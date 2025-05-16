using UnityEngine;

[CreateAssetMenu(fileName = "Personality", menuName = "Robots/Personality Type", order = 2)]
public class Personality : ScriptableObject
{
    public string id;
    
    [Header("=== RVO ===")]
    [Range(0f,1f)] public float responsibility_factor;
    public float safety_factor;
    public float inertia_factor;
    [Space]
    public float spatial_radius = 0.25f;
    public float max_speed = 1f;
    public float acceleration = 5f;

    [Header("=== Routing ===")]
    public float risk_aversion;
    public float dirtiness_aversion;
    public float crowdedness_aversion;
    public float distance_aversion;
    public float litter_inclination;
}