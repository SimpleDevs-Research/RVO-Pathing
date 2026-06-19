using UnityEngine;
using Random = UnityEngine.Random;

namespace RVO {
    [CreateAssetMenu(fileName = "Personality", menuName = "Robots/Personality Type", order = 2)]
    public class Personality : ScriptableObject
    {
        public string id;

        [Header("=== Prefab ===")]
        public SpawnRate<GameObject>[] agent_prefabs;
        
        [Header("=== RVO ===")]
        [Range(0f,1f)] public float responsibility_factor;
        public float safety_factor;
        public float inertia_factor;
        [Space]
        public float spatial_radius = 0.25f;
        public float max_speed = 1f;
        public float max_rotation_speed = 10f;   // radians per sec
        public float acceleration = 5f;

        [Header("=== Routing ===")]
        public float risk_aversion;
        public float dirtiness_aversion;
        public float crowdedness_aversion;
        public float distance_aversion;
        public float litter_inclination;

        public GameObject GetRandomAgent() {
            int r = (int)(Random.value * 100f);
            GameObject go = agent_prefabs[0].value;
            for(int i = 0; i < agent_prefabs.Length; i++) {
                Vector2Int chance = agent_prefabs[i].spawn_chance;
                if (chance.x <= r && r < chance.y) {
                    go = agent_prefabs[i].value;
                    break;
                }
            }
            return go;
        }
    }

}