using UnityEngine;
using Random = UnityEngine.Random;

namespace RVO {
    [CreateAssetMenu(fileName = "Demographic", menuName = "Robots/Demographic Group", order = 1)]
    public class Demographic : ScriptableObject
    {
        public string id;
        public SpawnRate<Personality>[] personalities;

        public Personality GetRandomPersonality() {
            int r = (int)(Random.value * 100f);
            Personality p = personalities[0].value;
            for(int i = 0; i < personalities.Length; i++) {
                Vector2Int chance = personalities[i].spawn_chance;
                if (chance.x <= r && r < chance.y) {
                    p = personalities[i].value;
                    break;
                }
            }
            return p;
        }

        public GameObject GetRandomAgent() {
            return GetRandomPersonality().GetRandomAgent();
        }
    }
}