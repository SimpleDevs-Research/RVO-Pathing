using System.Collections.Generic;
using UnityEngine;

namespace RVO {
    [CreateAssetMenu(fileName = "SpawnLogic", menuName = "Robots/Spawn Logic", order = 3)]
    public class SpawnLogic : ScriptableObject
    {    
        public string label;
        public int num_agents;
        public List<StartDestinationPair> paths = new List<StartDestinationPair>();
    }
}