using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace RVO {
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
            // Set position and velocity
            this.position = (float2)position.ToVector2();
            this.velocity = (float2)velocity.ToVector2();
        }
    }

    [System.Serializable]
    public struct CandidateDirection {
        public int index;
        public float distance_cost;
        public float time_cost;
        public float2 ttc;
        public float penalty;
        public CandidateDirection(int index, float penalty=0f) {
            this.index = index;
            this.distance_cost = 0f;
            this.time_cost  = 0f;
            this.ttc = new(0f,0f);
            this.penalty = penalty;
        }
        public CandidateDirection(int index, float distance_cost, float time_cost, float2 ttc, float penalty=0f) {
            this.index = index;
            this.distance_cost = distance_cost;
            this.time_cost = time_cost;
            this.ttc = ttc;
            this.penalty = penalty;
        }
    }

    [System.Serializable]
    public struct StartDestinationPair {
        public string name;
        public Vector3 start;
        public Vector3 end;
        public Color color;
        public List<Vector3> points;
    }
}
