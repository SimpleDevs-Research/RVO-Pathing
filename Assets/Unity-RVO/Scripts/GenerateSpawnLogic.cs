using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RVO {
    [ExecuteInEditMode]
    public class GenerateSpawnLogic : MonoBehaviour
    {
        public SpawnLogic toEdit;
        public Transform startRef, endRef;
        public Color color;
        
        #if UNITY_EDITOR
        void OnDrawGizmos() {
            if (startRef == null || endRef == null) return;
            DrawPath(startRef.position, endRef.position, color);

            if (toEdit != null && toEdit.paths.Count > 0) {
                foreach(StartDestinationPair pair in toEdit.paths) DrawPath(pair);
            }
        }

        private void DrawPath(Vector3 start, Vector3 end, Color color) {
            Gizmos.color = color;
            Gizmos.DrawWireCube(start, Vector3.one*0.25f);
            Gizmos.DrawWireSphere(end, 0.25f);
            Gizmos.DrawLine(start, end);
        }
        private void DrawPath(StartDestinationPair pair) {
            Gizmos.color = pair.color;
            Gizmos.DrawWireCube(pair.start, Vector3.one*0.25f);
            Gizmos.DrawWireSphere(pair.end, 0.25f);
            Gizmos.DrawLine(pair.start, pair.end);
        }
        #endif

        public void AddToLogic() {
            if (toEdit == null) {
                Debug.LogError("ERROR: Cannot add to a spawn logic that isn't set to be edited.");
                return;
            }
            StartDestinationPair newPair = new StartDestinationPair() { name=$"Agent {toEdit.num_agents+1}", start=startRef.position, end=endRef.position, color=color, points=new List<Vector3>() };
            toEdit.paths.Add(newPair);
            toEdit.num_agents = toEdit.paths.Count;
        }

    }
}
