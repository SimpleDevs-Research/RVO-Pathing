using UnityEngine;
using UnityEditor;

namespace RVO {
    [CustomEditor(typeof(GenerateSpawnLogic))]
    public class GenerateSpawnLogicEditor : Editor
    {
        public override void OnInspectorGUI() {
            GenerateSpawnLogic my_target = (GenerateSpawnLogic)target;
            DrawDefaultInspector();
            if (GUILayout.Button("Add New Pair to Spawn Logic")) {
                my_target.AddToLogic();
            }
        }
    }
}
