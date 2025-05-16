using UnityEngine;
using UnityEditor;

namespace Routing {
    [CustomEditor(typeof(RouteManager))]
    public class RouteManagerEditor : Editor
    {
        public override void OnInspectorGUI() {
            RouteManager current = (RouteManager)target;
            /*
            if (GUILayout.Button("Generate Regions")) current.GenerateRegions();
            */
            DrawDefaultInspector();
        }
    }
}
