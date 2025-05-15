using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class NavMeshUnderstanding : MonoBehaviour
{
    public Vector3[] vertices;
    public int[] polygons;
    public Vector3[] centroids;

    #if UNITY_EDITOR
    private void OnDrawGizmosSelected() {
        if (!Application.isPlaying) return;
        Gizmos.color = Color.black;
        for(int i = 0; i < centroids.Length; i++) Gizmos.DrawSphere(centroids[i], 0.5f);
        Gizmos.color = Color.red;
        for(int j = 0; j < vertices.Length; j++) Gizmos.DrawSphere(vertices[j], 0.75f);

    }
    #endif

    // Start is called before the first frame update
    private void Start() {
        var navMesh = NavMesh.CalculateTriangulation();
        vertices = navMesh.vertices;
        polygons = navMesh.indices;
        centroids = new Vector3[polygons.Length/3];
        for(int i = 0; i < polygons.Length; i+=3) {
            centroids[i/3] = (vertices[polygons[i]] + vertices[polygons[i+1]] + vertices[polygons[i+2]])/3f;
        }
    }
}
