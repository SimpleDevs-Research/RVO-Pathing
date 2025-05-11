using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RVO;

[ExecuteInEditMode]
public class DebugCosts : MonoBehaviour
{

    #if UNITY_EDITOR
    private void OnDrawGizmos() {
        Gizmos.color = cA;
        Gizmos.DrawRay(pA, vA);

        Gizmos.color = cB;
        Gizmos.DrawRay(pB, vB);
    }
    #endif

    [Header("Agent A")]
    public Transform a; 
    public Transform vA_ref;
    public Transform dA;
    public Vector3 pA;
    public Vector3 vA;
    public Vector3 desired_vA;
    public float rA = 0.25f;
    Color cA = Color.blue;

    [Header("Agent B")]
    public Transform b;
    public Transform vB_ref;
    public Transform dB;
    public Vector3 pB;
    public Vector3 vB;
    public Vector3 desired_vB;
    public float rB = 0.25f;
    Color cB = Color.red;

    [Header("Global Settings")]
    public float max_speed = 1.5f;
    public float deltaTime = 0.05f;
    public float safety_factor = 1f;
    public float inertia_factor = 1f;

    [Header("Agent A Results")]
    public Vector3 candidate_direction;
    public bool colliding = false;
    public float distance_cost = 0f;
    public float time_cost = 0;
    public float continuity_cost = 0f;
    public float discr = 0;
    public float continuity_diff = 0f;
    public float penalty = 0f;

    void Update() {
        // Update our stats
        pA = a.position;
        vA = a.forward * max_speed;
        desired_vA = (dA.position - pA).normalized * max_speed;

        pB = b.position;
        vB = b.forward * max_speed;
        desired_vB = (dB.position - pB).normalized * max_speed;

        // Check if A is colliding with B
        float distance = (pA - pB).magnitude;
        colliding = (distance < rA+rB);

        // For now, the candidate 
        candidate_direction = vA_ref.position - pA;
        
        // Calculate the distance cost and time cost
        distance_cost = 0f;
        if (!colliding) distance_cost = (candidate_direction - desired_vA).magnitude;
        time_cost = 1000000f;
        discr = 0;

        // Priming the time to collision for this Agent B, which we consider a neighbor
        float ct;

        // calculate time to collision for this agent
        Vector3 translate_vb_va = 2f * candidate_direction - vA - vB;
        float mink_sum = rA + rB;
        Vector2 time = TimeToCollision(pA, translate_vb_va, pB, mink_sum, colliding);

        // mod time to collision for this current agent with additional metrics, based on if we're colliding or not
        if (colliding) ct = -(time.x / deltaTime) - (absSq(candidate_direction)/sq(max_speed));
        else ct = time.x;

        // If the current time to collision is less than the time cost, then we set it
            time_cost = ct;
            discr = time.y;
        
        // Add an inertia cost
        continuity_diff = Vector3.Distance(candidate_direction, vA);
        continuity_cost = continuity_diff * inertia_factor;

        // ultimately, after considering all neighbor,s calculate the final penalty cost
        penalty = safety_factor / time_cost + distance_cost + continuity_cost;
    }

    // helper: calcualte determinatne of two float2's
    public float det(Vector3 a, Vector3 b) { return a.x*b.z - a.z*b.x; }
    
    // helper: calculate multiple of two float2's into single float
    public float mult(Vector3 a, Vector3 b) { return a.x*b.x + a.z*b.z; }

    // helper: calculate absolute squear of a float2
    public float absSq(Vector3 v) { return mult(v, v); }

    // helper: calculate square (not square root) of a float
    public float sq(float v) { return v*v; }

    // helper: calculate time to collision
    public Vector2 TimeToCollision(Vector3 pA, Vector3 Vab, Vector3 pB, float rr, bool collision) {
        Vector3 ba = pB - pA;
        float sq_diam = sq(rr);
        float Vab2 = absSq(Vab);
        float time;

        float discr = -sq(det(Vab, ba)) + sq_diam * Vab2;
        // They WILL collide. Usually occurs when two agents are moving into a direct collision
        if (discr > 0f) {
            if (collision) {
                time = (mult(Vab, ba) + Mathf.Sqrt(discr)) / Vab2;
                if (time < 0) time = -1000000f;
            } else {
                time = (mult(Vab, ba) - Mathf.Sqrt(discr)) / Vab2;
                if (time < 0) time = 1000000f;
            }
        } 
        // Their velocities will not lead to a collision.
        else {
            // let's do another equation: will my current trajectory lead 
            if (collision) time = -1000000f;
            else time = 1000000f;
        }
        
        return new Vector2(time, discr);
    }
}
