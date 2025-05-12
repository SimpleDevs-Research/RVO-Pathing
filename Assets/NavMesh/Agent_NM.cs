using UnityEngine;
using UnityEngine.AI;
using RVO;

public class Agent_NM : MonoBehaviour
{
    [Header("=== Nav Mesh ===")]
    public NavMeshAgent nm_agent;

    [Header("=== READ ONLY ===")]
    public Vector3 destination = Vector3.zero;
    public float distance_to_destination = 0f;
    public Vector3 prev_position;

    // This time, we update our NavMeshAgent component with our properties
    protected virtual void Awake() {
        // Ensure that this game object has a NavMeshAgent component
        if (nm_agent == null) nm_agent = GetComponent<NavMeshAgent>();
        if (nm_agent == null) {
            this.gameObject.AddComponent<NavMeshAgent>();
            nm_agent = GetComponent<NavMeshAgent>();
        }

        // Custom properties
        nm_agent.autoBraking = false;
        nm_agent.autoRepath = true;

        // Save our previous position
        prev_position = transform.position;
        distance_to_destination = 1000f;
    }

    /*
    // Note: We don't need to actually move the agent in this implementation
    // However, we still need to update `current_velocity`.
    public virtual void Movement(float deltaTime) {
        // Update our record of our current velocity
        current_velocity = (transform.position - prev_position) / deltaTime;
        prev_position = transform.position;

        // Inform if we've reached our destination, according to NavMeshAgent
        distance_to_destination = (destination - transform.position).magnitude;
        reached_destination = distance_to_destination <= nm_agent.stoppingDistance; // * 2f && (!nm_agent.hasPath || nm_agent.velocity.sqrMagnitude == 0f);
    }
    */

    public virtual void SetDestination(Vector3 d) {
        destination = d;
        nm_agent.SetDestination(d);
        distance_to_destination = nm_agent.remainingDistance;
    }
}
