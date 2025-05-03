using UnityEngine;
using UnityEngine.AI;
using RVO;

public class Agent_NM : Agent
{
    [Header("=== Nav Mesh ===")]
    public NavMeshAgent nm_agent;
    [HideInInspector] public Vector3 prev_position;

    // This time, we update our NavMeshAgent component with our properties
    protected virtual void Awake() {
        // Ensure that this game object has a NavMeshAgent component
        if (nm_agent == null) nm_agent = GetComponent<NavMeshAgent>();
        if (nm_agent == null) {
            this.gameObject.AddComponent<NavMeshAgent>();
            nm_agent = GetComponent<NavMeshAgent>();
        }

        // Set equivalent properties
        nm_agent.speed = this.max_speed;
        nm_agent.angularSpeed = this.angular_speed;
        nm_agent.acceleration = this.acceleration;
        nm_agent.stoppingDistance = this.stopping_distance*0.5f;
        nm_agent.radius = this.spatial_radius;

        // Custom properties
        nm_agent.autoBraking = false;
        nm_agent.autoRepath = true;

        // Save our previous position
        prev_position = transform.position;
        distance_to_destination = 1000f;
    }

    // Note: NavMesh does not expose the number of avoidance obstacles it is considering for each agent.
    // We still need to call observe to measure the number of nearby agents.
    // the `GenerateAgents` parent needs to know an agent's current and optimal velocity for successful reporting. So we need a modified version of `Processing`
    protected override void Processing() {
        // Need to determine the optimal velocity here.
        // In this case, the optimal velocity will be what we extract from navmeshagent
        // It's unclear whether the NavMeshAgent's `desiredVelocity` is the velocity it WANTS to move in (aka literally its desired vel.)...
        // ... or if it's the velocity after avoidance is considered
        desired_velocity = nm_agent.desiredVelocity;
        optimal_velocity = nm_agent.velocity;
    }

    // Note: We don't need to actually move the agent in this implementation
    // However, we still need to update `current_velocity`.
    protected override void Movement() {
        // Update our record of our current velocity
        current_velocity = (transform.position - prev_position) / Time.fixedDeltaTime;
        prev_position = transform.position;

        // Inform if we've reached our destination, according to NavMeshAgent
        distance_to_destination = (destination - transform.position).magnitude;
        reached_destination = distance_to_destination <= nm_agent.stoppingDistance && (nm_agent.hasPath || nm_agent.velocity.sqrMagnitude == 0f);
    }

    public override void SetDestination(Vector3 d) {
        destination = d;
        nm_agent.SetDestination(d);
        distance_to_destination = nm_agent.remainingDistance;
    }
}
