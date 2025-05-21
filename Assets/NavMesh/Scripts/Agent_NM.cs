using UnityEngine;
using UnityEngine.AI;
using RVO;

public class NMRobot : Robot
{
    [Header("=== Nav Mesh ===")]
    public NavMeshAgent nav_mesh_agent;

    // This time, we update our NavMeshAgent component with our properties
    protected virtual void Start() {
        // Ensure that this game object has a NavMeshAgent component
        if (nav_mesh_agent == null) nav_mesh_agent = GetComponent<NavMeshAgent>();
        if (nav_mesh_agent == null) {
            this.gameObject.AddComponent<NavMeshAgent>();
            nav_mesh_agent = GetComponent<NavMeshAgent>();
        }

        // Custom properties
        nav_mesh_agent.autoBraking = false;
        nav_mesh_agent.autoRepath = true;

        // Get the destination from Generator singleton if it exists.
        if (Generator.current != null) {
            nav_mesh_agent.speed = Generator.current.vo_op.max_speeds[this.agent_index];
            nav_mesh_agent.acceleration = Generator.current.vo_op.accelerations[this.agent_index];
            nav_mesh_agent.stoppingDistance = Generator.current.destination_buffer;
            nav_mesh_agent.radius = Generator.current.vo_op.radii[this.agent_index];
            SetDestination(Generator.current.vo_op.destinations[this.agent_index]);
        }
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
        nav_mesh_agent.SetDestination(d);    
    }
}
