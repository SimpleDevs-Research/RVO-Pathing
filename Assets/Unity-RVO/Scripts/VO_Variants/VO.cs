using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine.Jobs;

namespace RVO {
    public class VO_OP {
        // Reference to the Generator
        public Generator generator;

        // Native arrays that stores info on all agents
        // Basics
        public NativeParallelMultiHashMap<int, int> grid;
        public NativeArray<bool> active;
        public NativeArray<bool> is_agent;          // Is this an agent or non-agent?
        public NativeArray<float3> positions;       // Agent positions
        public NativeArray<quaternion> rotations;   // Agent rotations
        public NativeArray<float3> velocities;      // Agent velocities
        public NativeArray<float3> destinations;    // Agent destinations
        public Transform[] agent_transforms;        // Agent transforms
        public TransformAccessArray transforms;
        // Neighbors. Note that neighbor indices go up to a 
        //      max number of neighors per agent
        public NativeArray<int> neighbor_indices;           
        public NativeArray<int> num_neighbors;
        public NativeArray<bool> colliding;
        public NativeArray<bool> neighbor_collisions;
        // Agents also have some weight factors per agent
        public NativeArray<float> responsibility_factors;
        public NativeArray<float> safety_factors;
        public NativeArray<float> inertia_factors;
        // Additional RVO qualities important to VO/RVO/Etc.
        public NativeArray<float> radii;
        public NativeArray<float> max_speeds;
        public NativeArray<float> max_rotation_speeds;
        public NativeArray<float> accelerations;
        // Usually outputs
        public NativeArray<float3> new_velocities;
        public NativeArray<bool> reached_destination;


        // This is called at the beginning of each simulation
        public virtual void Initialize(Generator generator) {
            // Step 1: Setting the generator
            this.generator = generator;

            // Step 2: Get the number of agents. Note the discernment between `num_total_agents` and `num_total_agents`
            int num_total_agents = generator.num_total_agents;

            // Step 2a: Basics - shared across both agents and non-agents
            this.grid = new NativeParallelMultiHashMap<int, int>(num_total_agents, Allocator.Persistent);
            this.active = new NativeArray<bool>(num_total_agents, Allocator.Persistent);
            this.is_agent = new NativeArray<bool>(num_total_agents, Allocator.Persistent);
            this.positions = new NativeArray<float3>(num_total_agents, Allocator.Persistent);
            this.rotations = new NativeArray<quaternion>(num_total_agents, Allocator.Persistent);
            this.velocities = new NativeArray<float3>(num_total_agents, Allocator.Persistent);
            this.radii = new NativeArray<float>(num_total_agents, Allocator.Persistent);    // THis is a VO-related param, but it's also shared b/w agents and non-agents
            
            // Step 2b: Agent-only properties
            this.destinations = new NativeArray<float3>(num_total_agents, Allocator.Persistent);
            this.agent_transforms = new Transform[num_total_agents];
            // We don't set `transforms` JUST YET. That's done later!

            // Step 2b: Neighbors
            this.neighbor_indices = new NativeArray<int>(num_total_agents * generator.max_neighbors, Allocator.Persistent);
            this.num_neighbors = new NativeArray<int>(num_total_agents, Allocator.Persistent);
            this.colliding = new NativeArray<bool>(num_total_agents, Allocator.Persistent);
            this.neighbor_collisions = new NativeArray<bool>(num_total_agents * generator.max_neighbors, Allocator.Persistent);

            // Step 2c: RVO weight parameters
            this.responsibility_factors = new NativeArray<float>(num_total_agents, Allocator.Persistent);
            this.safety_factors = new NativeArray<float>(num_total_agents, Allocator.Persistent);
            this.inertia_factors = new NativeArray<float>(num_total_agents, Allocator.Persistent);
            // `radii` was added above
            this.max_speeds = new NativeArray<float>(num_total_agents, Allocator.Persistent);
            this.max_rotation_speeds = new NativeArray<float>(num_total_agents, Allocator.Persistent);
            this.accelerations = new NativeArray<float>(num_total_agents, Allocator.Persistent);
            
            // Step 2d: Outputs
            this.new_velocities = new NativeArray<float3>(num_total_agents, Allocator.Persistent);
            this.reached_destination = new NativeArray<bool>(num_total_agents, Allocator.Persistent);
        }

        // This is called whenever a new agent is added
        public virtual void AddAgent(int agent_index, Vector3 pos, Vector3 dest, Transform t, Personality p) {
            // Step 1: Basics
            this.active[agent_index] = t.gameObject.activeInHierarchy;
            this.is_agent[agent_index] = true;
            this.positions[agent_index] = pos;
            this.rotations[agent_index] = Quaternion.LookRotation((dest - pos).normalized);
            this.velocities[agent_index] = Vector3.zero;
            this.destinations[agent_index] = dest;
            this.agent_transforms[agent_index] = t;
            // Step 2: Neighbors
            //      We use a nested for loop because `neighbor_indices` 
            //      occupies a range of entries in `neighbor_indices`.
            for(int j = 0; j < this.generator.max_neighbors; j++) {
                this.neighbor_indices[agent_index*this.generator.max_neighbors+j] = agent_index;
                this.neighbor_collisions[agent_index*this.generator.max_neighbors+j] = false;
            }
            this.colliding[agent_index] = false;
            this.num_neighbors[agent_index] = 0;
            // Step 3: RVO weight parameters
            this.responsibility_factors[agent_index] = p.responsibility_factor;
            this.safety_factors[agent_index] = p.safety_factor;
            this.inertia_factors[agent_index] = p.inertia_factor;
            this.radii[agent_index] = p.spatial_radius;
            this.max_speeds[agent_index] = p.max_speed;
            this.max_rotation_speeds[agent_index] = p.max_rotation_speed;
            this.accelerations[agent_index] = p.acceleration;
            // Step 4: Outputs
            this.new_velocities[agent_index] = Vector3.zero;
            this.reached_destination[agent_index] = false;
        }

        // This is called whenever a new non-agent is to be handled. Note that the only contribution non-agents provide is:
        // - grid hash (for neighbor search) <- not here though, later
        // - active (to check if we should care about it during calculation)
        // - positions (in local space)
        // - velocities (in local space)
        // - radii
        
        // This is for adding a non-agent.
        public virtual void AddNonAgent(NonAgent non_agent = null) {
            int agent_index = non_agent.agent_index;
            this.active[agent_index] = non_agent.active;
            this.is_agent[agent_index] = false;
            this.positions[agent_index] = non_agent.position;
            this.rotations[agent_index] = non_agent.rotation;
            this.velocities[agent_index] = non_agent.velocity;
            this.radii[agent_index] = non_agent.radius;
        }
        // Overload to `AddNonAgent`. Means we fill with dummy data
        public virtual void AddNonAgent(int agent_index) {
            this.active[agent_index] = false;
            this.is_agent[agent_index] = false;
            this.positions[agent_index] = Vector3.zero;
            this.rotations[agent_index] = Quaternion.identity;
            this.velocities[agent_index] = Vector3.zero;
            this.radii[agent_index] = 0f;
        }

        // This is only callable if `non_agent` actually exists
        public virtual void UpdateNonAgent(NonAgent non_agent) {
            int agent_index = non_agent.agent_index;
            this.active[agent_index] = non_agent.active;
            this.positions[agent_index] = non_agent.position;
            this.rotations[agent_index] = non_agent.rotation;
            this.velocities[agent_index] = non_agent.velocity;
            this.radii[agent_index] = non_agent.radius;
        }

        // This is called after all agents are initialized
        public virtual void UpdateTransforms() {
            this.transforms = new TransformAccessArray(this.agent_transforms);   
        }

        // This is called per update loop.
        public virtual void Execute( float deltaTime, int num_threads=64 ) {}

        // This is called when the generator is terminated
        public virtual void Terminate() {
            // Step 1: Basics
            if (this.grid.IsCreated) this.grid.Dispose();
            if (this.active.IsCreated) this.active.Dispose();
            if (this.is_agent.IsCreated) this.is_agent.Dispose();
            if (this.positions.IsCreated) this.positions.Dispose();
            if (this.rotations.IsCreated) this.rotations.Dispose();
            if (this.velocities.IsCreated) this.velocities.Dispose();
            if (this.destinations.IsCreated) this.destinations.Dispose();
            if (this.transforms.isCreated) this.transforms.Dispose();
            // Step 2: Neighbors
            if (this.neighbor_indices.IsCreated) this.neighbor_indices.Dispose();
            if (this.num_neighbors.IsCreated) this.num_neighbors.Dispose();
            if (this.colliding.IsCreated) this.colliding.Dispose();
            if (this.neighbor_collisions.IsCreated) this.neighbor_collisions.Dispose();
            // Step 3: RVO Weight Parameters
            if (this.responsibility_factors.IsCreated) this.responsibility_factors.Dispose();
            if (this.safety_factors.IsCreated) this.safety_factors.Dispose();
            if (this.inertia_factors.IsCreated) this.inertia_factors.Dispose();
            if (this.radii.IsCreated) this.radii.Dispose();
            if (this.max_speeds.IsCreated) this.max_speeds.Dispose();
            if (this.max_rotation_speeds.IsCreated) this.max_rotation_speeds.Dispose();
            if (this.accelerations.IsCreated) this.accelerations.Dispose();
            // Step 4: Outputs
            if (this.new_velocities.IsCreated) this.new_velocities.Dispose();
            if (this.reached_destination.IsCreated) this.reached_destination.Dispose();
        }
    }
}
