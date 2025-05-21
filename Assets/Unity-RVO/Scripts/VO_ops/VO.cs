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

        // Native arrays that stores info on all on all agents
        // Basics
        public NativeArray<bool> active;
        public NativeArray<float3> positions;       // Agent positions
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
        public NativeArray<float> accelerations;
        // Usually outputs
        public NativeArray<float3> new_velocities;
        public NativeArray<bool> reached_destination;


        // This is called at the beginning of each simulation
        public virtual void Initialize(Generator generator) {
            // Step 1: Setting the generator
            this.generator = generator;

            // Step 2: Initialize the native arrays based on `num_agents`
            int n = generator.num_agents;
            // Step 2a: Basics
            this.active = new NativeArray<bool>(n, Allocator.Persistent);
            this.positions = new NativeArray<float3>(n, Allocator.Persistent);
            this.velocities = new NativeArray<float3>(n, Allocator.Persistent);
            this.destinations = new NativeArray<float3>(n, Allocator.Persistent);
            this.agent_transforms = new Transform[n];
            // We don't set `transforms` JUST YET. That's done later!
            // Step 2b: Neighbors
            this.neighbor_indices = new NativeArray<int>(n * generator.max_neighbors, Allocator.Persistent);
            this.num_neighbors = new NativeArray<int>(n, Allocator.Persistent);
            this.colliding = new NativeArray<bool>(n, Allocator.Persistent);
            this.neighbor_collisions = new NativeArray<bool>(n * generator.max_neighbors, Allocator.Persistent);
            // Step 2c: RVO weight parameters
            this.responsibility_factors = new NativeArray<float>(n, Allocator.Persistent);
            this.safety_factors = new NativeArray<float>(n, Allocator.Persistent);
            this.inertia_factors = new NativeArray<float>(n, Allocator.Persistent);
            this.radii = new NativeArray<float>(n, Allocator.Persistent);
            this.max_speeds = new NativeArray<float>(n, Allocator.Persistent);
            this.accelerations = new NativeArray<float>(n, Allocator.Persistent);
            // Step 2d: Outputs
            this.new_velocities = new NativeArray<float3>(n, Allocator.Persistent);
            this.reached_destination = new NativeArray<bool>(n, Allocator.Persistent);
        }

        // This is called whenever a new agent is added
        public virtual void AddAgent(int agent_index, Vector3 pos, Vector3 dest, Transform t, Personality p) {
            // Step 1: Basics
            this.active[agent_index] = true;
            this.positions[agent_index] = pos;
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
            this.accelerations[agent_index] = p.acceleration;
            // Step 4: Outputs
            this.new_velocities[agent_index] = Vector3.zero;
            this.reached_destination[agent_index] = false;
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
            if (this.active.IsCreated) this.active.Dispose();
            if (this.positions.IsCreated) this.positions.Dispose();
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
            if (this.accelerations.IsCreated) this.accelerations.Dispose();
            // Step 4: Outputs
            if (this.new_velocities.IsCreated) this.new_velocities.Dispose();
            if (this.reached_destination.IsCreated) this.reached_destination.Dispose();
        }
    }
}
