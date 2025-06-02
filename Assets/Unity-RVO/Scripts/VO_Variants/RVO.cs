using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Mathematics;
using Unity.Collections;

using Unity.Jobs;
using UnityEngine.Jobs;
using Unity.Burst;

namespace RVO {
    public class RVO_OP : VO_OP
    {
        // Job handle for managing the VO job
        JobHandle jobHandle = default;

        public override void Execute( float deltaTime, int num_threads=64 ) {
            var job = new RVOJobParallelFor() {
                positions = this.positions,
                velocities = this.velocities,
                destinations = this.destinations,
                reached_destination = this.reached_destination,
                neighbor_indices = this.neighbor_indices,
                num_neighbors = this.num_neighbors,
                colliding = this.colliding,
                active = this.active,
                responsibility_factors = this.responsibility_factors,
                safety_factors = this.safety_factors,
                inertia_factors = this.inertia_factors,
                radii = this.radii,
                max_speeds = this.max_speeds,
                // For now, assume equal radius size among all agents
                num_directions =generator.num_candidate_directions,
                max_neighbors = generator.max_neighbors,
                deltaTime = deltaTime,
                // The output
                new_velocities = this.new_velocities
            };
            jobHandle = job.Schedule(this.positions.Length, num_threads);
            jobHandle.Complete();
        }
    }

    // RVO Job
    [BurstCompile(CompileSynchronously = true)]
    struct RVOJobParallelFor : IJobParallelFor {
        // Current Agent States
        [ReadOnly] public NativeArray<float3> positions;       // List of all positions of all agents
        [ReadOnly] public NativeArray<float3> velocities;      // List of all velocities of all agents
        [ReadOnly] public NativeArray<float3> destinations;    // List of all destinations of all agents
        [ReadOnly] public NativeArray<bool> reached_destination;    // List of all agents who reached their destinations
        [ReadOnly] public NativeArray<int> neighbor_indices;    // List of, upwards to 8, neighbors of all agents
        [ReadOnly] public NativeArray<int> num_neighbors;       // List of the number of neighbors of all agents
        [ReadOnly] public NativeArray<bool> colliding;       // List of checks for collisions of all agents
        [ReadOnly] public NativeArray<bool> active;

        // Agent parameters
        [ReadOnly] public NativeArray<float> responsibility_factors;
        [ReadOnly] public NativeArray<float> safety_factors;
        [ReadOnly] public NativeArray<float> inertia_factors;
        [ReadOnly] public NativeArray<float> radii;             // The expected radius of all agents.
        [ReadOnly] public NativeArray<float> max_speeds;        // The maximum speeds of all agents.

        // Global parameters
        [ReadOnly] public float deltaTime;                      // Time step
        [ReadOnly] public int num_directions;                   // The number of directions we can iterate over
        [ReadOnly] public int max_neighbors;                    // The maximum number of neighbors possible
            
        // The output
        [WriteOnly] public NativeArray<float3> new_velocities;  // The final velocity of an agent

        // helper: calcualte determinatne of two float2's
        public float det(float3 a, float3 b) { return a.x*b.z - a.z*b.x; }
        // helper: calculate multiple of two float2's into single float
        public float mult(float3 a, float3 b) { return a.x*b.x + a.z*b.z; }
        // helper: calculate absolute squear of a float2
        public float absSq(float3 v) { return mult(v, v); }
        // helper: calculate square (not square root) of a float
        public float sq(float v) { return v*v; }

        // helper: calculate time to collision
        public float TimeToCollision(float3 pA, float3 Vab, float3 pB, float rr, bool c) {
            float3 ba = pB - pA;
            float sq_diam = sq(rr);
            float Vab2 = absSq(Vab);
            float time;

            /*
            if (math.lengthsq(ba) < sq_diam) {
                return 0.001f; // Immediate collision
            }
            */

            float discr = -sq(det(Vab, ba)) + sq_diam * Vab2;
            if (discr > 0f) {
                if (c) {
                    time = (mult(Vab, ba) + math.sqrt(discr)) / Vab2;
                    if (time < 0) time = float.NegativeInfinity;
                } else {
                    time = (mult(Vab, ba) - math.sqrt(discr)) / Vab2;
                    if (time < 0f) time = float.PositiveInfinity;
                }
            } else {
                if (c) time = float.NegativeInfinity;
                else time = float.PositiveInfinity;
            }
            return time;
        }

        // helper: process a potential candidate velocity
        public float2 CalculatePenalty(int index, float3 candidate_velocity, float3 preferred_velocity, float3 pA, float3 vA, int n_neighbors, bool c) {
            // Initialize the distance cost, time cost, and inertia costs
            float distance_cost = math.length(candidate_velocity - preferred_velocity);
            float time_cost = float.PositiveInfinity;
            float inertia_cost = math.length(candidate_velocity - vA) * inertia_factors[index];
            float ct;
            // Given the candidate velocity, iterate through our neighbors
            for(int j = 0; j < n_neighbors; j++) {
                // Get the position and velocity of the other agent
                int neighbor_index = neighbor_indices[index * max_neighbors + j];
                float3 pB = positions[neighbor_index];
                float3 vB = velocities[neighbor_index];
                float rB = radii[neighbor_index];
                //bool colliding = colliding[neighbor_indices[neighbor_indices_index]];
                // calculate time to collision for this agent
                float3 translate_vb_va = (1f/responsibility_factors[index])*candidate_velocity + (1f-(1f/responsibility_factors[index]))*vA - vB;
                //float mink_sum = radii[index] + radii[neighbor_index];
                float mink_sum = radii[index] + rB;
                float time = TimeToCollision(pA, translate_vb_va, pB, mink_sum, c);
                ct = time;
                if (c) ct = -(time / deltaTime) - (absSq(candidate_velocity)/sq(max_speeds[index]));
                // If the current time to collision is less than the time cost, then we set it
                if (ct < time_cost) time_cost = ct;
            }
            // Return the final penalty
            /*
            float penalty;
            if (time_cost <= 0f) penalty = float.PositiveInfinity;
            else penalty = safety_factors[index] / time_cost + distance_cost + inertia_cost;
            */
            float penalty = safety_factors[index] / time_cost + distance_cost + inertia_cost;
            return new float2(penalty,time_cost);
        }

        public void Execute(int index) {
            // Skip entirely if we're inactive or already reached our destination
            if (!active[index] || reached_destination[index]) {
                new_velocities[index] = new float3(0f,0f,0f);
                return;
            }
            // For agent i, we must determine the preferred velocity
            float3 pA = positions[index];
            float3 vA = velocities[index];
            int n_neighbors = num_neighbors[index];
            bool c = colliding[index];
            float3 preferred_velocity = math.normalize(destinations[index] - pA) * max_speeds[index];

            // If our n_neighbors is 0... then there's no need to perform the operation.
            if (n_neighbors == 0) {
                new_velocities[index] = preferred_velocity;
                return;
            }

            // Let's iterate across potential candidate velocities. For now, intitialize a candiate velocity (Vector2) and minimum penalty
            float3 new_velocity = preferred_velocity;
            float2 min_penalty = CalculatePenalty(index, preferred_velocity, preferred_velocity, pA, vA, n_neighbors, c);

            // Use a for loop to iterate across multiple possible velocities
            float angleStep = 2f * Mathf.PI / num_directions;
            for (int i = 0; i < num_directions; i++) {
                // let's increment from max_speed to the closest speed above 0, based on a velocity step
                float theta =(float)i * angleStep;
                float x = math.sin(theta);
                float y = math.cos(theta);
                // increment downwards to at least consider max speed
                for (float r = max_speeds[index]; r > 0f; r -= 0.1f) {
                    // Determine the candidate velocity
                    float3 candidate_velocity = new float3(r*x, 0f, r*y);
                    float2 est_penalty = CalculatePenalty(index, candidate_velocity, preferred_velocity, pA, vA, n_neighbors, c);
                    // Override the new velocity if the estimated penalty is smaller than the min penalty
                    if (est_penalty.x < min_penalty.x) {
                        min_penalty = est_penalty;
                        new_velocity = candidate_velocity;
                    }
                }
            }

            // Ultimately, set the new velocity as... the new velocity with the minimum penalty
            new_velocities[index] = new_velocity;
        }
    }

    
}
