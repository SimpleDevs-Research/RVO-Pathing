using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Mathematics;
using Unity.Collections;

using Unity.Jobs;
using UnityEngine.Jobs;
using Unity.Burst;

// This script recreates the HRVO operations conducted by Snape et al:
// Paper: https://ieeexplore.ieee.org/abstract/document/5746538
// Code: https://github.com/snape/HRVO

// HRVO attempts to solve the oscillation dance seen in RVO by emulating sidedness
// Namely. robots will choose optimal velocities based on where their current velocity lies within the RVO halfline.
// In the script operations, Snape does something... weird
// Namely, they divide the computation for new velocity into several iterative loops.
// We'll try to recreate them here, but we may try to optimize where possible.

namespace RVO {
    public class HRVO_OP : VO_OP
    {
        JobHandle neighborJobHandle = default;
        JobHandle hrvoJobHandle = default;

        [System.Serializable]
        public struct VOSides {
            public float3 side1;
            public float3 side2;
            public VOSides(float3 s1, float3 s2) {
                this.side1 = s1;
                this.side2 = s2;
            }
            public string ToString() { return side1.ToString() + " --- " + side2.ToString(); }
        }

        public NativeArray<VOSides> vo_sides;
        private VOSides[] agent_vo_sides;

        // Need to override the Initialize function to add a new native array for VO side tangents
        public override void Initialize(Generator generator) {
            base.Initialize(generator);
            this.vo_sides = new NativeArray<VOSides>(generator.num_agents * generator.max_neighbors, Allocator.Persistent);
            agent_vo_sides = new VOSides[this.vo_sides.Length];
        }

        public override void Execute(float deltaTime, int num_threads = 128) {
            /*
            var neighbors_job = new NeighborSidesJobParallelFor() {
                active = this.active,
                reached_destination = this.reached_destination,
                positions = this.positions,
                velocities = this.velocities,
                neighbor_indices = this.neighbor_indices,
                num_neighbors = this.num_neighbors,
                radii = this.radii,
                max_neighbors = generator.max_neighbors,
                vo_sides = this.vo_sides
            };
            neighborJobHandle = neighbors_job.Schedule(this.positions.Length, num_threads);
            neighborJobHandle.Complete();

            this.vo_sides.CopyTo(agent_vo_sides);
            for(int i = 0; i < agent_vo_sides.Length; i++) {
                Debug.Log($"{i}: {agent_vo_sides[i].ToString()}");
            }
            */
            
            var hrvo_job = new HRVOJobParallelFor() {
                positions = this.positions,
                velocities = this.velocities,
                destinations = this.destinations,
                reached_destination = this.reached_destination,
                neighbor_indices = this.neighbor_indices,
                //vo_sides = this.vo_sides,
                num_neighbors = this.num_neighbors,
                colliding = this.colliding,
                active = this.active,
                responsibility_factors = this.responsibility_factors,
                safety_factors = this.safety_factors,
                inertia_factors = this.inertia_factors,
                radii = this.radii,
                max_speeds = this.max_speeds,
                // For now, assume equal radius size among all agents
                num_directions = generator.num_candidate_directions,
                max_neighbors = generator.max_neighbors,
                deltaTime = deltaTime,
                // The output
                new_velocities = this.new_velocities
            };
            //hrvoJobHandle = hrvo_job.Schedule(this.positions.Length, num_threads, neighborJobHandle);
            hrvoJobHandle = hrvo_job.Schedule(this.positions.Length, num_threads);
            hrvoJobHandle.Complete();
            

        }

        // Need to override the Terminate method to account for the new vo_sides native array
        public override void Terminate() {
            base.Terminate();
            if (vo_sides.IsCreated) vo_sides.Dispose();
        }

        [BurstCompile(CompileSynchronously = true)]
        struct NeighborSidesJobParallelFor : IJobParallelFor {
            // Current Agent States
            [ReadOnly] public NativeArray<bool> active;
            [ReadOnly] public NativeArray<bool> reached_destination;
            [ReadOnly] public NativeArray<float3> positions;       // List of all positions of all agents
            [ReadOnly] public NativeArray<float3> velocities;      // List of all velocities of all agents
            [ReadOnly] public NativeArray<int> neighbor_indices;    // List of, upwards to 8, neighbors of all agents
            [ReadOnly] public NativeArray<int> num_neighbors;       // List of the number of neighbors of all agents
            [ReadOnly] public NativeArray<float> radii;             // The expected radius of all agents.
            // Global parameters
            [ReadOnly] public int max_neighbors;                    // The maximum number of neighbors possible

            // Outputs
            [WriteOnly] public NativeArray<VOSides> vo_sides;    // The VO sides for each neighbor for all agents

            // Helper function: rotate a vector given the center vector and the angle of separation
            public float3 RotateVector(float3 v, float angleRadians) {
                float cos = math.cos(angleRadians);
                float sin = math.sin(angleRadians);
                // Rotate around Y-axis (XZ plane)
                float x = v.x * cos - v.z * sin;
                float z = v.x * sin + v.z * cos;
                return new float3(x, 0f, z); // Keep y = 0
            }

            // Execute
            public void Execute(int agent_index) {
                // Disable if we are inactive or we already reached our destination
                if (!active[agent_index] || reached_destination[agent_index]) return;
                // For each neighbor, calculate the displacement characteristics
                float3 pA = positions[agent_index];
                float3 vA = velocities[agent_index];
                float rA = radii[agent_index];
                int n = num_neighbors[agent_index];
                // If no neighbors, don't bother
                if (n == 0) return;

                for (int j = 0; j < n; j++)
                {
                    // Get characteristics about each neighbor
                    int neighbor_index = neighbor_indices[agent_index * max_neighbors + j];
                    float3 pB = positions[neighbor_index];
                    float3 vB = velocities[neighbor_index];
                    float rB = radii[neighbor_index];
                    // Start calculating the displacement vector
                    float3 ba = pB - pA;
                    float dist = math.length(ba);
                    float3 dir = ba / dist;
                    // calculate the angle
                    float angle = math.asin((rA+rB) / dist);
                    float3 side1 = RotateVector(dir, angle);
                    float3 side2 = RotateVector(dir, -angle);
                    // Given both sides, record the side data for each neighbor
                    vo_sides[agent_index * max_neighbors + j] = new VOSides(side1, side2);
                }
            }
        }
    
        [BurstCompile(CompileSynchronously = true)]
        struct HRVOJobParallelFor : IJobParallelFor {
            // Current Agent States
            [ReadOnly] public NativeArray<float3> positions;       // List of all positions of all agents
            [ReadOnly] public NativeArray<float3> velocities;      // List of all velocities of all agents
            [ReadOnly] public NativeArray<float3> destinations;    // List of all destinations of all agents
            [ReadOnly] public NativeArray<bool> reached_destination;  // List of all agents who reached their destinations
            [ReadOnly] public NativeArray<int> neighbor_indices;    // List of, upwards to 8, neighbors of all agents
            //[ReadOnly] public NativeArray<VOSides> vo_sides;
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
            public float TimeToCollision(float3 pA, float3 Vab, float3 pB, float rr, bool colliding) {
                float3 ba = pB - pA;
                float sq_diam = sq(rr);
                float Vab2 = absSq(Vab);
                float time;

                float discr = -sq(det(Vab, ba)) + sq_diam * Vab2;
                if (discr > 0f) {
                    if (colliding) {
                        time = (mult(Vab, ba) + math.sqrt(discr)) / Vab2;
                        if (time < 0) time = -100000f;
                    } else {
                        time = (mult(Vab, ba) - math.sqrt(discr)) / Vab2;
                        if (time < 0f) time = 100000f;
                    }
                } else {
                    if (colliding) time = -100000f;
                    else time = 100000f;
                }
                return time;
            } 

            // Specific to HRVO
            public float3 rotateVector(float3 v, float angleRadians) {
                float cos = math.cos(angleRadians);
                float sin = math.sin(angleRadians);
                // Rotate around Y-axis (XZ plane)
                float x = v.x * cos - v.z * sin;
                float z = v.x * sin + v.z * cos;
                return new float3(x, 0f, z); // Keep y = 0
            }

            // Specific to HRVO
            public float3 computeDisplacementToExitVO(float3 pA, float3 pB, float3 v_rel, float rr) {
                float3 ba = pB - pA;
                float dist = math.length(ba);
                float3 dir = ba / dist;
    
                float angle = math.asin(rr / dist);
                float3 tangent1 = rotateVector(dir, angle);
                float3 tangent2 = rotateVector(dir, -angle);
                float3 leg1 = math.dot(v_rel, tangent1) < math.dot(v_rel, tangent2) ? tangent1 : tangent2;

                // Project v_rel onto leg1
                float3 projection = math.dot(v_rel, leg1) * leg1;
                float3 u = projection - v_rel;
                return u;
            }

            // helper: process a potential candidate velocity
            public float2 CalculatePenalty(int index, float3 candidate_velocity, float3 preferred_velocity, float3 pA, float3 vA, int n_neighbors, bool c)
            {
                // Initialize the distance cost, time cost, and inertia costs
                float distance_cost = math.length(candidate_velocity - preferred_velocity);
                float time_cost = 100000f;
                float inertia_cost = math.length(candidate_velocity - vA) * inertia_factors[index];
                float ct;
                // Given the candidate velocity, iterate through our neighbors
                for (int j = 0; j < n_neighbors; j++)
                {
                    // Get the position and velocity of the other agent
                    int neighbor_index = neighbor_indices[index * max_neighbors + j];
                    //VOSides sides = vo_sides[index * max_neighbors + j];
                    float3 pB = positions[neighbor_index];
                    float3 vB = velocities[neighbor_index];
                    float rB = radii[neighbor_index];
                    float mink_sum = radii[index] + rB;
                    // Different from HRVO: Compute rel_vel, then compute the amont of displacement to excit the VO. Then re-estimate translate_vb_va.
                    float3 rel_vel = (1f/responsibility_factors[index])*candidate_velocity + (1f-(1f/responsibility_factors[index]))*vA - vB;
                    /*
                    // Given the VOSides for this neighbor, howdo we displace the VO apex?
                    float3 leg = math.dot(rel_vel, sides.side1) < math.dot(rel_vel, sides.side2) ? sides.side1 : sides.side2;
                    // Project rel_vel onto leg
                    float3 projection = math.dot(rel_vel, leg) * leg;
                    float3 u = projection - rel_vel;
                    */
                    float3 u = computeDisplacementToExitVO(pA, pB, rel_vel, mink_sum);
                    float3 v_apex_hrvo = vB + 0.5f * u;
                    // Now calculate translate_vb_va with respect to this new apex
                    float3 translate_vb_va = candidate_velocity - v_apex_hrvo;
                    // calculate time to collision for this agent
                    float time = TimeToCollision(pA, translate_vb_va, pB, mink_sum, c);
                    ct = time;
                    if (c) ct = -(time / deltaTime) - (absSq(candidate_velocity) / sq(max_speeds[index]));
                    // If the current time to collision is less than the time cost, then we set it
                    if (ct < time_cost) time_cost = ct;
                }
                // Return the final penalty
                return new float2(
                    safety_factors[index] / time_cost + distance_cost + inertia_cost,
                    time_cost
                );
            }

            public void Execute(int index) {
                // Skip entirely if we're inactive
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
}
