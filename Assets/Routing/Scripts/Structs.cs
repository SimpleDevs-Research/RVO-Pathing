using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RVO;

namespace Routing {

    [System.Serializable]
    public class Segment {
        
        [Header("= References =")]
        [Tooltip("The two nodes representing this segment")]
        public Node node1;
        public Node node2;
        [Tooltip("The path regions that this segment is comprised of")]
        public Region[] regions;

        [Header("= Manual Properties =")]
        [Tooltip("The base cost of this segment")]
        public float base_cost;

        [Header("= READ ONLY =")]
        public float distance;
        public float density;
        public float dirtiness;
        public float risk;

        public void UpdateSegment() {
            // Distance is an easy one, and we don't adjust neither safety nor base_cost
            distance = Vector3.Distance(node1.transform.position, node2.transform.position);

            // Iterate through path regions. We need to auto-calculate density and dirtiness
            density = 0f;
            dirtiness = 0f;
            risk = 0f;
            foreach(Region region in regions) {
                density += region.density;
                dirtiness += region.dirtiness;
                risk += region.risk;
            }
            // Check with our main nodes
            if (node1 != null && node1.region != null) {
                density += node1.region.density;
                dirtiness += node1.region.dirtiness;
                risk += node1.region.risk;
            }
            if (node2 != null && node2.region != null) {
                density += node2.region.density;
                dirtiness += node2.region.dirtiness;
                risk += node2.region.risk;
            }
        }

        public float ComputePersonalCost(Personality personality) {
            float distance_cost = distance * personality.distance_aversion;
            float risk_cost = risk * personality.risk_aversion;
            float density_cost = density * distance * personality.crowdedness_aversion;
            float condition_cost = dirtiness * personality.dirtiness_aversion;
            // If RouteManager singleton is active, then we update these values
            if (RouteManager.Instance != null) {
                distance_cost *= RouteManager.Instance.distance_weight;
                if (!RouteManager.Instance.consider_risk) distance_cost = 0f;
                risk_cost *= RouteManager.Instance.risk_weight;
                if (!RouteManager.Instance.consider_risk) risk_cost = 0f;
                density_cost *= RouteManager.Instance.density_weight;
                if (!RouteManager.Instance.consider_crowding) density_cost = 0f;
                condition_cost *= RouteManager.Instance.condition_weight;
                if (!RouteManager.Instance.consider_dirtiness) condition_cost = 0f;
            }
            return base_cost +  distance_cost + risk_cost + density_cost + condition_cost;
        }
    }

}

