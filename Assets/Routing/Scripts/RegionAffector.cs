using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

namespace Routing {
    public class RegionAffector : MonoBehaviour
    {

        public enum EffectType { 
            none,
            dirtiness,
            risk
        }

        public EffectType effect_type;
        public float effect_level;
        public float despawn_time;
        public bool randomize_rotation;
        public Region affected_region = null;
        private float time_since_spawn = 0f;

        protected void Start() {
            // Rotation. Randomizes rotation
            if(randomize_rotation) transform.Rotate(new Vector3(0, Random.Range(0, 360), 0));
            
            // Adjust the height if needed
            RaycastHit hit;
            if ((Physics.Raycast(transform.position, -Vector3.up, out hit, 10f))) {
                if (hit.distance > 0.3f) transform.position = new Vector3(transform.position.x, transform.position.y - hit.distance, transform.position.z);
            }

            // If spawned in a world with RouteManager, let it query to find the closest Region
            // If this effector is within the radius of that affector, then we have to inform...
            //      ... that a new affector is in place.
            if (RouteManager.Instance != null) {
                Region closest_region = RouteManager.Instance.getClosestRegion(transform.position);
                if (Vector3.Distance(transform.position, closest_region.transform.position) <= closest_region.radius) {
                    affected_region = closest_region;
                    affected_region.AddAffector(this);
                }
            }
        }

        protected void Update() {
            if (RouteManager.Instance != null) {
                Region closest_region = RouteManager.Instance.getClosestRegion(transform.position);
                if (Vector3.Distance(transform.position, closest_region.transform.position) <= closest_region.radius) {
                    if (affected_region != null) affected_region.RemoveAffector(this);
                    affected_region = closest_region;
                    affected_region.AddAffector(this);
                }
            }
        }

        protected void LateUpdate() {
            if (despawn_time <= 0f) return;
            time_since_spawn += Time.deltaTime;
            if(time_since_spawn >= despawn_time) {
                if (affected_region != null) affected_region.RemoveAffector(this);
                Destroy(gameObject);
            }
        }
    }

}