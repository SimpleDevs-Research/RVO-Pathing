using UnityEngine;

namespace RVO {

    // A "NonAgent" is a Robot that doesn't follow the conventional workflow of VO/RVO/HRVO.
    // Instead, this agent is acknowledged by Generator and is considered during some workflows
    // (e.g. neighbor detection). However, its movements and behaviors otherwise are to be dictated
    // by either itself or other managers/controllers/player interactions.

    // Examples of NonAgents include:
    // - Obstacles
    // - User Avatar
    // - Cameras

    public class NonAgent : Robot
    {

        [Header("=== References ===")]
        [Tooltip("The parent transform that this transform OUGHT to consider its parent. If unset, defaults to this transform's parent in the hierarchy")]
        public Transform parent;
        [Tooltip("What's the radius of this in VO/RVO/HRVO?")]
        public float radius;

        // Outputs
        public Vector3 position = Vector3.zero; // Note that this is ALWAYS the local position relative to `parent`
        public Vector3 velocity = Vector3.zero; // Note that this is ALWAYS the local vector relative to `parent`
        public Vector3 prev_position = Vector3.zero;

        public bool active => gameObject.activeInHierarchy;
        public Vector3 localPosition => (parent != null) 
            ? parent.InverseTransformPoint(transform.position)
            : transform.localPosition;

        // Methods
        private void OnEnable() {
            // Make sure we haven't accidentally forgotten about the parent.
            if (parent == null) parent = transform.parent;
        }
        
        public void UpdateAgent(float deltaTime) {
            position = localPosition;
            velocity = (position - prev_position) / deltaTime;
            prev_position = position;
        }

    }

}