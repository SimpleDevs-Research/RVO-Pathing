using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RVO;

namespace Routing {
    public class Region : MonoBehaviour
    {
        public float density; //Person per unit
        public float dirtiness;
        public float risk;

        public List<Robot> agents_within = new List<Robot>();
        public List<RegionAffector> affectors = new List<RegionAffector>();
        public float radius, size, area;

        private void Start()
        {
            radius = Mathf.Min(transform.localScale.x, transform.localScale.z);
            area = Mathf.PI * radius * radius;
            size = transform.localScale.x * transform.localScale.z;
        }
        
        public void AddAffector(RegionAffector affector)
        {
            if (affectors.Contains(affector)) return;
            affectors.Add(affector);
            switch (affector.effect_type)
            {
                case RegionAffector.EffectType.dirtiness:
                    dirtiness += affector.effect_level;
                    break;
                case RegionAffector.EffectType.risk:
                    risk += affector.effect_level;
                    break;
            }
        }

        public void RemoveAffector(RegionAffector affector)
        {
            if (!affectors.Contains(affector)) return;
            affectors.Remove(affector);
            switch (affector.effect_type)
            {
                case RegionAffector.EffectType.dirtiness:
                    dirtiness -= affector.effect_level;
                    break;
                case RegionAffector.EffectType.risk:
                    risk -= affector.effect_level;
                    break;
            }
        }

        public void AddRobot(Robot r)
        {
            if (!agents_within.Contains(r))
            {
                agents_within.Add(r);
                density = (float)agents_within.Count / area;
            }
        }

        public void RemoveRobot(Robot r)
        {
            if (agents_within.Contains(r))
            {
                agents_within.Remove(r);
                density = (float)agents_within.Count / area;
            }
        }
    }
}
