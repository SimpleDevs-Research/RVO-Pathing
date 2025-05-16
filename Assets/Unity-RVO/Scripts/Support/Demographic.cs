using System.Collections;
using UnityEngine;
using Random = UnityEngine.Random;
using RVO;

[CreateAssetMenu(fileName = "Demographic", menuName = "Robots/Demographic Group", order = 1)]
public class Demographic : ScriptableObject
{
    public string id;
    public SpawnRate<Personality>[] personalities;
}