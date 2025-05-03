using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RVO;

public class GenerateRows : Generator
{

    // This is a special variant of `Generator` that forms rows of agents.
    // You give the number of agents per row, then you can designate properties such as if they're staggered or aligned.
    
    [Tooltip("Should the opposing rows be staggered?")]
    public bool staggered = false;
    private bool prev_staggered = false;

    [Tooltip("The distance between rows.")]
    public float row_distance = 10f;
    private float prev_row_distance = 10f;


    protected void OnValidate() {
        // Calculate the row properties based on 
    }


    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
