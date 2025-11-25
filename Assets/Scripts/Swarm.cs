using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.UIElements;

public class Swarm : MonoBehaviour
{
    public struct BBoid
    {
        public Vector3 position;
        public Vector3 forward;
        public Vector3 velocity;
        public Vector3 alignment;
        public Vector3 cohesion;
        public Vector3 separation;
        public Vector3 obstacle;
        public Vector3 currentTotalForce;
    }

    public Transform boidPrefab;

    public int numberOfBoids = 200;

    public float boidForceScale = 20f;

    public float maxSpeed = 5.0f;

    public float rotationSpeed = 40.0f;

    public float obstacleCheckRadius = 1.0f;

    public float separationWeight = 1.1f;
    
    public float alignmentWeight = 0.5f;

    public float cohesionWeight = 1f;

    public float goalWeight = 1f;

    public float obstacleWeight = 0.9f;

    public float wanderWeight = 0.3f;

    public float neighbourDistance = 2.0f;

    public float initializationRadius = 1.0f;

    public float initializationForwardRandomRange = 50f;

    private BBoid[] boids;

    private Transform[] boidObjects;

    private float sqrNeighbourDistance;

    private Vector3 boidZeroGoal;
    private NavMeshPath boidZeroPath;
    private int currentCorner;
    private bool boidZeroNavigatingTowardGoal = false;


    /// <summary>
    /// Start, this function is called before the first frame
    /// </summary>
    private void Start()
    {
        InitBoids();
    }

    /// <summary>
    /// Initialize the array of boids
    /// </summary>
    private void InitBoids()
    {
        boids = new BBoid[numberOfBoids];
        boidObjects = new Transform[numberOfBoids];

        for (int i = 0; i < numberOfBoids; i++)
        {
            boids[i] = new BBoid();

            var dir = Quaternion.Euler(UnityEngine.Random.Range(0f, 360f),UnityEngine.Random.Range(0f, 360f),UnityEngine.Random.Range(0f, 360f));
            var distance = Mathf.Pow(UnityEngine.Random.Range(0f, initializationRadius),1f/3f);
            boids[i].position = transform.position + dir * Vector3.forward * distance;

            dir = Quaternion.Euler(UnityEngine.Random.Range(-initializationForwardRandomRange, initializationForwardRandomRange),
            UnityEngine.Random.Range(-initializationForwardRandomRange, initializationForwardRandomRange),
            UnityEngine.Random.Range(-initializationForwardRandomRange, initializationForwardRandomRange));
            boids[i].forward = dir.eulerAngles;
            print(dir);

            boidObjects[i] = Instantiate(boidPrefab, boids[i].position, dir);
        }
    }


    /// <summary>
    /// Reset the particle forces
    /// </summary>
    public void ResetBoidForces()
    {
        for (int i = 0; i < numberOfBoids; i++)
        {
            boids[i].currentTotalForce = Vector3.zero;
        }
    }


    /// <summary>
    /// Sim Loop
    /// </summary>
    private void FixedUpdate()
    {
        
    }


    private void Update()
    {
        /* Render information for boidzero, useful for debugging forces and path planning
        int boidCount = boids.Length;
        for (int i = 1; i < boidCount; i++)
        {
            Vector3 boidNeighbourVec = boids[i].position - boids[0].position;
            if (boidNeighbourVec.sqrMagnitude < sqrNeighbourDistance &&
                    Vector3.Dot(boidNeighbourVec, boids[0].forward) > 0f)
            { 
                Debug.DrawLine(boids[0].position, boids[i].position, Color.blue);
            }
        }
        
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].alignment, Color.green);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].separation, Color.magenta);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].cohesion, Color.yellow);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].obstacle, Color.red);

        if (boidZeroPath != null)
        {
            int cornersLength = boidZeroPath.corners.Length;
            for (int i = 0; i < cornersLength - 1; i++)
                Debug.DrawLine(boidZeroPath.corners[i], boidZeroPath.corners[i + 1], Color.black);
        }
        */
    }


    public void SetGoal(Vector3 goal)
    {

    }
}

