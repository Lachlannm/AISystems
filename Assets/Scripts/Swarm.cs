using System;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
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

    public float timeScale = 1f;

    private BBoid[] boids;

    private Transform[] boidObjects;

    private float sqrNeighbourDistance;
    private float fovAngle;

    private Vector3 boidZeroGoal;
    private NavMeshPath boidZeroPath;
    private int currentCorner;
    private bool boidZeroNavigatingTowardGoal = false;

    private Vector3[] newVelocities;
    private Vector3[] newPositions;

    private Vector3 worldBoundMin = new Vector3(-8f, 1f, -8f);
    private Vector3 worldBoundMax = new Vector3(8f, 4f, 8f);

    private Vector3 flockCenter;

    /// <summary>
    /// Start, this function is called before the first frame
    /// </summary>
    private void Start()
    {
        sqrNeighbourDistance = Mathf.Pow(neighbourDistance, 2);
        fovAngle = Mathf.Cos(Mathf.PI/2.0f);
        newVelocities = new Vector3[numberOfBoids];
        newPositions = new Vector3[numberOfBoids];
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

            dir = Quaternion.Euler(UnityEngine.Random.Range(-initializationForwardRandomRange/2.0f, initializationForwardRandomRange/2.0f),
            UnityEngine.Random.Range(-initializationForwardRandomRange/2.0f, initializationForwardRandomRange/2.0f),
            UnityEngine.Random.Range(-initializationForwardRandomRange/2.0f, initializationForwardRandomRange/2.0f));
            boids[i].forward = dir * Vector3.forward;
            boids[i].velocity = boids[i].forward.normalized;

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
            boids[i].separation = Vector3.zero;
            boids[i].cohesion = Vector3.zero;
            boids[i].alignment = Vector3.zero;
            boids[i].obstacle = Vector3.zero;
        }
    }

    /// <summary>
    /// returns a list of boids that are considered neighbours
    /// </summary>
    /// <param name="boidIndex"></param>
    /// <returns></returns>
    public List<int> GetNeighbours(int boidIndex)
    {
        var neighbours = new List<int>();
        for (var i = 0; i < numberOfBoids; i++)
        {
            // we aren't really neighbours with ourselves are we?
            if (i == boidIndex) continue;

            // check if this boid is close and within the FOV
            if (((boids[i].position - boids[boidIndex].position).sqrMagnitude < sqrNeighbourDistance) &&
                Vector3.Dot((boids[i].position - boids[boidIndex].position).normalized,boids[boidIndex].velocity.normalized) > fovAngle)
            {
                neighbours.Add(i);
            }
        }
        return neighbours;
    }

    /// <summary>
    /// Sim Loop
    /// </summary>
    private void FixedUpdate()
    {
        ResetBoidForces();

        var delta = Time.fixedDeltaTime*timeScale;

        for (int i = 0; i < numberOfBoids; i++)
        {
            var neighbours = GetNeighbours(i);

            // neighbour rules
            if (neighbours.Count > 0)
            {
                boids[i].separation = CalculateSeperationForce(i, neighbours);
                boids[i].cohesion += CalculateCohesionForce(i, neighbours);
                boids[i].alignment += CalculateAlignmentForce(i, neighbours);
            }
            else
            {
                boids[i].separation += CalculateWanderForce(i);
            }

            boids[i].obstacle += CalculateObstacleForce(i);

            boids[i].currentTotalForce = boids[i].separation + boids[i].cohesion + boids[i].alignment + boids[i].obstacle;

            // determine new position and velocity
            var accel = boids[i].currentTotalForce;
            newVelocities[i] = boids[i].velocity + accel * delta;

            if (newVelocities[i].magnitude > maxSpeed)
            {
                newVelocities[i] = newVelocities[i].normalized * maxSpeed;
            }

            newPositions[i] = boids[i].position + newVelocities[i] * delta;
        }

        // apply the new values
        for (var i = 0; i < numberOfBoids; i++)
        {
            boids[i].velocity = newVelocities[i];
            boids[i].position = newPositions[i];
            boids[i].forward = boids[i].velocity.normalized;

            boidObjects[i].position = boids[i].position;
            boidObjects[i].LookAt(boids[i].position+boids[i].forward);
        }
    }

    private Vector3 CalculateSeperationForce(int boidIndex, List<int> neighbours)
    {
        var distances = Vector3.zero;

        for (int i = 0; i < neighbours.Count; i++)
        {
            distances += boids[boidIndex].position - boids[neighbours[i]].position;
        }

        return ((distances/neighbours.Count).normalized*boidForceScale - boids[boidIndex].velocity)*separationWeight;
    }

    private Vector3 CalculateCohesionForce(int boidIndex, List<int> neighbours)
    {
        var positions = Vector3.zero;

        for (int i = 0; i < neighbours.Count; i++)
        {
            positions += boids[neighbours[i]].position;
        }

        if (boidIndex == 0) flockCenter = positions/neighbours.Count;

        return ((positions/neighbours.Count - boids[boidIndex].position).normalized*boidForceScale - boids[boidIndex].velocity)*cohesionWeight;
    }

    private Vector3 CalculateAlignmentForce(int boidIndex, List<int> neighbours)
    {
        var directions = Vector3.zero;

        for (int i = 0; i < neighbours.Count; i++)
        {
            directions += boids[neighbours[i]].velocity.normalized;
        }

        return ((directions/neighbours.Count - boids[boidIndex].velocity).normalized*boidForceScale - boids[boidIndex].velocity)*alignmentWeight;
    }

    private Vector3 CalculateWanderForce(int boidIndex)
    {
        return (boids[boidIndex].velocity*boidForceScale - boids[boidIndex].velocity)*wanderWeight;
    }

    private Vector3 CalculateObstacleForce(int boidIndex)
    {
        // obstacle rule
        var colliders = Physics.OverlapSphere(boids[boidIndex].position,obstacleCheckRadius);
        var obstacleForce = Vector3.zero;
        foreach (var collider in colliders)
        {
            obstacleForce += (boids[boidIndex].position - collider.ClosestPointOnBounds(boids[boidIndex].position)).normalized;
        }

        if (boids[boidIndex].position.x > worldBoundMax.x)
        {
            obstacleForce += new Vector3(-1f, 0f, 0f); 
        }
        if (boids[boidIndex].position.y > worldBoundMax.y)
        {
            obstacleForce += new Vector3(0f, -1f, 0f); 
        }
        if (boids[boidIndex].position.z > worldBoundMax.z)
        {
            obstacleForce += new Vector3(0f, 0f, -1f); 
        }

        if (boids[boidIndex].position.x < worldBoundMin.x)
        {
            obstacleForce += new Vector3(1f, 0f, 0f); 
        }
        if (boids[boidIndex].position.y < worldBoundMin.y)
        {
            obstacleForce += new Vector3(0f, 1f, 0f); 
        }
        if (boids[boidIndex].position.z < worldBoundMin.z)
        {
            obstacleForce += new Vector3(0f, 0f, 1f); 
        }

        return (obstacleForce.normalized*boidForceScale - boids[boidIndex].velocity)*obstacleWeight;
    }

    private void Update()
    {
        //Render information for boidzero, useful for debugging forces and path planning
        // int boidCount = boids.Length;
        // for (int i = 1; i < boidCount; i++)
        // {
        //     Vector3 boidNeighbourVec = boids[i].position - boids[0].position;
        //     if (boidNeighbourVec.sqrMagnitude < sqrNeighbourDistance &&
        //             Vector3.Dot(boidNeighbourVec, boids[0].forward) > 0f)
        //     { 
        //         Debug.DrawLine(boids[0].position, boids[i].position, Color.blue);
        //     }
        // }

        var neighbours = GetNeighbours(0);
        for (int i = 0; i < neighbours.Count; i++)
        {
            Debug.DrawLine(boids[0].position, boids[neighbours[i]].position, Color.blue);
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
    }

    public void OnDrawGizmos()
    {
        Gizmos.DrawSphere(flockCenter,0.02f);
    }


    public void SetGoal(Vector3 goal)
    {

    }
}

