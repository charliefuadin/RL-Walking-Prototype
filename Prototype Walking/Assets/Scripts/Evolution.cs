using JetBrains.Annotations;
using NUnit;
using NUnit.Framework;
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Unity.MLAgents;
using Unity.VisualScripting;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.U2D.IK;
using Random = UnityEngine.Random;
using System.Linq;
using UnityEngine.UIElements;



public class Environment
{

    public GameObject environment;
    public WalkBehaviour[] agentsWalkScripts;
    public float[] agentFitnessList;
    public float[] traits;
    
    public Environment(GameObject environment)
    {
        this.environment = environment;
        this.agentsWalkScripts = environment.GetComponentsInChildren<WalkBehaviour>();
        this.agentFitnessList = new float[agentsWalkScripts.Length];
        //Initializes the genome values to start off with.
        
      
            traits = agentsWalkScripts[0].genome;
            for (int i = 0; i < traits.Length; i++)
            {
                float percent = Random.Range(-0.05f, 0.05f);
                traits[i] *= 1f + percent; //Adds a random percentage to the initial genome values to create some variation in the first generation.
            }
            foreach (var walkScript in agentsWalkScripts)
            {
                walkScript.genome = traits;
            }
        

    }
    public void mutation(float mutationRate)
    {
        //mutates the genome values by a radnom percentage in the range of the mutation rate.
        foreach (var walkScript in agentsWalkScripts)
        {
            for (int i = 0; i < walkScript.genome.Length; i++)
            {
                float percent = Random.Range(-mutationRate, mutationRate);
                walkScript.genome[i] *= 1f + percent;
            }
        }
    }
}

public class Evolution : MonoBehaviour
{
    [Header("Gravity Components")]
    [SerializeField] private float gravityLevel;
    [Header("Mutation Componenets")]
    public float mutationRate; //How much the genes will change during mutation
    [SerializeField] private float stopThreshold; //The fitness score at which the algorithm will stop
    private float previousCost;
    private List<float> Scores = new List<float>();

    [Header("Environment Components")]
    [SerializeField] private GameObject environmentObject;
    [SerializeField] private GameObject agentObject;
    [SerializeField] private int environmentAmount;
    [SerializeField] private int agentsAmount;
    [SerializeField] private float verticalSpacing;
    private Environment[] environments;
    
    [Header("CrossOver Componenets")]
    private int Count;
    [SerializeField] private GameObject[] Agents;
    [SerializeField] private int iterationRate; //How much the genes will change during mutation



    void Awake()
    {
        InitializeEnvironment();
        Count = 0;
         // Initial mutation rate, can be adjusted based on experimentation
        Agents = GameObject.FindGameObjectsWithTag("Player");
        Physics2D.gravity = new Vector2(0, gravityLevel);
    }
    private void InitializeEnvironment()
    {
        environments = new Environment[environmentAmount];
        GameObject environment;
        Vector3 currentPosition = new Vector3(0, 4, 0);
       // Adjust the spacing between agents as needed

        for (int i = 0; i < environmentAmount; i++)
        {
            environment = Instantiate(environmentObject, currentPosition, Quaternion.identity);
            for (int j = 0; j < agentsAmount; j++)
            {
                // Adjust the spacing between agents as needed
                
                Instantiate(agentObject, currentPosition, Quaternion.identity,environment.transform);
            }
            environments[i] = new Environment(environment);
            
            currentPosition.y += verticalSpacing;
        }
    }
 
    #region gradient descent
    private void gradientDescent(List<float> scores)
    { 
        float[] currentScores = new float[iterationRate];
        float currentCost = 0f;

        for (int i = 1; i <= currentScores.Length; i++)
        {
            currentScores[i - 1] = scores[scores.Count - i]; 
        }                                                   
        currentCost = costFunction(currentScores);
        if (previousCost != 0)
        {
            mutationRate = mutationRate * (currentCost/ previousCost); // Adjust the mutation rate based on the cost function
        }
        previousCost = currentCost;
        Debug.Log("currentCost: " + currentCost);
        Debug.Log(" Mutation Rate: " + mutationRate);
    }
    private float costFunction(float[] currentScores)
    {
        //Calculate the cost based on the fitness scores of the agents in the environment.
        float cost = 0f;
        for (int i = 0; i < currentScores.Length-1; i++)
        {
            cost += currentScores[i]; // Give more weight to recent scores
        }
        return cost / currentScores.Length;
    }
    #endregion



    #region evolutionary algorithm
    //If you make count active 
    public void checkActive(int currentEpisode)
    { 
        Count++;
        if (Count == Agents.Length)
        {
            crossOver(bestFittness());
            if (currentEpisode % iterationRate == 0)
            {
                gradientDescent(Scores);
            }
            foreach (var env in environments)
            {
                env.mutation(mutationRate);
                foreach (var walkScript in env.agentsWalkScripts)
                {
                    walkScript.EndEpisode();
                }
            }
            Count = 0;
        }
    }
    
    //Step 3
    private void crossOver(Environment[] bestEnvironments)
    {  
        foreach(var env in environments)
        {
            foreach(var walkScript in env.agentsWalkScripts)
            {
                for (int i = 0; i < walkScript.genome.Length; i++)
                {
                    walkScript.genome[i] = (bestEnvironments[0].traits[i] + bestEnvironments[1].traits[i]) / 2f; // Averages the traits of the two best environments to create a new set of genes for the next generation.// Adds a random percentage to the new genes to create some variation in the next generation.
                }
            }
        }
    }
    //Step 2
    private Environment[] bestFittness()
    {
        float best1 = float.MaxValue;
        float best2 = float.MaxValue;
        Environment [] bestEnvironments = new Environment[2];
        for (int i = 0; i < environments.Length; i++)
        {
            float fitness = AveragedOutFitness(environments[i]);
            Debug.Log("fitness: " + fitness);
            if (fitness < best1)
            {
                best2 = best1;
                best1 = fitness;

                bestEnvironments[1] = bestEnvironments[0];
                bestEnvironments[0] = environments[i];
                

            }
            else if (fitness < best2)
            {
                best2 = fitness;
                bestEnvironments[1] = environments[i];
            }
        }   
        Scores.Add(best1);
        Scores.Add(best2);
        return bestEnvironments;
    }
    //Step 1
    private float AveragedOutFitness(Environment environment)
    {
        float cumalitiveFitness = 0f;
        float averagedOutFitness;
        for (int i = 0; i < environment.agentsWalkScripts.Length; i++)
        {
            environment.agentFitnessList[i] = environment.agentsWalkScripts[i].fitnessScore;
        }

        foreach (float fitness in environment.agentFitnessList)
        {
            cumalitiveFitness += fitness;
        }
        averagedOutFitness = cumalitiveFitness / environment.agentFitnessList.Length;
        return averagedOutFitness;
    }
}


#endregion

