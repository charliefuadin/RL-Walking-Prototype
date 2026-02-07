using JetBrains.Annotations;
using NUnit.Framework;
using System;
using System.Runtime.InteropServices;
using Unity.MLAgents;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.U2D.IK;
using Random = UnityEngine.Random;


public enum Genome
{
    //add basic attributes here if you want more variables
    //some of these affect eachother
    stamina, // Index = 0
    bone, // Index = 1
    muscle, // Index = 2
    size,  //Special Variable // Index = 3
    organEfficiency, // Index = 6
    limbLength

}

public class Environment
{
    public GameObject environment;
    public Dna[] agentsDna;
    public float[] geneValue = new float[Enum.GetValues(typeof(Genome)).Length];

    public float[] agentFitnessList;
    public Environment(GameObject environment)
    {
        this.environment = environment;
        this.agentsDna = environment.GetComponentsInChildren<Dna>();

        this.agentFitnessList = new float[agentsDna.Length];

        for (int i = 0; i < geneValue.Length; i++)
        {
            this.geneValue[i] = Random.Range(1, 8);
        }
    }
}

public class Evolution : MonoBehaviour
{
    [Header("Genetic Componenets")]
    public float[] bestGenes = new float[7];
    private float bestScore = 0f;

    [Header("EnvironmentComponents")]
    [SerializeField] private GameObject environmentObject;
    [SerializeField] private int spawnAmount;
    [SerializeField] private float verticalSpacing;
    private Environment[] environments;

    //Kamal make sure you set the values in the inspector
    private void InitializeEnvironment()
    {
        environments = new Environment[spawnAmount];
        GameObject environment;
        Vector3 currentPosition = new Vector3(0, 4, 0);
        for (int i = 0; i < spawnAmount; i++)
        {
            environment = Instantiate(environmentObject, currentPosition, Quaternion.identity);
            environments[i] = new Environment(environmentObject);
            currentPosition.y += verticalSpacing;
        }
    }

    private void Start()
    {
        InitializeEnvironment();
    }

  
    //Called every environment has done AvergedOutFitness
    public float[] NewGeneSelection(float newFitness, float[] genes)
    {
        Debug.Log("New Best Score Found");
        float fitnessScore = 1 * (newFitness - bestScore);
        foreach (var environment in environments)
        {
            for (int i = 0; i < bestGenes.Length; i++)
            {
                fitnessScore += Math.Abs(genes[i] - bestGenes[i]);
            }

            if (fitnessScore > bestScore)
            {

                bestGenes = genes;
                bestScore = fitnessScore;
            }
        }
        

        return bestGenes;
    }


    // MAKE SURE: THIS IS CALLED AFTER ALL THE ONES INSIDE THE ENVIRONMNET IS IN DEATH STATE
    private float AveragedOutFitness(Environment environment)
    {
        float cumalitiveFitness = 0f;
        float averagedOutFitness = 0f;
        for (int i = 0; i < environment.agentsDna.Length; i++)
        {
            environment.agentFitnessList[i] = environment.agentsDna[i].fitnessScore;
        }

        foreach (float fitness in environment.agentFitnessList)
        {
            cumalitiveFitness += fitness;
        }
        averagedOutFitness = cumalitiveFitness / environment.agentFitnessList.Length;
        return averagedOutFitness;
       
    }


    


    //REMINDER: this mutation code is percent stacking
    //Speed of growith will be exponential in long enough time
    //Mutate all environment except the best
    public void Mutation(float[] bestGenes, float[] geneValue)
    {
        /// <summary>
        /// LOG AND STORE MUTATION EVENTS
        /// AND GET SEEDS
        /// BE ABLE TO RE LOG THIS SEED AND MUTATION HISTORY ONTO A SHEET
        /// KAAMAALALA
        /// </summary>
        foreach (Genome attr in Enum.GetValues(typeof(Genome)))
        {
            int index = (int)attr;  // convert enum to 0,1,2,3
            float percent = Random.Range(-0.05f, 0.05f);
            geneValue[index] = Math.Clamp(geneValue[index] * (1f + percent), 0, 100);
        }

    }
}


