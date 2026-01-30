using System;
using System.Drawing;
using System.Xml.Linq;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Rendering;
using Random = UnityEngine.Random;

public class Dna : MonoBehaviour
{
    [HideInInspector] public float fitnessScore = 0f;
    [HideInInspector] public float agentScore = 0f;

    private WalkBehaviour walkScript;

    private void Start()
    {
        walkScript = GetComponent<WalkBehaviour>();
    }

    public class PhysicalTrait
    {
        /// <summary>
        /// make formula for how genome affects physical traits
        /// we can also try reversing the process 
        /// mutating the physical traits and create formulas affecting the genome
        /// We can also make visualsations based on these values like stamina, muscle mass, etc
        /// </summary>
        public float density;
        public float sizePhy;
        public float speed;
        public float strength;
        public float mass;
        //public float maybe density
        //Genome 7 chromosomes these were picked to represent basic attributes of an organism in are simplified simulation

    }

    //public void musculoskeletal()
    //{
    //    //make formulas that affect physical traits based on genome values
    //    //size genome affects size physical trait

    //    physicalTrait.sizePhy = Math.Clamp(genome.geneValue[4], 1, 2.5f);
    //    physicalTrait.strength = Math.Clamp(genome.geneValue[2] * MathF.Pow(physicalTrait.strength, 2 * physicalTrait.sizePhy), 1, 100);
    //    physicalTrait.mass = MathF.Pow(physicalTrait.mass, 3 * physicalTrait.sizePhy);
    //    physicalTrait.speed = Math.Clamp(genome.geneValue[0] * (200 / physicalTrait.sizePhy), 10, 500);

    //}


    public PhysicalTrait physicalTrait = new PhysicalTrait();

    //if deathstate call this KAMAL
    public float GetFitness(float energyAccumlation, float survivalTime)
    {
        float normalizedEnergy = energyAccumlation / Mathf.Abs(Physics.gravity.y);
        float fitness = normalizedEnergy + survivalTime;

        return fitness;
    }

}



