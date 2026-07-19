using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Unity.Mathematics;
using System.Collections.Generic;
using UnityEngine.UIElements;
using Unity.Mathematics.Geometry;
using Random = UnityEngine.Random;

[System.Serializable]

public class WalkBehaviour : Agent
{
    #region Limb Class
    [System.Serializable]
    public class Limb
    {
        public GameObject gameObject;
        public Transform transform;
        public Rigidbody2D rb;
        public Collider2D collider;
        public HingeJoint2D joint;
        public AgentVisualizations.Muscle[] agentMuscles = new AgentVisualizations.Muscle[2];

        public float energyAccumalation;
        public float maxMotorForce;
        public float baseMassLimb;
        public float centerInertia;
    }
    public Limb[] limbs;
    #endregion
    #region Dna class
    public class Dna
    {
        public float size;
        public float strength;
        public float headMass;
        public float limbMass;
        public float speed;
        public float density;
        public float stamina;
        
    }
   
    
    
    public float[] genome;
    #endregion
    #region Variables
    [Header("Visual Componenets")]
    private SpriteRenderer walkSprite;
    private Color normalColor;

    [Header("Data Exporting")]
    private StatsRecorder agentStats;

    [Header("Physics varaibles")]
    [SerializeField] private float targetVelocity;
    [SerializeField] private float baseAccelerationTime;// seconds to targetVelocity
    [SerializeField] private Transform topJoint;

    private Rigidbody2D rb2D;

    [Header("ML Rewards")]
    [SerializeField] private float smoothTimeConstant = 0.25f;
    [SerializeField] private float velocityDeadZone = 0.05f;
    private float smoothedSpeed = 0f;
    private float previousX;

    [SerializeField] private float heightRewardWeight;
    [SerializeField] private float groundTouchHeight;
    [SerializeField] private float baseSpawnY;
    private float currentSpawnY;
    [SerializeField] private Transform floor;
    private float maxReferenceHeight;


    [HideInInspector] public int currentEpisode = 0;
    [HideInInspector] public float cumalitiveReward = 0f;

    [Header("ML Actions")]
    private ActionSegment<int> currentActions;

    [Header("Evolution Component")]
    [SerializeField] private float baseHeadMass;
    private float previousScale;
    private float maxSizeClamp = 2;
    private float minSizeClamp = 0.5f;

    [SerializeField] private Evolution evolution;
    [SerializeField] private float cumalitiveEnergy;
    private AgentVisualizations agentVisual;
    private DecisionRequester decisionRequester;
    public float fitnessScore = 0f;
    Dna dna = new Dna();

    //private Evolution.genome Dna;
    #endregion


    #region Intializations
    public override void Initialize()
    {
        InitializeReferences();
        InitializeAgent();
        InitializeDNA();

        foreach (var limb in limbs)
        {
            agentVisual.SetBodyMuscles(limb.transform, 3, limb.agentMuscles);
            limb.baseMassLimb = limb.rb.mass;
            limb.centerInertia = limb.rb.inertia;
        }
    }

    private void InitializeReferences()
    {
        rb2D = GetComponent<Rigidbody2D>();
        walkSprite = GetComponent<SpriteRenderer>();

        agentVisual = GetComponent<AgentVisualizations>();
        decisionRequester = GetComponent<DecisionRequester>();
        agentStats = Academy.Instance.StatsRecorder;

        evolution = GameObject.Find("EvolutionManager").GetComponent<Evolution>();
        floor = GameObject.Find("Floor").transform;
    }

    private void InitializeAgent()
    {
        normalColor = walkSprite.color;
        currentEpisode = 0;
        cumalitiveReward = 0;

        smoothedSpeed = rb2D.linearVelocityX;
        currentSpawnY = baseSpawnY;
        rb2D.mass = baseHeadMass;

        previousScale = transform.localScale.x;

        previousX = transform.localPosition.x;
    }

    private void InitializeDNA()
    {
       
        
        //Set Physical Traits in Dna for use in evolution and visualization
        dna.size = transform.localScale.x;
        dna.strength = baseAccelerationTime;
        dna.headMass = rb2D.mass; //Look at this one more closely later because of square cube law mix up and having it ot have mass = dna.agnetmass 
        dna.limbMass = limbs[0].rb.mass; // Assuming all limbs start with the same mass
        dna.speed = targetVelocity;
        dna.density = rb2D.mass / (transform.localScale.x * transform.localScale.y);
        dna.stamina = 1000f; // Placeholder value, can be adjusted based on energy system implementation
        genome = new float[] { dna.size, dna.headMass, dna.limbMass, dna.strength, dna.speed, dna.density, dna.stamina };
    }

    private void UpdateDNA()
    {
        //DNA Vals that require physical update
        dna.size = genome[0];
        
        dna.headMass = genome[1];
        dna.limbMass = genome[2];
        //DNA Vals that don't require physical update
        dna.strength = genome[3];
        dna.speed = genome[4];
        dna.density = genome[5];
        dna.stamina = genome[6];
    }

    private void UnFreezeAgents()
    {
        rb2D.bodyType = RigidbodyType2D.Dynamic;
        foreach (var Limb in limbs)
        {
            Limb.rb.bodyType = RigidbodyType2D.Dynamic;
        }
    }

    public override void OnEpisodeBegin()
    {
       
        //UnFreezes Agents limbs and head after check active
        UnFreezeAgents();
        currentEpisode++;
        cumalitiveReward = 0f;
        decisionRequester.enabled = true;
        UpdateDNA();
        SquareCubeLaw();
        GetMaxReferenceHeight();
        ResetAgent();
        foreach (var limb in limbs)
        {
            ResetLimbs(limb);
        }
    }

    private void GetMaxReferenceHeight()
    {
        float totalLimbYValues = 0f;
        foreach (var limb in limbs)
        {
            totalLimbYValues += limb.transform.localScale.y;
        }
        float actualLegHeight = totalLimbYValues / 2; // 2 Leg sections
        float headHeightRadius = transform.localScale.y / 2;
        float groundColliderYVal = floor.localPosition.y + (floor.localScale.y / 2); // 2 is based by radius

        maxReferenceHeight = groundColliderYVal + actualLegHeight + headHeightRadius;
    }
    
    private void SquareCubeLaw()
    {
        dna.size = Mathf.Clamp(dna.size, minSizeClamp, maxSizeClamp); // Limit size to a reasonable range to prevent extreme scaling
        transform.localScale = new Vector3(dna.size, dna.size, 1);

        float scale = dna.size / previousScale;
        dna.strength = dna.strength * Mathf.Pow(scale, 2);
        rb2D.mass  = dna.headMass * Mathf.Pow(scale, 3);

        foreach (var limb in limbs)
        {
            limb.rb.mass = dna.limbMass * Mathf.Pow(scale, 3);
        }
        previousScale = transform.localScale.x;

        dna.headMass = rb2D.mass;
        dna.limbMass = limbs[0].rb.mass; // Assuming all limbs have the same mass
    }
    

    private void GetNewInertia(Limb limb)
    {
        float distance = Vector2.Distance(limb.joint.anchor, limb.rb.centerOfMass);
        float newInertia = limb.centerInertia + (limb.rb.mass * Mathf.Pow(distance, 2));
        //I = Icm + md^2

        limb.rb.inertia = newInertia;
    }


    private float MinimumMotorForce(Limb limb)
    {
        GetNewInertia(limb);
        float intertia = limb.rb.inertia;
        float angularAcceleration = dna.speed/dna.strength; // this value is not exact and should be referenced

        float torque = angularAcceleration * intertia;
        float minimumForce = torque / LimbLength(limb);// F = t/radius*sin

        return minimumForce;
    }

    private float LimbLength(Limb limb)
    {
        Vector2 jointPos = topJoint.position;
        Vector2 limbPos = limb.transform.position;
        float limblength = Vector2.Distance(limbPos, jointPos);

        //Lower leg has shorter limblength
        //Upper leg is carrying shorter leg
        if (limb.gameObject.CompareTag("LowerLeg"))
        {
            limblength /= 2f; 
        }
        else
        {
            limblength *= 2f;
        }
        return limblength;
    }

    private void ResetAgent()
    {
        //Transform reset
        currentSpawnY = baseSpawnY; // Scales with size

        transform.localPosition = new Vector2(-20, currentSpawnY);
        Vector3 eulerAngles = transform.localEulerAngles;
        eulerAngles.z = Random.Range(0, 0);
        transform.localEulerAngles = eulerAngles;

        //Physics reset
        rb2D.linearVelocity = Vector2.zero;
        rb2D.angularVelocity = 0f;

        //Fitness reset
        //Debug.Log("cumalitiveEnergy: " + cumalitiveEnergy);
        cumalitiveEnergy = 0;
    }

    private void ResetLimbs(Limb limb)
    {
        //Transform reset
        Vector3 eulerAnglesLimb = limb.transform.localEulerAngles;
        eulerAnglesLimb.z = Random.Range(-5f, 5f);
        limb.transform.localEulerAngles = eulerAnglesLimb;

        //Motor Reset
        JointMotor2D motor = limb.joint.motor;
        motor.motorSpeed = 0;
        limb.joint.motor = motor;

        //Physics reset
        limb.rb.linearVelocity = Vector2.zero;
        limb.rb.angularVelocity = 0f;
        limb.maxMotorForce = MinimumMotorForce(limb);

        //Muscle redefine
        agentVisual.DefineBodyMuscles(limb.transform, limb.agentMuscles, 4);
    }
    
    private void AgentDeath()
    {
        decisionRequester.enabled = false;
        DataRecorder();
        cumalitiveReward = GetCumulativeReward();
        GetFitness(cumalitiveEnergy); 
        previousScale = transform.localScale.x;
        //freeze agent and limbs to prevent further reward gain until reset
        rb2D.bodyType = RigidbodyType2D.Static;
        foreach (var Limb in limbs)
        {
            Limb.rb.bodyType = RigidbodyType2D.Static;
        }
        evolution.checkActive(currentEpisode);
    }
    #endregion

    #region Observations
    private float AverageLimbLength()
    {
        float totalLimbLength = 0f;
        foreach (var limb in limbs)
        {
            totalLimbLength += LimbLength(limb);
        }
        float averageLimbLength = totalLimbLength / limbs.Length;
        return averageLimbLength;
    }

    private void DataRecorder()
    {
        //General Data
        agentStats.Add("Agent/cumalitiveEnergy", cumalitiveEnergy, StatAggregationMethod.Average);
        agentStats.Add("Agent/mutationRate", evolution.mutationRate, StatAggregationMethod.Average);
        agentStats.Add("Agent/Fitness", fitnessScore, StatAggregationMethod.Average);

        //DNA Data
        agentStats.Add("DNA/Size", dna.size, StatAggregationMethod.Average);
        agentStats.Add("DNA/HeadMass", dna.headMass, StatAggregationMethod.Average);
        agentStats.Add("DNA/LimbMass", dna.limbMass, StatAggregationMethod.Average);
        agentStats.Add("DNA/Strengh", dna.strength, StatAggregationMethod.Average);
        agentStats.Add("DNA/Speed", dna.speed, StatAggregationMethod.Average);
    }

    //Test with Min-Max Scaling later
    public override void CollectObservations(VectorSensor sensor)
    {

        //Rotation & Position of overall Agent
        float positionNormalizedX = transform.localPosition.x / (floor.localScale.x / 2);// normalize with SCALE AND FLOOR later DO NOT FORGET
        float positionNormalizedY = transform.localPosition.y / 6f;// normalize with SCALE later DO NOT FORGET

        //Physics of the agent
        float expectedMaxVelocity = dna.speed * AverageLimbLength();
        float currentVelocity = Vector2.Dot(rb2D.linearVelocity, transform.right); //Dot to make sure relative forward motion and not worldspace
        float laterVelocity = Vector2.Dot(rb2D.linearVelocity, transform.up);

        float normalizedVelocity = math.tanh(currentVelocity/expectedMaxVelocity);
        float normalizedLateralVelocity = math.tanh(laterVelocity / expectedMaxVelocity);

        //Per Limb
        foreach (var limb in limbs)
        {
            float limbRotationNormalized = (limb.transform.localRotation.eulerAngles.z / 360f) * 2f - 1f;
            sensor.AddObservation(limbRotationNormalized);

            float limbAngularVelocityNormalized = limb.rb.angularVelocity / 360f;
            sensor.AddObservation(limbAngularVelocityNormalized);
        }
        
        //DNA Observations
        float normalizedSize = dna.size / maxSizeClamp;
        float sizeScale = maxSizeClamp / dna.size;
        float maxStrength = baseAccelerationTime * Mathf.Pow(sizeScale, 2);
        float maxHeadMass = baseHeadMass * Mathf.Pow(sizeScale, 2);
        float maxLimbMass = limbs[0].baseMassLimb * Mathf.Pow(sizeScale, 2);

        float normalizedStrength = dna.strength / maxStrength;
        float normalizedHeadMass = dna.headMass / maxHeadMass;
        float normalizedLimbMass = dna.limbMass / maxLimbMass;

        sensor.AddObservation(normalizedStrength);
        sensor.AddObservation(normalizedHeadMass);
        sensor.AddObservation(normalizedLimbMass);
        sensor.AddObservation(normalizedSize);

        //Position and Velocities Observations
        sensor.AddObservation(normalizedVelocity);
        sensor.AddObservation(normalizedLateralVelocity);

        sensor.AddObservation(positionNormalizedX);
        sensor.AddObservation(positionNormalizedY);

    }

    public void GetFitness(float energyAccumlation)
    {
        float gravityScale = Mathf.Abs(Physics.gravity.y);
        float normalizedEnergy = energyAccumlation / gravityScale; //Ranges in hundreds
        float normalizeCumalitiveReward = cumalitiveReward * (500 / gravityScale); //500 for 50% of real val (500/1000)
        //**Subtract because higher reward should get us closer to zero
        float fitness = normalizedEnergy - normalizeCumalitiveReward; 

        fitnessScore = Mathf.Clamp(fitness, 0f, Mathf.Infinity);
    }

    private void MovementReward()
    {
        //Real problem was EMA should be used for bonus and not main reward

        // Progress (main reward efficiency)
        float deltaX = transform.localPosition.x - previousX;
        previousX = transform.localPosition.x;
        if(deltaX > 0f)
        {
            AddReward(deltaX * 0.004f);
        }
        else
        {
            AddReward(deltaX * 0.002f);
        }

        // EMA momentum bonus (Style for realism)
        float currentVelocity = rb2D.linearVelocity.x;
        float timeStep = Time.fixedDeltaTime;
        float smoothingFactor = 1f - Mathf.Exp(-timeStep / smoothTimeConstant);
        smoothedSpeed = Mathf.Lerp(smoothedSpeed, currentVelocity, smoothingFactor);

        if (deltaX > 0f && smoothedSpeed > velocityDeadZone)
        {
            float momentum = smoothedSpeed - velocityDeadZone;
            AddReward(momentum * 0.0005f * Time.fixedDeltaTime);
        }
    }

    private void MaxStepReset()
    {
        float currentStep = StepCount;
        if (currentStep > 10000)
        {
            AddReward(-1f);
            AgentDeath();
        }
    }
    #endregion

    #region Actions
    public override void OnActionReceived(ActionBuffers actions)
    {
        MaxStepReset();
        currentActions = actions.DiscreteActions;
        MovementReward();
        AddReward(-0.0002f); //Dont make em waste movement
        cumalitiveReward = GetCumulativeReward();
    }

    private void CalculateCurrentEnergy(Limb limb)
    {
        float currentTorque = limb.joint.GetReactionTorque(Time.fixedDeltaTime);
        float velocity = limb.rb.angularVelocity * Mathf.Deg2Rad;
        // Power * Time = Joules
        float energyJoules = Mathf.Abs(currentTorque * velocity) * Time.fixedDeltaTime;

        cumalitiveEnergy += energyJoules;
        //Reset cumalitive to zero once in deathstate
    }


    private void FixedUpdate()
    {
        if (currentActions.Length > 0)
        {
            MoveAgent(currentActions);
        }

        foreach (var limb in limbs)
        {
            agentVisual.BodyMuscleActivation(limb.agentMuscles, limb.rb);
            CalculateCurrentEnergy(limb);
        }
    }

    //Called in FixedUpdate for physics changes
    private void MoveAgent(ActionSegment<int> act)
    {
        //Later try continous actions on speed only specifically
        int actionIndex = 0;
        foreach (var limb in limbs)
        {
            JointMotor2D motor = limb.joint.motor;
            var action = act[actionIndex];
            motor.maxMotorTorque = limb.maxMotorForce;
            switch (action)
            {
                case 1:
                    motor.motorSpeed = dna.speed;
                    break;
                case 2:
                    motor.motorSpeed = -dna.speed;
                    break;
                case 3:
                    motor.motorSpeed = 0;
                    break; ;
                default:
                    motor.motorSpeed = 0f; // safe fallback
                    break;

            }
            bool isActive = Mathf.Abs(motor.motorSpeed) > 0f;
            foreach (var muscle in limb.agentMuscles)
            {
                muscle.muscleActivated = isActive;
            }
            limb.joint.motor = motor;
            actionIndex++;
        }
        
    }
    private void OnCollisionEnter2D(Collision2D collision)
    {
        if (collision.gameObject.CompareTag("Floor"))
        {
            AddReward(-3f);
            AgentDeath();
        }
    }
    #endregion


    #region Collision Checks
    private void OnTriggerEnter2D(Collider2D collision)
    {
        if (collision.gameObject.CompareTag("Goal"))
        {
            AddReward(3.0f);
            AgentDeath();
        }

    }
    #endregion
}