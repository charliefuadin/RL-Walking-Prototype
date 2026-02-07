using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.UIElements;
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

    #region Variables
    [Header("Visual Componenets")]
    private SpriteRenderer walkSprite;
    private Color normalColor;

    [Header("Physics varaibles")]
    [SerializeField] private float targetVelocity;
    [SerializeField] private float baseAccelerationTime;
    private float accelerationTime;// seconds to targetVelocity
    [SerializeField] private float maxTorque;
    [SerializeField] private float strengthMultiplier;
    [SerializeField] private Transform topJoint;

    private Rigidbody2D rb2D;

    [Header("ML Rewards")]
    [SerializeField] private float forwardScale = 1f;
    [SerializeField] private float smoothTimeConstant = 0.25f;
    [SerializeField] private float velocityDeadZone = 0.05f;
    [SerializeField] private float maxRewardPerStep;
    private float smoothedSpeed = 0f;

    [SerializeField] private float heightRewardWeight;
    [SerializeField] private float groundTouchHeight;
    private Transform groundEnvironment;
    private float maxReferenceHeight;


    [HideInInspector] public int currentEpisode = 0;
    [HideInInspector] public float cumalitiveReward = 0f;

    [Header("ML Actions")]
    private ActionSegment<int> currentActions;



    [Header("Evolution Component")]
    [SerializeField] private Dna dna;
    [SerializeField] private Evolution evolution;
    [SerializeField] private float cumalitiveEnergy;
    private float baseAgentScale; //For the square cube law
    private float baseAgentMass;
    private AgentVisualizations agentVisual;
    public bool liveState = true;

    //private Evolution.genome Dna;
    #endregion


    #region Intializations
    public override void Initialize()
    {
        Debug.Log("Initialized");
        rb2D = GetComponent<Rigidbody2D>();
        walkSprite = GetComponent<SpriteRenderer>();
        dna = GetComponent<Dna>();
        agentVisual = GetComponent<AgentVisualizations>();
        groundEnvironment = GetComponentInParent<Transform>();

        normalColor = walkSprite.color;
        currentEpisode = 0;
        cumalitiveReward = 0;

        smoothedSpeed = rb2D.linearVelocityX;

        ////make a calculated ground touch height
        //float colliderY = transform.localPosition.y - (transform.localScale.y / 2);
        //Vector2 currentPosition = new Vector2(0, colliderY);
        //float groundColliderY = groundEnvironment.transform.localPosition.y + (groundEnvironment.transform.localScale.y / 2);
        //Vector2 groundPosition = new Vector2(0, groundColliderY);

        //groundTouchHeight = Vector2.Distance(currentPosition, groundPosition);
        //Debug.Log(groundTouchHeight);
        maxReferenceHeight = transform.localPosition.y + groundTouchHeight;

        accelerationTime = baseAccelerationTime;
        baseAgentScale = transform.localScale.x;
        baseAgentMass = rb2D.mass;

        Debug.Log("Reference Height: " + maxReferenceHeight);

        foreach (var limb in limbs)
        {
            agentVisual.SetBodyMuscles(limb.transform, 3, limb.agentMuscles);
            limb.baseMassLimb = limb.rb.mass;
            limb.centerInertia = limb.rb.inertia;
            Debug.Log("centerInertia " + limb.centerInertia);
        }
    }

    public override void OnEpisodeBegin()
    {
        currentEpisode++;
        cumalitiveReward = 0f;
        ResetAgent();
        foreach (var limb in limbs)
        {
            ResetLimbs(limb);
        }
        //On begin for evolutionary purposes in case of changes in rb

    }
    private void SquareCubeLaw(float newMass, float baseMass) //Seperate Limb later so same assignment doesnt happene multiple times
    {
        float scale = transform.localScale.x / baseAgentScale;
        accelerationTime = baseAccelerationTime * Mathf.Pow(scale, 2);

        newMass = baseMass * Mathf.Pow(scale, 3);
       
    }

    private void GetNewInertia(Limb limb)
    {
        float distance = Vector2.Distance(limb.joint.anchor, limb.rb.centerOfMass);
        float newInertia = limb.centerInertia + (limb.rb.mass * Mathf.Pow(distance, 2));
        //I = Icm + md^2

        limb.rb.inertia = newInertia;
        Debug.Log("newInertia "+limb.rb.inertia);
    }

    
    private float MinimumMotorForce(Limb limb, float targetVelocity)
    {
        GetNewInertia(limb);
        float intertia = limb.rb.inertia;
        float angularAcceleration = targetVelocity / accelerationTime;
        //This value is still an approximation
        //Value for wanted movement should be based off reference

        float torque = angularAcceleration * intertia;
        float minimumForce = torque / LimbLength(limb);// F = t/radius*sin

        Debug.Log("minimumForce " + minimumForce);
        return minimumForce;
    }

    private float LimbLength(Limb limb)
    {
        Vector2 jointPos = topJoint.position;
        Vector2 limbPos = limb.transform.position;
        float limblength = Vector2.Distance(limbPos, jointPos);
        if (limb.gameObject.CompareTag("LowerLeg"))
        {
            limblength -= 0.5f;
        }
        else
        {
            limblength +=0.5f;
        }
            return limblength;
    }

    private void ResetAgent()
    {
        //Transform reset
        transform.localPosition = new Vector2(-20, -7.5f);
        Vector3 eulerAngles = transform.localEulerAngles;
        eulerAngles.z = Random.Range(0, 0);
        transform.localEulerAngles = eulerAngles;

        //Physics reset
        SquareCubeLaw(rb2D.mass, baseAgentMass);
        rb2D.linearVelocity = Vector2.zero;
        rb2D.angularVelocity = 0f;

        //Fitness reset
        Debug.Log("cumalitiveEnergy: " + cumalitiveEnergy);
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
        SquareCubeLaw(limb.rb.mass, limb.baseMassLimb);
        limb.maxMotorForce = MinimumMotorForce(limb, targetVelocity);

        //Muscle redefine
        agentVisual.DefineBodyMuscles(limb.transform, limb.agentMuscles, 4);
    }
    #endregion

    #region Observations
    //Test with Min-Max Scaling later
    public override void CollectObservations(VectorSensor sensor)
    {

        //Rotation & Position of overall Agent
        float positionNormalizedX = transform.localPosition.x / 21f;// normalize with SCALE AND FLOOR later DO NOT FORGET
        float positionNormalizedY = transform.localPosition.y / 10f;// normalize with SCALE later DO NOT FORGET

        //Per Limb
        foreach (var limb in limbs)
        {
            float limbRotationNormalized = (limb.transform.localRotation.eulerAngles.z / 360f) * 2f - 1f;
            sensor.AddObservation(limbRotationNormalized); 

            float limbAngularVelocityNormalized = limb.rb.angularVelocity / 360f;
            sensor.AddObservation(limbAngularVelocityNormalized);
        }

        //Velocities Check
        sensor.AddObservation(rb2D.linearVelocityX / 6f); // normalize based on ABSOLUTE VELOCITY with Min-Max Scaling
        sensor.AddObservation(rb2D.linearVelocityY / 6f);

        sensor.AddObservation(positionNormalizedX);
        sensor.AddObservation(positionNormalizedY);

        //We have to add an observation about size and mass seems like
        //Tested pretrained model's reaction to a different size and mass ratio and couldnt walk the same
        //Test later what we also have to add as an observation
    }

    public void GoalReached()
    {
        AddReward(3.0f);
        cumalitiveReward = GetCumulativeReward();
        EndEpisode();
        //gameObject.SetActive(false);

    }

    private void VelocityReward()
    {
        float timeStep = Time.fixedDeltaTime;
        float currentVelocity = rb2D.linearVelocity.x;

        // Compute alpha for exponential moving average (EMA)
        // Formula: alpha = 1 - exp(-dt / tau)
        float smoothingFactor = 1f - Mathf.Exp(-timeStep / smoothTimeConstant);

        // Equivalent to EMA: smoothedSpeed = alpha * current + (1-alpha) * previous smoothedspeed`
        float previousSmoothedSpeed = smoothedSpeed;
        smoothedSpeed = Mathf.Lerp(previousSmoothedSpeed, rb2D.linearVelocityX, smoothingFactor);

        float usedSpeed;
        if (Mathf.Abs(smoothedSpeed) > velocityDeadZone)
        {
            // Subtract deadzone so tiny velocities don't contribute to reward
            usedSpeed = Mathf.Sign(smoothedSpeed) * (Mathf.Abs(smoothedSpeed) - velocityDeadZone);
        }
        else
        {
            usedSpeed = 0f;
        }

        float rewardThisStep = usedSpeed * forwardScale * timeStep;
        rewardThisStep = Mathf.Clamp(rewardThisStep, -maxRewardPerStep, maxRewardPerStep);

        AddReward(rewardThisStep);
    }

    //Height threshold
    private void HeightCheck()
    {   //TODO Turn groundtouchheight based on localScale.y 
        float currenHeight = transform.localPosition.y + groundTouchHeight;
        float normalizedHeight = currenHeight / maxReferenceHeight; // this is going to be max height later

        float percent = 0.75f; // 75% of standing height
        float reward = Mathf.Clamp((normalizedHeight - percent), -0.05f, 0.05f);

        AddReward(reward * heightRewardWeight);
    }


    #endregion

    #region Actions
    public override void OnActionReceived(ActionBuffers actions)
    {
        currentActions = actions.DiscreteActions;
        HeightCheck();
        if (transform.localPosition.y < -groundTouchHeight + 0.1)
        {
            AddReward(-3f);
            EndEpisode();
        }
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

        //find a way to set this after the height check is achieved
        VelocityReward();
        foreach (var limb in limbs)
        {
            agentVisual.BodyMuscleActivation(limb.agentMuscles, limb.rb);
            CalculateCurrentEnergy(limb);
        }
        if (!liveState)
        {

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
                    motor.motorSpeed = targetVelocity;
                    break;
                case 2:
                    motor.motorSpeed = -targetVelocity;
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
    #endregion


    #region Collision Checks
    private void OnTriggerEnter2D(Collider2D collision)
    {
        if (collision.gameObject.CompareTag("Goal"))
        {
            GoalReached();
        }
    }
    #endregion
}