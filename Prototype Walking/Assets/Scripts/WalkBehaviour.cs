using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
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

        public float limbLength;
    }
    public Limb[] limbs;
    #endregion

    #region Variables
    [Header("Visual Componenets")]
    private SpriteRenderer walkSprite;
    private Color normalColor;

    [Header("Physics varaibles")]
    [SerializeField] private float motorSpeed;
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
    [SerializeField] private Evolution evolution;

    [Header("Evolution Component")]
    [SerializeField] private Dna dna;
    [SerializeField] private float cumalitiveEnergy;
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

        Debug.Log("Reference Height: " + maxReferenceHeight);

        foreach (var limb in limbs)
        {
            limb.limbLength = LimbLength(limb.transform);
            limb.maxMotorForce = MaxMotorForce(limb.limbLength, limb.rb.mass, strengthMultiplier);
            agentVisual.SetBodyMuscles(limb.transform, 3, limb.agentMuscles);
        }

        //transform.localScale = new Vector3(Dna.weight / 50f, Dna.weight / 50f, 1f);
    }

    public override void OnEpisodeBegin()
    {

        currentEpisode++;
        cumalitiveReward = 0f;



        ResetScene();

        //On begin for evolutionary purposes in case of changes in rb

    }
    //If needed add mathf.sin(angle) to be more specific on what torque we need based on the angle of the leg
    //but maxmotor force would be inside of fixedupdate and not on reset

    private float MaxMotorForce(float limbLength, float limbMass, float strengthMultiplier)
    {
        float gravity = Mathf.Abs(Physics2D.gravity.y) * rb2D.gravityScale;

        //τorque = m⋅g⋅r⋅sin(θ)
        float maxMotorForce = (limbMass * gravity * limbLength * strengthMultiplier);
        maxMotorForce = Mathf.Clamp(maxMotorForce, 0f, maxTorque);
        return maxMotorForce;
    }

    //Make sure to know whether this on initialize or not
    //We dont know if a specific length is gonna change or proportionally full body change during evolution
    private float LimbLength(Transform limb)
    {
        Vector2 jointPos = topJoint.position;
        Vector2 limbPos = limb.position;

        float limblength = Vector2.Distance(limbPos, jointPos);
        return limblength;
    }

    private void ResetScene()
    {
        rb2D.linearVelocity = Vector2.zero;
        rb2D.angularVelocity = 0f;

        transform.localPosition = new Vector2(-20, -7.5f);
        transform.localRotation = Quaternion.identity;

        foreach (var limb in limbs)
        {
            //Transform reset
            Vector3 eulerAngles = limb.transform.localEulerAngles;
            eulerAngles.z = Random.Range(-5f, 5f);
            limb.transform.localEulerAngles = eulerAngles;

            //Motor Reset
            JointMotor2D motor = limb.joint.motor;
            motor.motorSpeed = 0;
            limb.joint.motor = motor;

            limb.rb.linearVelocity = Vector2.zero;
            limb.rb.angularVelocity = 0f;

            limb.limbLength = LimbLength(limb.transform);
            limb.maxMotorForce = MaxMotorForce(limb.limbLength, limb.rb.mass, strengthMultiplier);

            //Muscle Redefine
            agentVisual.DefineBodyMuscles(limb.transform, limb.agentMuscles, 4);

        }
        Debug.Log("cumalitiveEnergy: " + cumalitiveEnergy);
        cumalitiveEnergy = 0;

    }
    #endregion

    #region Observations
    public override void CollectObservations(VectorSensor sensor)
    {
        //Rotation & Position of overall Agent
        float positionNormalizedX = transform.localPosition.x / 21f;
        float positionNormalizedY = transform.localPosition.y / 10f;

        //Per Limb
        foreach (var limb in limbs)
        {
            float limbRotationNormalized = (limb.transform.localRotation.eulerAngles.z / 360f) * 2f - 1f;
            sensor.AddObservation(limbRotationNormalized); //try using cosine and sine and check changes later

            float limbAngularVelocityNormalized = limb.rb.angularVelocity / 360f;
            sensor.AddObservation(limbAngularVelocityNormalized);
        }

        //Velocities Check
        sensor.AddObservation(rb2D.linearVelocityX / 6f); // scale / normalize as needed
        sensor.AddObservation(rb2D.linearVelocityY / 6f);

        sensor.AddObservation(positionNormalizedX);
        sensor.AddObservation(positionNormalizedY);

    }

    public void GoalReached()
    {
        AddReward(6.0f);
        cumalitiveReward = GetCumulativeReward();
        gameObject.SetActive(false);

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
        float currentAngleRadians = limb.joint.jointAngle * Mathf.Deg2Rad;
        float currentTorque = MaxMotorForce(limb.limbLength, limb.rb.mass, Mathf.Sin(currentAngleRadians));
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
        //show muscle activation by gradient normalized based on rotational velocity
        int actionIndex = 0;
        foreach (var limb in limbs)
        {
            JointMotor2D motor = limb.joint.motor;
            motor.maxMotorTorque = limb.maxMotorForce;
            var action = act[actionIndex];
            switch (action)
            {
                case 1:
                    motor.motorSpeed = motorSpeed;
                    break;
                case 2:
                    motor.motorSpeed = -motorSpeed;
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