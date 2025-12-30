using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.U2D.IK;
using System.Collections;
using System.Linq;
public class WalkBehaviour : Agent
{
    /// <summary>
    ///Changed the Limb to its own class
    ///Now its like custom variables in cs Academy so easier
    ///Saw this shit on a yt video now im so happy ;)
    /// </summary>
    [System.Serializable]
    public class Limb
    {
        public GameObject gameObject;
        public Transform transform;
        public Rigidbody2D rb;
        public Collider2D collider;
        public HingeJoint2D joint;

        public float maxMotorForce;
    }
    [SerializeField] private Limb[] limbs;

    #region Variables
    [Header("Visual Componenets")]
    private SpriteRenderer walkSprite;
    private Color normalColor;

    [Header("Physics varaibles")]
    [SerializeField] private float motorSpeed;
    [SerializeField] private float maxTorque;
    [SerializeField] private float torqueScaler;
    private Rigidbody2D rb2D;

    [Header("ML Rewards")]
    [SerializeField] private float forwardScale = 1f;
    [SerializeField] private float smoothTimeConstant = 0.25f;
    [SerializeField] private float velocityDeadZone = 0.05f;
    [SerializeField] private float maxRewardPerStep;
    private float smoothedSpeed = 0f;

    [HideInInspector] public int currentEpisode = 0;
    [HideInInspector] public float cumalitiveReward = 0f;

    [Header("ML Actions")]
    private ActionSegment<int> currentActions;
    #endregion


    #region Intializations
    public override void Initialize()
    {
        Debug.Log("Initialized");
        rb2D = GetComponent<Rigidbody2D>();
        normalColor = walkSprite.color;
        currentEpisode = 0;
        cumalitiveReward = 0;

        smoothedSpeed = rb2D.linearVelocityX;

        walkSprite = GetComponent<SpriteRenderer>();

    }

    public override void OnEpisodeBegin()
    {

        Debug.Log("EpisodeBegin" + currentEpisode);
        currentEpisode++;
        cumalitiveReward = 0f;

        ResetScene();
        //On begin for evolutionary purposes in case of changes in rb
        foreach (var limb in limbs)
        {
            float limbLength = LimbLength(limb.rb, limb.transform);
            limb.maxMotorForce = MaxMotorForce(limbLength, limb.rb.mass);
        }
    }
    //If needed add mathf.sin(angle) to be more specific on what torque we need based on the angle of the leg
    //τorque = m⋅g⋅r⋅sin(θ)
    private float MaxMotorForce(float limbLength, float limbMass)
    {
        float gravity = Mathf.Abs(Physics2D.gravity.y) * rb2D.gravityScale;
        float maxMotorForce = (limbMass * gravity * limbLength * torqueScaler);
        maxMotorForce = Mathf.Clamp(maxMotorForce, 0f, maxTorque);
        return maxMotorForce;
    }

    //Make sure to know whether this on initialize or not
    //We dont know if a specific length is gonna change or proportionally full body change during evolution
    private float LimbLength(Rigidbody2D limb, Transform joint)
    {
        Vector2 jointPos = joint.transform.position;
        Vector2 limbPos = limb.transform.position;

        float limblength = Vector2.Distance(limbPos, jointPos);
        return limblength;
    }

    private void ResetScene()
    {
        transform.localPosition = new Vector2(-20, -7.5f);
        transform.localRotation = Quaternion.identity;
        foreach (var limb in limbs)
        {
            //Transform reset
            Vector3 eulerAngles = limb.transform.localEulerAngles;
            eulerAngles.z = 0f;
            limb.transform.localEulerAngles = eulerAngles;

            //Motor Reset
            JointMotor2D motor = limb.joint.motor;
            motor.motorSpeed = 0;
            limb.joint.motor = motor;
        }
        rb2D.linearVelocity = Vector2.zero;
        rb2D.angularVelocity = 0f;

    }
    #endregion

    #region Observations
    public override void CollectObservations(VectorSensor sensor)
    {
        //Rotation & Position of overall Agent
        float positionNormalizedX = transform.localPosition.x / 40;
        float positionNormalizedY = transform.localPosition.y / 9f;
        float rotationNormalized = (transform.localRotation.eulerAngles.z / 360f) * 2f - 1f;

        //Rotatoin per Limb
        foreach (var limb in limbs)
        {
            float limbRotationNormalized = (limb.transform.localRotation.eulerAngles.z / 360f) * 2f - 1f;
            sensor.AddObservation(limbRotationNormalized);
        }
        //Velocities Check
        sensor.AddObservation(rb2D.linearVelocityX / 6f); // scale / normalize as needed
        sensor.AddObservation(rb2D.linearVelocityY / 6f);

        sensor.AddObservation(positionNormalizedX);
        sensor.AddObservation(positionNormalizedY);
        sensor.AddObservation(rotationNormalized);
    }

  
    private void ChooseReward()
    {
        //I should do some better Reward shaping 
        //Like for smooth walking or reaching certain positions
        //Makesure to get to the goal Quickly
        if (walkSprite != null)
        {
            
            HeightCheck();
          
        }
    }
    public void GoalReached()
    {
        AddReward(6.0f);
        cumalitiveReward = GetCumulativeReward();
        EndEpisode();
    }

    /// <summary>
    /// EMA(Exponential Moving Average) based position check
    /// im gonna cry :(
    /// Refine the global variables IE refine the rewards and penalization magnitude
    /// test out collision so no weird movement
    /// FIX THE WEIRD MOVEMENT EVEN THOUGH THEY RUN EFFICIENTLY NOW
    /// Weird movement also based off of the maxmotorforce which i did up in intitialization
    /// </summary>
    private void VelocityReward()
    {
        // Get the current fixed timestep
        float timeStep = Time.fixedDeltaTime;

        // Current horizontal velocity of lebron man
        float currentVelocity = rb2D.linearVelocity.x;

        // Compute alpha for exponential moving average (EMA)
        // EMA smooths out velocity over time for stable reward calculation
        // Formula: alpha = 1 - exp(-dt / tau)
        float alpha = 1f - Mathf.Exp(-timeStep / smoothTimeConstant);

        // Update smoothed speed using linear interpolation
        // Equivalent to EMA: smoothedSpeed = alpha * current + (1-alpha) * previous smoothedspeed
        smoothedSpeed = Mathf.Lerp(smoothedSpeed, rb2D.linearVelocityX, alpha);

        // Apply a deadzone to ignore very small jitters
        float usedSpeed;
        if (Mathf.Abs(smoothedSpeed) > velocityDeadZone)
        {
            // Subtract deadzone so tiny velocities don't contribute to reward
            usedSpeed = Mathf.Sign(smoothedSpeed) * (Mathf.Abs(smoothedSpeed) - velocityDeadZone);
        }
        else
        {
            // Velocity too small, treated as zero
            usedSpeed = 0f;
        }

        // Compute reward for this timestep
        // Reward is proportional to forward speed and timestep
        float rewardThisStep = usedSpeed * forwardScale * timeStep;

        // Clamp reward to avoid spikes
        rewardThisStep = Mathf.Clamp(rewardThisStep, -maxRewardPerStep, maxRewardPerStep);

        // Apply reward to the agent
        AddReward(rewardThisStep);
    }

    //apply a continuous threshold here later
    private void HeightCheck()
    {
        //Makes Sure agent stays a certain height to insure good walkiong
        if (transform.localPosition.y < -8.2f)
        {
            AddReward(-0.005f);
            //walkSprite.color = Color.red;
        }
        //else
        //{
        //    walkSprite.color = normalColor;
        //}
    }
    #endregion

    #region Actions
    public override void OnActionReceived(ActionBuffers actions)
    {
        currentActions = actions.DiscreteActions;
        ChooseReward();
        cumalitiveReward = GetCumulativeReward();
    }
    

    private void FixedUpdate()
    {
        if(currentActions.Length > 0)
        {
            MoveAgent(currentActions);
        }
        VelocityReward();
    }

    //Called in FixedUpdate for physics changes
    private void MoveAgent(ActionSegment<int> act)
    {
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

    private void OnCollisionEnter2D(Collision2D collision)
    {
        if (collision.gameObject.CompareTag("Wall"))
        {
            AddReward(-0.1f);
            if (walkSprite != null)
            {
                walkSprite.color = Color.red;
            }
        }
 
        foreach (var limb in limbs)
        {
            if (limb.gameObject.CompareTag("Wall"))
            {
                AddReward(-0.1f);
            }
        }

    }
    private void OnCollisionStay2D(Collision2D collision)
    {
        if (collision.gameObject.CompareTag("Wall"))
        {
            AddReward(-0.025f * Time.fixedDeltaTime);
        }
       
        foreach (var limb in limbs)
        {
            if (limb.gameObject.CompareTag("Wall"))
            {
                AddReward(-0.025f * Time.fixedDeltaTime);
            }
        }
    }

    private void OnCollisionExit2D(Collision2D collision)
    {
        if (walkSprite != null)
        {
            walkSprite.color = normalColor;
        }
    }
    #endregion
}