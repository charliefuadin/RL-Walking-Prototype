using Unity.InferenceEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.U2D.IK;
public class WalkBehaviour : Agent
{
    [SerializeField] private SpriteRenderer walkSprite;
    private Color normalColor;

    [SerializeField] HingeJoint2D[] limbJoints;
    [SerializeField] Transform[] limbs;
    [SerializeField] Collider2D[] legs;

    [SerializeField] private float MOTORSPEED = 100;
    [SerializeField] private float MOTORFORCE = 100;

    private ActionBuffers currentActions;
    private float lastPositionX;
    private bool shouldMove = false;

    [HideInInspector] public int currentEpisode = 0;
    [HideInInspector] public float cumalitiveReward = 0f;
    public override void Initialize()
    {
        Debug.Log("Initialized");
        normalColor = walkSprite.color;
        currentEpisode = 0;
        cumalitiveReward = 0;
        foreach (var leg in GameObject.FindGameObjectsWithTag("Leg"))
        {
            if (leg.gameObject.CompareTag("Leg"))
            {
                leg.layer = LayerMask.NameToLayer("Legs");
            }
        }
        Physics2D.IgnoreLayerCollision
        (
        LayerMask.NameToLayer("Legs"),
        LayerMask.NameToLayer("Legs"),
        true
        );
    }

    public override void OnEpisodeBegin()
    {

        Debug.Log("EpisodeBegin" + currentEpisode);
        currentEpisode++;
        cumalitiveReward = 0f;

        ResetScene();
    }

    private void ResetScene()
    {
        transform.localPosition = new Vector2(-20, -7.5f);
        transform.localRotation = Quaternion.identity;
        foreach (Transform limb in limbs)
        {
            Vector3 eulerAngles = limb.localEulerAngles;
            eulerAngles.z = 0f;
            limb.localEulerAngles = eulerAngles;
        }
        foreach (HingeJoint2D joint in limbJoints)
        {
            JointMotor2D motor = joint.motor;
            motor.motorSpeed = 0;
            joint.motor = motor;
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        //Rotation & Position of overall Agent
        float positionNormalizedX = transform.localPosition.x / 20;
        float positionNormalizedY = transform.localPosition.y / -6.5f;
        float rotationNormalized = (transform.localRotation.eulerAngles.z / 360f) * 2f - 1f;

        //Rotatoin per Limb
        foreach (Transform transform in limbs)
        {
            float limbRotationNormalized = (transform.localRotation.eulerAngles.z / 360f) * 2f - 1f;
            sensor.AddObservation(limbRotationNormalized);
        }

        sensor.AddObservation(positionNormalizedX);
        sensor.AddObservation(positionNormalizedY);
        sensor.AddObservation(rotationNormalized);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {

        currentActions = actions;
        shouldMove = true; 
        ChooseReward();
        cumalitiveReward = GetCumulativeReward();
    }

    private void ChooseReward()
    {
        //I should do some better Reward shaping 
        //Like for smooth walking or reaching certain positions

        //Makesure to get to the goal Quickly
        AddReward(-1f / MaxStep);

        float currentPositionX = transform.localPosition.x;
        if (currentPositionX < lastPositionX)
        {
            AddReward(-0.01f);
        }

        //Makes Sure agent stays a certain height to insure good walkiong
        if (transform.localPosition.y < -8.3f)
        {
            AddReward(-0.01f);
            walkSprite.color = Color.red;
        }
        else
        {
            walkSprite.color = normalColor;
        }
   
    }

    //Called in FixedUpdate for physics changes
    private void MoveAgent(ActionBuffers actionBuffers)
    {
        //Afterwards try the continuos actions set up instead of discrete
        lastPositionX = transform.localPosition.x;
        int actionIndex = 0;
        int offset = 20;
        foreach (HingeJoint2D joint in limbJoints)
        {

            JointMotor2D motor = joint.motor;
            var motorSpeedSignal = actionBuffers.ContinuousActions[actionIndex];
            var maxMotorForceSignal = actionBuffers.ContinuousActions[actionIndex];

            motor.motorSpeed = motorSpeedSignal * MOTORSPEED + (offset * motorSpeedSignal);
            motor.maxMotorTorque =   Mathf.Abs(maxMotorForceSignal) * MOTORFORCE + offset;
            joint.motor = motor;

            actionIndex++;
        }

    }

    private void FixedUpdate()
    {
        if (shouldMove)
        {
            MoveAgent(currentActions);
            shouldMove = false; //reset the flag after its use
        }
    }

    public void GoalReached()
    {
        AddReward(5.0f);
        cumalitiveReward = GetCumulativeReward();

        EndEpisode();
    }

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
        foreach (var collider in GameObject.FindGameObjectsWithTag("Leg"))
        {
            if (collider.gameObject.CompareTag("Wall"))
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
        foreach (var collider in GameObject.FindGameObjectsWithTag("Leg"))
        {
            if (collider.gameObject.CompareTag("Wall"))
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
}
