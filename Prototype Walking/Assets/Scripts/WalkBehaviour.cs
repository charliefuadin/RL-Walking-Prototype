using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.VisualScripting;
public class WalkBehaviour : Agent
{
    [SerializeField] private SpriteRenderer walkSprite;
    private Color normalColor;

    [SerializeField] HingeJoint2D[] limbJoints;
    [SerializeField] Transform[] limbs;
    [SerializeField] private float motorSpeed;
    [SerializeField] private float maxMotorForce;

    private ActionSegment<int> currentActions;
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
    }
    public override void OnEpisodeBegin()
    {
        Debug.Log("EpisodeBegin");
        currentEpisode++;
        cumalitiveReward = 0f;

        ResetScene();
        
    }
    private void ResetScene()
    {
        transform.localPosition = new Vector2(-20, -7.5f);
        transform.localRotation = Quaternion.identity;
        foreach (Transform transform in limbs)
        {
            transform.localRotation = Quaternion.identity;
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
        currentActions = actions.DiscreteActions;
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

        //Adds Reward if Agent made any distance
        float currentPositionX = transform.localPosition.x;
        if (currentPositionX < lastPositionX)
        {
            AddReward(0.01f);
        }
        else
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
    private void MoveAgent(ActionSegment<int> act)
    {
        int actionIndex = 0;
        lastPositionX = transform.localPosition.x;
        foreach (HingeJoint2D joint in limbJoints)
        {
            JointMotor2D motor = joint.motor;
            motor.maxMotorTorque = maxMotorForce;
            var action = act[actionIndex];
            switch (action)
            {
                case 0:
                    motor.motorSpeed = 0;
                    break; ;
                case 1:
                    motor.motorSpeed = motorSpeed;
                    break;
                case 2:
                    motor.motorSpeed = -motorSpeed;
                    break;
                
            }
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
}
