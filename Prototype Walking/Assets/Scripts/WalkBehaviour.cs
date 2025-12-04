using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.VisualScripting;
public class WalkBehaviour : Agent
{
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
        }

    }

    private void MoveAgent(ActionSegment<int> act)
    {
        lastPositionX = transform.localPosition.x;
        var action = act[0];
        foreach (HingeJoint2D joint in limbJoints)
        {
            JointMotor2D motor = joint.motor;
            motor.maxMotorTorque = maxMotorForce;
            joint.motor = motor;
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
            }

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
