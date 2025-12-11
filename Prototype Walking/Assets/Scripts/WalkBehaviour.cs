using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.U2D.IK;
using System.Collections;
public class WalkBehaviour : Agent
{
    [SerializeField] private SpriteRenderer walkSprite;
    private Color normalColor;

    [SerializeField] HingeJoint2D[] limbJoints;
    [SerializeField] Transform[] limbs;
    [SerializeField] private float motorSpeed;
    [SerializeField] private float maxMotorForce;

    private ActionSegment<int> currentActions;
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
        currentActions = actions.DiscreteActions;
        shouldMove = true;
        ChooseReward();
        cumalitiveReward = GetCumulativeReward();
    }

    private IEnumerator PositionPenalization(float duration)
    {
        float currentPositionX;
        float lastPositionX = transform.localPosition.x;
        yield return new WaitForSeconds(duration);
        currentPositionX = transform.localPosition.x;
        if (currentPositionX < lastPositionX)
        {
            AddReward(-0.01f);
            walkSprite.color = Color.red;
            Debug.Log("Behind");
        }
        else
        {
            walkSprite.color = normalColor;
        }

    }
    private void ChooseReward()
    {
        //I should do some better Reward shaping 
        //Like for smooth walking or reaching certain positions
        //Makesure to get to the goal Quickly
        AddReward(-1f / MaxStep);
        StartCoroutine(PositionPenalization(3f));
        if (walkSprite != null)
        {
            //Makes Sure agent stays a certain height to insure good walkiong
            if (transform.localPosition.y < -8.2f)
            {
                AddReward(-0.01f);
                //walkSprite.color = Color.red;
            }
            //else
            //{
            //    walkSprite.color = normalColor;
            //}
          
        }
    }

    //Called in FixedUpdate for physics changes
    private void MoveAgent(ActionSegment<int> act)
    {
        //Afterwards try the continuos actions set up instead of discrete

        int actionIndex = 0;
        foreach (HingeJoint2D joint in limbJoints)
        {
            JointMotor2D motor = joint.motor;
            motor.maxMotorTorque = maxMotorForce;
            var action = act[actionIndex];
            switch (action)
            {
                case 2:
                    motor.motorSpeed = motorSpeed;
                    break;
                case 1:
                    motor.motorSpeed = -motorSpeed;
                    break;
                case 3:
                    motor.motorSpeed = 0;
                    break; ;

            }
            joint.motor = motor;
            actionIndex++;
        }

    }
    private void Update()
    {
        ChooseReward();
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
        AddReward(4.0f);
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
            //if (walkSprite != null)
            //{
            //    walkSprite.color = Color.red;
            //}
        }
    }
    private void OnCollisionStay2D(Collision2D collision)
    {
        if (collision.gameObject.CompareTag("Wall"))
        {
            AddReward(-0.025f * Time.fixedDeltaTime);
        }
    }

    private void OnCollisionExit2D(Collision2D collision)
    {
        //if (walkSprite != null)
        //{
        //    walkSprite.color = normalColor;
        //}
    }
}