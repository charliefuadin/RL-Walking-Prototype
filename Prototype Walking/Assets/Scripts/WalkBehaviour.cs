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
    #region Variables
    [Header("Visual Componenets")]
    [SerializeField] private SpriteRenderer walkSprite;
    private Color normalColor;

    [Header("Joint Components")]
    [SerializeField] Collider2D[] limbColliders;
    [SerializeField] HingeJoint2D[] limbJoints;
    [SerializeField] Transform[] limbs;

    [Header("Physics varaibles")]
    [SerializeField] private float motorSpeed;
    [SerializeField] private float maxMotorForce;

    [SerializeField] private float velocityDeadZone;
    private Rigidbody2D rb2D;


    [Header("ML Rewards")]
    [HideInInspector] public int currentEpisode = 0;
    [HideInInspector] public float cumalitiveReward = 0f;
    private float lastPositionX;

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
        StartCoroutine(PositionCheck());
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
        foreach (Transform transform in limbs)
        {
            float limbRotationNormalized = (transform.localRotation.eulerAngles.z / 360f) * 2f - 1f;
            sensor.AddObservation(limbRotationNormalized);
        }
        //Velocities Check
        sensor.AddObservation(rb2D.linearVelocity.x / 10f); // scale / normalize as needed
        sensor.AddObservation(rb2D.linearVelocity.y / 10f);

        sensor.AddObservation(positionNormalizedX);
        sensor.AddObservation(positionNormalizedY);
        sensor.AddObservation(rotationNormalized);
    }

  
    private void ChooseReward()
    {
        //I should do some better Reward shaping 
        //Like for smooth walking or reaching certain positions
        //Makesure to get to the goal Quickly
        AddReward(-1f / MaxStep);
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
    //private IEnumerator PositionCheck()
    //{
    //    lastPositionX = transform.position.x;
    //    yield return new WaitForSeconds(3f);
    //    float currentPositionX = transform.position.x;
    //    float differenceX = currentPositionX - lastPositionX;
    //    if (differenceX < 0)
    //    {
    //        AddReward(-0.01f);
    //        walkSprite.color = Color.red;
    //        Debug.Log("Behind");
    //    }
    //    else
    //    {
    //        walkSprite.color = normalColor;
    //    }
    //    StartCoroutine(PositionCheck());
    //}

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
    }

    //Called in FixedUpdate for physics changes
    private void MoveAgent(ActionSegment<int> act)
    {
        int actionIndex = 0;
        foreach (HingeJoint2D joint in limbJoints)
        {
            JointMotor2D motor = joint.motor;
            motor.maxMotorTorque = maxMotorForce;
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
            joint.motor = motor;
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
 
        foreach (var limb in limbColliders)
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
       
        foreach (var limb in limbColliders)
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