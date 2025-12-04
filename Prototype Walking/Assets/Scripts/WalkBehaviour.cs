using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
public class WalkBehaviour : Agent
{
    [SerializeField] private GameObject[] limbs;
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
        
        //ResetScene();
    }
    private void ResetScene()
    {
        transform.localPosition = new Vector2(-20, -7.5f);
        transform.localRotation = Quaternion.identity;



    }
    public override void CollectObservations(VectorSensor sensor)
    {
       
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        base.OnActionReceived(actions);
    }

    public void GoalReached()
    {
        AddReward(1.0f);
        cumalitiveReward = GetCumulativeReward();

        EndEpisode();
    }
}
