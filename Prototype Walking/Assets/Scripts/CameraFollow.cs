using UnityEngine;
using static UnityEngine.InputSystem.LowLevel.InputStateHistory;

public class CameraFollow : MonoBehaviour
{
    [SerializeField] private GameObject[] agents;
    private Camera agentCamera;

    [SerializeField] private float smoothTime; // SMOOTH = 1 IF INFERENCE  OR = 3 IF DEFAULT AND RUNNING
     private Vector3 velocity = Vector3.zero;
    void Start()
    {
        agentCamera = GetComponent<Camera>();
        agents = GameObject.FindGameObjectsWithTag("Player");
    }

    // Update is called once per frame
    void Update()
    {
        CameraAgentFollow(FindFarthestAgent());
    }

    private void CameraAgentFollow(Vector2 farthestAgent)
    {
        agentCamera.transform.position = Vector3.SmoothDamp(transform.position, new Vector3(farthestAgent.x, farthestAgent.y, -10), ref velocity, smoothTime);
    }

     private Vector2 FindFarthestAgent()
    {
        float farthestX = -100f;
        Vector2 farthestPosition = Vector2.zero;
        foreach (var agent in agents)
        {
            float tempX = agent.transform.position.x;

            if (tempX > farthestX)
            {
                farthestX = tempX;
                farthestPosition = agent.transform.position; 
            }

        }
        return farthestPosition;
    }
}
