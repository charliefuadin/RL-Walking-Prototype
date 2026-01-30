using Google.Protobuf.WellKnownTypes;
using UnityEngine;
using TMPro;
using Unity.VisualScripting;

public class EnvironmentVisualizer : MonoBehaviour
{
    [SerializeField] private GameObject environmentFloor;
    [SerializeField] private Transform cameraPosition;

    [SerializeField] private GameObject distancePrefab;
    [SerializeField] private int distanceSections;
    void Start()
    {
        InstantiateDistancePoints();
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void InstantiateDistancePoints()
    {
        float floorScaleX = environmentFloor.transform.localScale.x;
        float sectionSplit = floorScaleX / distanceSections;

        float currentX = sectionSplit;

        for (int i = 0; i < distanceSections; i++)
        {
            GameObject instance = Instantiate(distancePrefab, new Vector3(currentX - (floorScaleX /2), environmentFloor.transform.position.y, 0f), Quaternion.identity );
            currentX += sectionSplit;
        }
    }
}
