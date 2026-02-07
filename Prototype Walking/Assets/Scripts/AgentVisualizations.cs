using System.Linq;
using UnityEngine;
using UnityEngine.U2D.IK;

public class AgentVisualizations : MonoBehaviour
{
    [Header("Muscle Components")]
    [SerializeField] GameObject musclePrefab;
    public class Muscle
    {
        public GameObject muscleObject;
        public SpriteRenderer muscleSprite;
        public float initalMuscleWidth = 0.5f;

        public bool muscleActivated = false;
        public float smoothTime = 0.5f;
        public float velocity = 0.0f;

        public Muscle(GameObject muscleObject)
        {
            this.muscleObject = muscleObject;
            this.muscleSprite = muscleObject.GetComponent<SpriteRenderer>();
        }
    }

    [Header("Bone Components")]
    [SerializeField] GameObject bonePrefab;
    [SerializeField] private float initialBoneWidth; //Gets less transparent

    public void SetBodyMuscles(Transform limbTransform, float muscleGenome, Muscle[] muscles) //make sure genome is used as a parameter later
    {   
        GameObject muscle;
        float sectionSeperation = limbTransform.localScale.y / 2 ;

        for (int i = 0; i < 2; i++)
        {
            sectionSeperation = -sectionSeperation; //Reverses Y pos
            muscle = Instantiate(musclePrefab, new Vector3(limbTransform.position.x, limbTransform.position.y, limbTransform.position.z), Quaternion.identity, limbTransform);
            
            Vector3 setScale = muscle.transform.localScale;
            setScale.y = sectionSeperation;
            muscle.transform.localScale = setScale;

            Vector3 setSection = muscle.transform.localPosition;
            setSection.y = sectionSeperation;
            muscle.transform.localPosition = setSection;


            muscles[i] = new Muscle(muscle);           
        }
        DefineBodyMuscles(limbTransform, muscles, muscleGenome);
    }

    public void DefineBodyMuscles(Transform limbTransform, Muscle[] muscles, float muscleGenome) 
    {
        foreach (var muscle in muscles)
        {
            float muscleWidthPercentage = 0.5f; //turn into an equation from muscle genome later P.S FOR YOU KAMAL
            Vector3 setScale = muscle.muscleObject.transform.localScale; 
            setScale.x = muscleWidthPercentage;
            muscle.muscleObject.transform.localScale = setScale;
        }
    }

    public void BodyMuscleActivation(Muscle[] muscles, Rigidbody2D limbRB)
    {
        float maxValue = 255;
        float intercept = 50f;
        float goalValue;

        foreach (var muscle in muscles)
        {
            if (muscle.muscleActivated)
            {
                goalValue = maxValue/maxValue;
            }
            else
            {
                goalValue = intercept/maxValue;
            }
            Color currentColor = muscle.muscleSprite.color;
            currentColor.a = Mathf.SmoothDamp(currentColor.a, goalValue, ref muscle.velocity, muscle.smoothTime);
            muscle.muscleSprite.color = currentColor;
        }
    }

    ///<summary>
    ///change float val based off muscleGenome and different percentage
    ///Example setBody muscle as 10% of its possible max value kinda like that, which is probably 50 percent width 
    ///0% genome is probably the size of the bone as the muscle should be a capsule to the bone, so it be seen 
    ///muscle has to be bigger/wider
    /// </summary>

    public void SetBodyBones(Transform limbTransform, Rigidbody2D limbRb, float boneGenome)
    {
        ///<summary>
        ///Show and visualize bone stress, size, and density
        /// </summary>
    }

}
