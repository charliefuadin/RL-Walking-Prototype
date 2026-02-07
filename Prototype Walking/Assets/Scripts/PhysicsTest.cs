using UnityEngine;

public class PhysicsTest : MonoBehaviour
{
    [System.Serializable]
    public class TestObject
    {
        public GameObject gameObject;
        public Transform transform;
        public Rigidbody2D rb;
        public Collider2D collider;
        public HingeJoint2D joint;

        public float baseMassObject;
        public float centerInertia;
        public float previousAngularVelocity;
    }
    public TestObject testObject;
    [SerializeField] private float accelerationTime;
    [SerializeField] private float targetVelocity;
    void Start()
    {
        testObject.baseMassObject = testObject.rb.mass;
        testObject.centerInertia = testObject.rb.inertia;

        JointMotor2D motor = testObject.joint.motor;
        motor.motorSpeed = targetVelocity;
        motor.maxMotorTorque = MinimumMotorForce(testObject, targetVelocity);
        testObject.joint.motor = motor;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if (Mathf.Abs(testObject.rb.angularVelocity) >= targetVelocity)
        {
            Debug.Log(Time.time);
        }
    }

    private void GetNewInertia(TestObject testObject)
    {
        float distance = Vector2.Distance(testObject.joint.anchor, testObject.rb.centerOfMass);
        float newInertia = testObject.centerInertia + (testObject.rb.mass * Mathf.Pow(distance, 2));
        //I = Icm + md^2

        testObject.rb.inertia = newInertia;
        Debug.Log("newInertia " + testObject.rb.inertia);
    }

    private float MinimumMotorForce(TestObject testObject, float targetVelocity)
    {
        GetNewInertia(testObject);
        float intertia = testObject.rb.inertia;
        float angularAcceleration = targetVelocity / accelerationTime; //This value is still an approximation

        float torque = angularAcceleration * intertia;
        float minimumForce = torque / testObject.transform.localScale.x;// F = t/radius*sin

        Debug.Log("minimumForce " + minimumForce);
        return minimumForce;
    }

    private float GetAcceleration(TestObject testObject)
    {
        float currentAngularVeloctiy = testObject.rb.angularVelocity;
        float deltaAngularVelocity = currentAngularVeloctiy - testObject.previousAngularVelocity;
        testObject.previousAngularVelocity = currentAngularVeloctiy;
        float currentAcceleration = deltaAngularVelocity / Time.fixedDeltaTime;
        return currentAcceleration;
    }
}
