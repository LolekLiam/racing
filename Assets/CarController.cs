using UnityEngine;
using UnityEngine.InputSystem;


[RequireComponent(typeof(Rigidbody))]
public class CarController : MonoBehaviour
{
    [System.Serializable]
    public class Wheel
    {
        public WheelCollider collider;
        public Transform mesh;
        [HideInInspector] public Quaternion initialRotation;
    }

    public Wheel[] wheels;

    [Header("Car Settings")]
    public float motorPower = 800f;
    public float brakePower = 3000f;
    public float maxSteerAngle = 30f;
    public float highSpeedSteerAngle = 10f;
    public float steerSpeedThreshold = 25f; // m/s (~90 km/h)
    public float downforce = 50f;


    private Rigidbody rb;
    private float throttle;
    private float steer;
    private bool braking;

    private CarInput input;

    [Header("Wheel References")]
    public Wheel frontLeft;
    public Wheel frontRight;
    public Wheel rearLeft;
    public Wheel rearRight;

    void Awake()
    {
        input = new CarInput();
    }


    void Start()
    {
        rb = GetComponent<Rigidbody>();

        rb.mass = 1200f;
        rb.centerOfMass = new Vector3(0f, -0.8f, 0f);
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.Continuous;

        // Store wheel mesh correction rotations
        foreach (var w in wheels)
        {
            w.initialRotation =
    Quaternion.Inverse(w.collider.transform.rotation) *
    w.mesh.rotation;


        }
    }

    void OnEnable()
    {
        input.Newactionmap.Enable();
    }

    void OnDisable()
    {
        input.Newactionmap.Disable();
    }


    void Update()
    {
        throttle = input.Newactionmap.Throttle.ReadValue<float>();
        steer = input.Newactionmap.Steer.ReadValue<float>();

        if (Mathf.Abs(steer) < 0.05f)
            steer = 0f;

        braking = input.Newactionmap.Brake.IsPressed();
    }


    void FixedUpdate()
    {
        HandleMotor();
        HandleSteering();
        HandleBraking();

        UpdateWheel(frontLeft);
        UpdateWheel(frontRight);
        UpdateWheel(rearLeft);
        UpdateWheel(rearRight);

        ApplyAntiRoll(frontLeft, frontRight, 8000f);
        ApplyAntiRoll(rearLeft, rearRight, 12000f);

        // Yaw damping (prevents spin buildup)
        rb.angularVelocity = new Vector3(
            rb.angularVelocity.x,
            rb.angularVelocity.y * 0.85f,
            rb.angularVelocity.z
        );

        rb.AddForce(-transform.up * downforce * rb.linearVelocity.magnitude);

    }

    void HandleMotor()
    {
        float speed = rb.linearVelocity.magnitude; // m/s
        float speedFactor = Mathf.Clamp01(1f - speed / 30f); // fades torque above ~108 km/h

        float torque = throttle * motorPower * speedFactor;

        rearLeft.collider.motorTorque = torque;
        rearRight.collider.motorTorque = torque;
    }



    void ApplyAntiRoll(Wheel left, Wheel right, float antiRollForce)
    {
        bool leftGrounded = left.collider.GetGroundHit(out WheelHit leftHit);
        bool rightGrounded = right.collider.GetGroundHit(out WheelHit rightHit);

        float leftTravel = 1f;
        float rightTravel = 1f;

        if (leftGrounded)
            leftTravel = (-left.collider.transform.InverseTransformPoint(leftHit.point).y -
                          left.collider.radius) / left.collider.suspensionDistance;

        if (rightGrounded)
            rightTravel = (-right.collider.transform.InverseTransformPoint(rightHit.point).y -
                           right.collider.radius) / right.collider.suspensionDistance;

        float antiRoll = (leftTravel - rightTravel) * antiRollForce;

        if (leftGrounded)
            rb.AddForceAtPosition(left.collider.transform.up * -antiRoll,
                                  left.collider.transform.position);

        if (rightGrounded)
            rb.AddForceAtPosition(right.collider.transform.up * antiRoll,
                                  right.collider.transform.position);
    }


    void HandleSteering()
    {
        float speed = rb.linearVelocity.magnitude;

        float t = Mathf.Clamp01(speed / steerSpeedThreshold);
        float steerLimit = Mathf.Lerp(maxSteerAngle, highSpeedSteerAngle, t * t);


        frontLeft.collider.steerAngle = steer * steerLimit;
        frontRight.collider.steerAngle = steer * steerLimit;
    }



    void HandleBraking()
    {
        float brake = braking ? brakePower : 0f;

        frontLeft.collider.brakeTorque = brake;
        frontRight.collider.brakeTorque = brake;
        rearLeft.collider.brakeTorque = brake;
        rearRight.collider.brakeTorque = brake;
    }

    void UpdateWheel(Wheel w)
    {
        w.collider.GetWorldPose(out Vector3 pos, out Quaternion rot);

        w.mesh.position = pos;

        // Steering (yaw)
        float steerAngle = w.collider.steerAngle;

        // Rolling (rpm to degrees)
        float rollAngle = w.collider.rpm * 6f * Time.fixedDeltaTime;

        Quaternion steerRot = Quaternion.Euler(0f, steerAngle, 0f);
        Quaternion rollRot = Quaternion.Euler(rollAngle, 0f, 0f);

        w.mesh.rotation = steerRot * rollRot;
    }

    void ApplyTractionControl()
    {
        float slipLimit = 0.4f;

        rearLeft.collider.GetGroundHit(out WheelHit leftHit);
        rearRight.collider.GetGroundHit(out WheelHit rightHit);

        if (Mathf.Abs(leftHit.forwardSlip) > slipLimit ||
            Mathf.Abs(rightHit.forwardSlip) > slipLimit)
        {
            rearLeft.collider.motorTorque *= 0.7f;
            rearRight.collider.motorTorque *= 0.7f;
        }
    }

}
