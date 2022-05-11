using Cinemachine;
using FishNet;
using FishNet.Object;
using FishNet.Object.Prediction;
using UnityEngine;

enum SpeedType
{
    KPH,
    MPH
}

public class SimpleCarController : NetworkBehaviour
{
    [SerializeField] private GameObject visual;
    [SerializeField] private float maxSteerAngle;
    [SerializeField] private float motorForce;
    [SerializeField] private float brakeForce;
    [SerializeField] private float topSpeed;
    [SerializeField] private SpeedType speedType;
    [SerializeField] private float antiRoll = 1000f;
    [SerializeField] private bool tractionControl = true;
    [SerializeField] private float slipLimit = 0.3f;
    [SerializeField] private bool steeringAssist = true;
    [SerializeField] private float steeringAssistRatio = 0.5f;
    [SerializeField] private int numberOfGears;
    [SerializeField] private AudioSource audioSource;
    [SerializeField] private float minimumPitch;
    [SerializeField] private float maximumPitch;
    [SerializeField] private float boostZoneMultiplier;
    [SerializeField] private WheelCollider[] wheelColliders = new WheelCollider[4];
    [SerializeField] private Transform[] wheelMeshes = new Transform[4];

    #region Types.
    public struct MoveData
    {
        public float Horizontal;
        public float Vertical;
        public MoveData(float horizontal, float vertical)
        {
            Horizontal = horizontal;
            Vertical = vertical;
        }
    }
    public struct ReconcileData
    {
        public Vector3 Position;
        public Quaternion Rotation;
        public Vector3 Velocity;
        public Vector3 AngularVelocity;
        public float RotationInPreviousFrame;
        public int CurrentGear;
        public float FrontLeftSteerAngle;
        public float FrontRightSteerAngle;
        public float FrontLeftMotorTorque;
        public float FrontRightMotorTorque;

        public float FrontLeftBrakeTorque;
        public float FrontRightBrakeTorque;
        public float BackLeftBrakeTorque;
        public float BackRightBrakeTorque;

        public ReconcileData(Vector3 position, Quaternion rotation, Vector3 velocity, Vector3 angularVelocity, float rotationInPreviousFrame, int currentGear,
               float frontLeftSteerAngle, float frontRightSteerAngle,
               float frontLeftMotorTorque, float frontRightMotorTorque,
               float frontLeftBrakeTorque, float frontRightBrakeTorque,
               float backLeftBrakeTorque, float backRightBrakeTorque)
        {
            Position = position;
            Rotation = rotation;
            Velocity = velocity;
            AngularVelocity = angularVelocity;
            RotationInPreviousFrame = rotationInPreviousFrame;
            CurrentGear = currentGear;

            FrontLeftSteerAngle = frontLeftSteerAngle;
            FrontRightSteerAngle = frontRightSteerAngle;
            FrontLeftMotorTorque = frontLeftMotorTorque;
            FrontRightMotorTorque = frontRightMotorTorque;

            FrontLeftBrakeTorque = frontLeftBrakeTorque;
            FrontRightBrakeTorque = frontRightBrakeTorque;
            BackLeftBrakeTorque = backLeftBrakeTorque;
            BackRightBrakeTorque = backRightBrakeTorque;
        }
    }

    #endregion

    private Rigidbody rb;    
    private float horizontalInput;
    private float verticalInput;
    private bool isReversing = false;
    private float rotationInPreviousFrame;
    private int currentGear = 0;
    private float currentSpeed;
    private float gearFactor;
    private float engineRpm;
    private float motorForceWithoutBoost;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        InstanceFinder.TimeManager.OnTick += TimeManager_OnTick;
        InstanceFinder.TimeManager.OnPostTick += TimeManager_OnPostTick;
    }

    private void OnDestroy()
    {
        if (InstanceFinder.TimeManager != null)
        {
            InstanceFinder.TimeManager.OnTick -= TimeManager_OnTick;
            InstanceFinder.TimeManager.OnPostTick -= TimeManager_OnPostTick;
        }
    }

    public override void OnStartClient()
    {
        base.OnStartClient();

        if (base.IsOwner)
        {
            var cfl = GameObject.FindGameObjectWithTag("PlayerFollowCamera").GetComponent<CinemachineFreeLook>();
            cfl.Follow = visual.transform;
            cfl.LookAt = visual.transform;
        }

        Cursor.lockState = CursorLockMode.Locked;
    }

    private void TimeManager_OnTick()
    {
        if (base.IsOwner)
        {
            Reconciliation(default, false);
            CheckInput(out MoveData md);
            Move(md, false);
        }
        if (base.IsServer)
        {
            Move(default, true);
        }
    }


    private void TimeManager_OnPostTick()
    {
        if (base.IsServer)
        {
            ReconcileData rd = new ReconcileData(transform.position, transform.rotation, rb.velocity, rb.angularVelocity, rotationInPreviousFrame, currentGear,
                wheelColliders[0].steerAngle, wheelColliders[1].steerAngle, 
                wheelColliders[0].motorTorque, wheelColliders[1].motorTorque,
                wheelColliders[0].brakeTorque, wheelColliders[1].brakeTorque, wheelColliders[2].brakeTorque, wheelColliders[3].brakeTorque);

            Reconciliation(rd, true);
        }
    }

    [Replicate]
    private void Move(MoveData md, bool asServer, bool replaying = false)
    {
        horizontalInput = md.Horizontal;
        verticalInput = md.Vertical;

        UpdateCurrentSpeed();
        HandleSteering();
        HandleDrive();
        HandleWheelTransform();

        AntiRoll();
        DetectReverse();
        TractionControl();
        SteeringAssist();
        HandleGearChange();
        CalculateEngineRevs();
        HandleAudio();
    }

    [Reconcile]
    private void Reconciliation(ReconcileData rd, bool asServer)
    {
        transform.position = rd.Position;
        transform.rotation = rd.Rotation;
        rb.velocity = rd.Velocity;
        rb.angularVelocity = rd.AngularVelocity;
        rotationInPreviousFrame = rd.RotationInPreviousFrame;
        currentGear = rd.CurrentGear;
        wheelColliders[0].steerAngle = rd.FrontLeftSteerAngle;
        wheelColliders[1].steerAngle = rd.FrontRightSteerAngle;
        wheelColliders[0].motorTorque = rd.FrontLeftMotorTorque;
        wheelColliders[1].motorTorque = rd.FrontRightMotorTorque;
        wheelColliders[0].brakeTorque = rd.FrontLeftBrakeTorque;
        wheelColliders[1].brakeTorque = rd.FrontRightBrakeTorque;
        wheelColliders[2].brakeTorque = rd.BackLeftBrakeTorque;
        wheelColliders[3].brakeTorque = rd.BackRightBrakeTorque;
    }

    private void CheckInput(out MoveData md)
    {
        md = default;

        var horizontal = Input.GetAxis("Horizontal");
        var vertical = Input.GetAxis("Vertical");

        if (horizontal == 0f && vertical == 0f)
            return;

        md = new MoveData(horizontal, vertical);
    }

    private void Start()
    {
        motorForceWithoutBoost = motorForce;       
    }


    private void UpdateCurrentSpeed()
    {
        if (speedType == SpeedType.KPH)
        {
            currentSpeed = rb.velocity.magnitude * 3.6f;
        }
        else
        {
            currentSpeed = rb.velocity.magnitude * 2.23693629f;
        }
    }

    private void HandleSteering()
    {
        wheelColliders[0].steerAngle = maxSteerAngle * horizontalInput;
        wheelColliders[1].steerAngle = maxSteerAngle * horizontalInput;
    }

    private void HandleDrive()
    {
        wheelColliders[0].motorTorque = motorForce * verticalInput / 2;
        wheelColliders[1].motorTorque = motorForce * verticalInput / 2;
        
        if (!isReversing && verticalInput < 0 && rb.velocity.magnitude > 1)
        {
            ApplyBrakes();
        }
        else
        {
            ResetBrakes();
        }
    }

    private void ApplyBrakes()
    {
        for (int i = 0; i < wheelColliders.Length; i++)
        {
            wheelColliders[i].brakeTorque = -brakeForce * verticalInput;
        }
    }

    private void ResetBrakes()
    {
        for (int i = 0; i < wheelColliders.Length; i++)
        {
            wheelColliders[i].brakeTorque = 0f;
        }
    }

    private void HandleWheelTransform()
    {
        for (int i = 0; i < wheelMeshes.Length; i++)
        {
            Vector3 pos = wheelMeshes[i].position;
            Quaternion quat = wheelMeshes[i].rotation;
            
            wheelColliders[i].GetWorldPose(out pos, out quat);

            // adjust because visuals are on different parent
            pos = pos - wheelColliders[i].transform.parent.position + wheelMeshes[i].parent.position;

            wheelMeshes[i].position = pos;
            wheelMeshes[i].rotation = quat;
        }
    }

    private void AntiRoll()
    {
        // Front axle
        ApplyAntiRoll(wheelColliders[0], wheelColliders[1]);
        // Back axle
        ApplyAntiRoll(wheelColliders[2], wheelColliders[3]);
    }

    private void ApplyAntiRoll(WheelCollider left, WheelCollider right)
    {
        // Credits: http://projects.edy.es/trac/edy_vehicle-physics/wiki/TheStabilizerBars
        WheelHit hit;
        float travelLeft = 1f;
        float travelRight = 1f;

        bool isGroundedLeft = left.GetGroundHit(out hit);
        if (isGroundedLeft)
        {
            travelLeft = (-left.transform.InverseTransformPoint(hit.point).y - left.radius) / left.suspensionDistance;
        }
        bool isGroundedRight = right.GetGroundHit(out hit);
        if (isGroundedRight)
        {
            travelRight = (-right.transform.InverseTransformPoint(hit.point).y - right.radius) / right.suspensionDistance;
        }

        float antirollForce = (travelLeft - travelRight) * antiRoll;

        if (isGroundedLeft)
        {
            rb.AddForceAtPosition(left.transform.up * -antirollForce, left.transform.position);
        }
        if (isGroundedRight)
        {
            rb.AddForceAtPosition(right.transform.up * antirollForce, right.transform.position);
        }
    }

    private void DetectReverse()
    {
        float rpmSum = 0f;
        for (int i = 0; i < wheelColliders.Length; i++)
        {
            rpmSum += wheelColliders[i].rpm;
        }
        isReversing = rpmSum / wheelColliders.Length < 0;
    }

    private void TractionControl()
    {
        if (tractionControl)
        {
            WheelHit hit;
            wheelColliders[0].GetGroundHit(out hit);
            if (hit.forwardSlip >= slipLimit && wheelColliders[0].motorTorque > 0)
            {
                wheelColliders[0].motorTorque *= 0.9f;
            }
            wheelColliders[1].GetGroundHit(out hit);
            if (hit.forwardSlip >= slipLimit && wheelColliders[1].motorTorque > 0)
            {
                wheelColliders[1].motorTorque *= 0.9f;
            }
        }
    }

    private void SteeringAssist()
    {
        if (Mathf.Abs(rotationInPreviousFrame - transform.eulerAngles.y) < 10f && steeringAssist)
        {
            var turnadjust = (transform.eulerAngles.y - rotationInPreviousFrame) * steeringAssistRatio;
            Quaternion velocityRotation = Quaternion.AngleAxis(turnadjust, Vector3.up);
            rb.velocity = velocityRotation * rb.velocity;
        }
        rotationInPreviousFrame = transform.eulerAngles.y;
    }

    private void HandleGearChange()
    {
        float speedRatio = Mathf.Abs(currentSpeed / topSpeed);
        float upshiftLimit = 1 / (float) numberOfGears * (currentGear + 1);
        float downshiftLimit = 1 / (float) numberOfGears * currentGear;
        
        if (currentGear > 0 && speedRatio < downshiftLimit)
        {
            currentGear--;
        }

        if (speedRatio > upshiftLimit && (currentGear < (numberOfGears - 1)))
        {
            currentGear++;
        }
    }
    
    private static float ULerp(float from, float to, float value)
    {
        return (1.0f - value) * from + value * to;
    }
    
    private static float CurveFactor(float factor)
    {
        return 1 - (1 - factor) * (1 - factor);
    }

    private void CalculateGearFactor()
    {
        float f = (1/(float) numberOfGears);
        var targetGearFactor = Mathf.InverseLerp(f*currentGear, f*(currentGear + 1), Mathf.Abs(currentSpeed/topSpeed));
        gearFactor = Mathf.Lerp(gearFactor, targetGearFactor, (float) (TimeManager.TickDelta * 5f));
    }
    
    private void CalculateEngineRevs()
    {
        CalculateGearFactor();
        var gearNumFactor = currentGear/(float) numberOfGears;
        var revsRangeMin = ULerp(0f, 1f, CurveFactor(gearNumFactor));
        var revsRangeMax = ULerp(1f, 1f, gearNumFactor);
        engineRpm = ULerp(revsRangeMin, revsRangeMax, gearFactor);
    }

    private void HandleAudio()
    {
        float pitch = ULerp(minimumPitch, maximumPitch, engineRpm);

        if (pitch < minimumPitch)
        {
            pitch = minimumPitch;
        }

        audioSource.pitch = pitch;
    }

    public float GetCurrentSpeed()
    {
        return Mathf.Floor(currentSpeed);
    }

    public void MuteAudio()
    {
        audioSource.volume = 0;
    }
    
    public void ActivateBoost()
    {
        motorForce = motorForceWithoutBoost * boostZoneMultiplier;
    }

    public void DeactivateBoost()
    {
        motorForce = motorForceWithoutBoost;
    }
}
