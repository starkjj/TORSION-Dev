using UnityEngine;

public class TMEasyWheel : Wheel
{
    private float deltaTime;
    public Rigidbody vehicleBody;

    [Header("Hit Detection - Inputs")]
    public LayerMask layerMask;
    [Header("Hit Detection - Outputs")]
    public bool isGrounded;
    private RaycastHit hit;
    
    [Header("Suspension - Inputs")]
    public float restLength = 0.5f;
    public float springStiffness = 30000f;
    public float damperStiffness = 3000f;
    [Header("Suspension - Outputs")]
    public Vector3 fZ;
    public float currentLength;
    public float lastLength;

    [Header("Wheel Motion - Inputs")]
    public float driveTorque;
    public float wheelRadius = 0.35f;
    public float wheelInertia = 1.5f;
    [Header("Wheel Motion - Outputs")]
    public float wheelAngularVelocity;
    public Vector3 linearVelocityLocal;
    public float totalTorque;
    public Vector3 angularVelocityLocal;
    public Vector3 longitudinalDir;
    public Vector3 lateralDir;

    [Header("TMEasy Friction - Longitudinal Inputs")]
    public float dfx0 = 17.0f; // Initial stiffness/slope
    public float sxm = 0.12f;  // Slip at peak friction
    public float fxm = 1.0f;   // Peak friction coefficient
    public float sxs = 0.5f;   // Slip at sliding friction
    public float fxs = 0.8f;   // Sliding friction coefficient

    [Header("TMEasy Friction - Lateral Inputs")]
    public float dfy0 = 15.0f; // Initial stiffness/slope
    public float sym = 0.15f;  // Slip at peak friction
    public float fym = 0.95f;  // Peak friction coefficient
    public float sys = 0.6f;   // Slip at sliding friction
    public float fys = 0.75f;  // Sliding friction coefficient

    [Header("Friction - Outputs")]
    public Vector3 fX;
    public Vector3 fY;
    public Vector3 outputForce;
    public float slipAngle;
    public float slipSpeed;
    public float muX;
    public float muY;
    
    // TMEasy specific telemetry
    public float sx; // Normalized longitudinal slip
    public float sy; // Normalized lateral slip

    public void UpdatePhysicsPre(float argDeltaTime)
    {
        deltaTime = argDeltaTime;
        
        if (Physics.Raycast(transform.position, -transform.up, out hit, restLength + wheelRadius, layerMask))
        {
            isGrounded = true;
        }
        else
        {
            isGrounded = false;
        }

        if (!isGrounded) return;
        
        currentLength = hit.distance - wheelRadius;
        CalculateSuspensionForce();
        ApplySuspensionForce();
        GetWheelMotionOnGround();
    }

    public void UpdatePhysicsDrivetrain(float argDeltaTime, float argDriveTorque)
    {
        deltaTime = argDeltaTime;
        driveTorque = argDriveTorque;

        if (isGrounded)
        {
            CalculateTMEasyFriction();
        }
        else
        {
            GetWheelMotionInAir();
        }
    }

    public void UpdatePhysicsPost()
    { 
        if (isGrounded)
        {
            ApplyFrictionForce();
        }
    }

    void CalculateSuspensionForce()
    {
        float springDisplacement = restLength - currentLength;
        float springForce = springDisplacement * springStiffness;

        float springVelocity = (lastLength - currentLength) / deltaTime;
        float damperForce = springVelocity * damperStiffness;

        float suspensionForce = springForce + damperForce;
        fZ = hit.normal.normalized * suspensionForce; 

        lastLength = currentLength; 
    }
    
    void ApplySuspensionForce()
    {
        vehicleBody.AddForceAtPosition(fZ, transform.position); 
    }

    void GetWheelMotionOnGround()
    {
        linearVelocityLocal = transform.InverseTransformDirection(vehicleBody.GetPointVelocity(hit.point));
        angularVelocityLocal = linearVelocityLocal / wheelRadius; 

        longitudinalDir = Vector3.ProjectOnPlane(transform.forward, hit.normal).normalized;
        lateralDir = Vector3.ProjectOnPlane(transform.right, hit.normal).normalized;
    }

    float EvaluateTMEasyCurve(float s, float df0, float sm, float fm, float ss, float fs)
    {
        s = Mathf.Abs(s);
        if (s == 0.0f) return 0.0f;

        if (s <= sm)
        {
            // Region 1: Cubic interpolation from (0,0) to peak (sm, fm) matching initial slope df0
            float sm2 = sm * sm;
            float sm3 = sm2 * sm;
            float A = df0;
            float B = -(2.0f * df0 * sm - 3.0f * fm) / sm2;
            float C = (df0 * sm - 2.0f * fm) / sm3;
            return A * s + B * s * s + C * s * s * s;
        }
        else if (s <= ss)
        {
            // Region 2: Smooth transition from peak (sm, fm) to sliding (ss, fs)
            float t = (s - sm) / (ss - sm);
            return fm + (fs - fm) * (3.0f * t * t - 2.0f * t * t * t);
        }
        else
        {
            // Region 3: Constant sliding friction
            return fs;
        }
    }

    void CalculateTMEasyFriction()
    {
        float normalForce = Mathf.Max(fZ.y, 0.0f);
        if (normalForce < 0.01f)
        {
            ResetFrictionValues();
            return;
        }

        float v_long = linearVelocityLocal.z;
        float v_lat = linearVelocityLocal.x;

        // Regularization velocity to avoid singularity at zero speed
        float v_limit = Mathf.Max(Mathf.Abs(v_long), 0.1f);

        // 1. Calculate Kinematic Slips
        float v_sx = (wheelAngularVelocity * wheelRadius) - v_long;
        float v_sy = -v_lat; 

        sx = v_sx / v_limit;
        sy = v_sy / v_limit;

        // 2. Normalize slips to combine them
        float sx_norm = sx / sxm;
        float sy_norm = sy / sym;
        float s_mag = Mathf.Sqrt(sx_norm * sx_norm + sy_norm * sy_norm);

        if (s_mag > 0.0001f)
        {
            // Evaluate base TMEasy curves using the combined slip magnitude
            float mu_x_curve = EvaluateTMEasyCurve(s_mag * sxm, dfx0, sxm, fxm, sxs, fxs);
            float mu_y_curve = EvaluateTMEasyCurve(s_mag * sym, dfy0, sym, fym, sys, fys);

            // 3. Distribute forces proportionally to slip components
            muY = mu_x_curve * (Mathf.Abs(sx_norm) / s_mag) * Mathf.Sign(sx); // Longitudinal coeff
            muX = mu_y_curve * (Mathf.Abs(sy_norm) / s_mag) * Mathf.Sign(sy); // Lateral coeff
        }
        else
        {
            muX = 0f;
            muY = 0f;
        }

        // 4. Integrate Angular Velocity (Longitudinal dynamics)
        float frictionTorque = muY * normalForce * wheelRadius;
        totalTorque = driveTorque - frictionTorque;

        float wheelAngularAcceleration = totalTorque / wheelInertia;
        wheelAngularVelocity += wheelAngularAcceleration * deltaTime;

        // Telemetry outputs
        slipSpeed = v_sx; 
        slipAngle = Mathf.Atan2(-v_lat, Mathf.Abs(v_long)); 
    }

    void ApplyFrictionForce()
    {
        float normalForce = Mathf.Max(fZ.y, 0.0f);
        
        // F_lat = u_lat * N * latDir
        fX = lateralDir * muX * normalForce; 
        
        // F_long = u_long * N * longDir
        fY = longitudinalDir * muY * normalForce; 
        
        outputForce = (fX + fY);
        
        // Apply combined friction force at the wheel's contact patch
        vehicleBody.AddForceAtPosition(outputForce, hit.point); 
    }

    void ResetValues()
    {
        lastLength = currentLength = restLength; 
        ResetFrictionValues();
        fZ = Vector3.zero; 
    }

    void ResetFrictionValues()
    {
        slipAngle = slipSpeed = sx = sy = 0.0f; 
        muX = muY = 0.0f; 
        fX = fY = Vector3.zero; 
    }

    void GetWheelMotionInAir()
    {
        float wheelAngularAcceleration = driveTorque / wheelInertia;
        wheelAngularVelocity += wheelAngularAcceleration * deltaTime;
    }
}