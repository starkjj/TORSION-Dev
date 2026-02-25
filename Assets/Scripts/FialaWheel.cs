using UnityEngine;

public class FialaWheel : Wheel
{
    private float deltaTime;
    public Rigidbody vehicleBody;

    [Header("Hit Detection - Inputs")]
    public LayerMask layerMask;
    [Header("Hit Detection - Outputs")]
    public bool isGrounded;
    private RaycastHit hit;
    
    [Header("Suspension - Inputs")]
    public float restLength;
    public float springStiffness;
    public float damperStiffness;
    [Header("Suspension - Outputs")]
    public Vector3 fZ;
    public float currentLength;
    public float lastLength;

    [Header("Wheel Motion - Inputs")]
    public float driveTorque;
    public float wheelRadius;
    public float wheelInertia;
    [Header("Wheel Motion - Outputs")]
    public float wheelAngularVelocity;
    public Vector3 linearVelocityLocal;
    public float totalTorque;
    public Vector3 angularVelocityLocal;
    public Vector3 longitudinalDir;
    public Vector3 lateralDir;

    [Header("Tire Model (Fiala Brush) - Inputs")]
    public float corneringStiffness = 50000f;     // C_alpha (N/rad)
    public float longitudinalStiffness = 50000f;  // C_kappa (N/slip_ratio)
    public float frictionCoefficient = 1.0f;      // Peak friction (mu)

    [Header("Friction - Outputs")]
    public Vector3 fX;
    public Vector3 fY;
    public Vector3 outputForce;
    public float slipAngle;
    public float muX;
    public float muY;
    public float slipRatio; // Replaced slipSpeed with normalized slipRatio

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
            CalculateCombinedFriction();
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
        // Hooke's Law
        float springDisplacement = restLength - currentLength;
        float springForce = springDisplacement * springStiffness;

        // Damping Equation
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

    void CalculateCombinedFriction()
    {
        float normalForce = Mathf.Max(fZ.magnitude, 0.0f);
        
        // Prevent division by zero and unnecessary math if there's no load
        if (normalForce < 0.001f)
        {
            muX = muY = 0f;
            return;
        }

        // 1. Calculate Individual Slips
        // Lateral (tan of slip angle, often denoted as Sy)
        float vx = Mathf.Max(Mathf.Abs(linearVelocityLocal.z), 0.1f);
        float sy = -linearVelocityLocal.x / vx; 
        slipAngle = Mathf.Atan(sy); 

        // Longitudinal (slip ratio, often denoted as Sx or kappa)
        float wheelLinearVelocity = wheelAngularVelocity * wheelRadius;
        slipRatio = (wheelLinearVelocity - linearVelocityLocal.z) / vx;

        // 2. Calculate Combined Slip Magnitude
        float combinedSlip = Mathf.Sqrt((slipRatio * slipRatio) + (sy * sy));

        if (combinedSlip < 0.001f)
        {
             muX = muY = 0f;
             return;
        }

        // 3. Blend Stiffness
        // Tires usually have different lateral and longitudinal stiffness. 
        // We blend them based on which direction is slipping more.
        float slipRatioWeight = Mathf.Abs(slipRatio) / combinedSlip;
        float combinedStiffness = Mathf.Lerp(corneringStiffness, longitudinalStiffness, slipRatioWeight);
        
        // 4. Evaluate Total Force using the Fiala model
        // Notice we evaluate ONCE using the combined slip and blended stiffness
        float totalForceMagnitude = EvaluateFiala(combinedSlip, combinedStiffness, frictionCoefficient, normalForce);

        // 5. Resolve Force Back into Components
        // We ensure totalForceMagnitude is treated as a positive scalar here, 
        // the signs are handled by the slip components
        float forceLong = Mathf.Abs(totalForceMagnitude) * (slipRatio / combinedSlip);
        float forceLat = Mathf.Abs(totalForceMagnitude) * (sy / combinedSlip);

        // 6. Calculate Torques & Integrate Wheel Speed
        // Only the longitudinal force applies torque back to the wheel
        float frictionTorque = forceLong * wheelRadius;
        totalTorque = driveTorque - frictionTorque;

        float wheelAngularAcceleration = totalTorque / wheelInertia;
        wheelAngularVelocity += wheelAngularAcceleration * deltaTime;

        // 7. Store as coefficients for your existing ApplyFrictionForce() method
        muY = forceLong / normalForce;
        muX = forceLat / normalForce;
    }
    
    // Fiala Tire Model Implementation
    float EvaluateFiala(float slip, float stiffness, float mu, float normalForce)
    {
        // Avoid division by zero
        if (normalForce <= 0.001f || mu <= 0.001f) return 0f;

        float absSlip = Mathf.Abs(slip);
        
        // Non-dimensional slip parameter (z)
        float z = (stiffness * absSlip) / (3.0f * mu * normalForce);
        
        float force = 0f;
        if (z < 1.0f)
        {
            // Elastic/transient region (Brush bristles are stretching but not fully sliding)
            force = mu * normalForce * 3.0f * z * (1.0f - z + (z * z) / 3.0f);
        }
        else
        {
            // Sliding region (Brush bristles have broken traction)
            force = mu * normalForce;
        }

        return force * Mathf.Sign(slip);
    }
    
    void CalculateLateralFriction()
    {
        float normalForce = Mathf.Max(fZ.magnitude, 0.0f); // Use magnitude to account for slopes

        // Calculate Slip Angle (alpha)
        // Added epsilon to prevent division by zero at standstill
        slipAngle = Mathf.Atan2(-linearVelocityLocal.x, Mathf.Max(Mathf.Abs(linearVelocityLocal.z), 0.1f)); 
        
        float forceLat = EvaluateFiala(slipAngle, corneringStiffness, frictionCoefficient, normalForce);
        
        // Store as a coefficient to fit into your existing ApplyFrictionForce architecture
        muX = normalForce > 0.001f ? (forceLat / normalForce) : 0f;
    }

    void CalculateLongitudinalFriction()
    {
        float normalForce = Mathf.Max(fZ.magnitude, 0.0f);

        // Calculate Torque Acting On Wheel
        float frictionTorque = muY * normalForce * wheelRadius;
        totalTorque = driveTorque - frictionTorque;

        // Integrate Angular Velocity
        float wheelAngularAcceleration = totalTorque / wheelInertia;
        wheelAngularVelocity += wheelAngularAcceleration * deltaTime;

        // Calculate Longitudinal Slip Ratio (kappa)
        // Brush models require normalized slip ratio rather than raw slip speed
        float wheelLinearVelocity = wheelAngularVelocity * wheelRadius;
        float vx = linearVelocityLocal.z;
        slipRatio = (wheelLinearVelocity - vx) / Mathf.Max(Mathf.Abs(vx), 0.1f);

        // Map Wheel Slip To Fiala Friction Curve
        float forceLong = EvaluateFiala(slipRatio, longitudinalStiffness, frictionCoefficient, normalForce);
        
        muY = normalForce > 0.001f ? (forceLong / normalForce) : 0f;
    }

    void ApplyFrictionForce()
    {
        float normalForce = Mathf.Max(fZ.magnitude, 0.0f);
        fX = lateralDir * muX * normalForce; // F_lat
        fY = longitudinalDir * muY * normalForce; // F_long
        outputForce = (fX + fY);
        
        vehicleBody.AddForceAtPosition(outputForce, hit.point); 
    }

    void ResetValues()
    {
        lastLength = currentLength = restLength; 
        slipAngle = slipRatio = 0.0f; 
        muX = muY = 0.0f; 
        fX = fY = fZ = Vector3.zero; 
    }

    void GetWheelMotionInAir()
    {
        float netTorque = driveTorque;

        float wheelAngularAcceleration = netTorque / wheelInertia;
        wheelAngularVelocity += wheelAngularAcceleration * deltaTime;
    }
}