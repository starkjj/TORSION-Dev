using System;
using UnityEngine;

// float B = 10.0f;	// Stiffness
// float C = 1.9f;		// Shape
// float E = 0.97f;	// Curvature
// float D = 1.0f;     // Peak 

public class BrushWheel : MonoBehaviour
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

    [Header("Friction - Outputs")]
    public Vector3 fX;
    public Vector3 fY;
    public Vector3 outputForce;
    public float slipAngle;
    public float muX;
    public float muY;
    public float slipSpeed;

    [Header("Brush Parameters")] 
    public float cX = 50000f; // Longitudinal Stiffness
    public float cY = 40000f; // Lateral stiffness
    public float kZ = 200000f; // Vertical stiffness
    private float uX_max = 1.3f; // Longitudinal friction coefficients
    private float uX_min = 0.9f; // Longitudinal friction coefficients
    private float uY_max = 1.2f; // Lateral friction coefficients
    private float uY_min = 1.0f; // Lateral friction coefficients
    private float v_s = 7.2f; // Stribeck velocity (?)
    private float v_f; // Tire forward velocity
    private float c_t = 0.001f; // Caster trail
    private float lateralForceNormalized;
    private float longitudinalForceNormalized;
    private float aligningMomentNormalized;
    private float fZ_0; // nominal tire load (?)

    private float fX_p;
    private float fY_p;
    private float mZ_p;

    private float mZ;
    
    
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
            //Calculate the friction force (Fx, Fy)
            muX = CalculateLateral(0, Mathf.Tan(10 * slipAngle * (2 * Mathf.PI / 360.0f))) / fZ.y;
            muY = CalculateLongitudinal(slipSpeed, 0) / fZ.y;
        }
        else
        {
            //Keep the wheel's ability to spin
            GetWheelMotionInAir();
        }
    }

    public void UpdatePhysicsPost()
    { 
        if (isGrounded)
        {
            //Apply the friction force (Fx, Fy)
            ApplyFrictionForce();
        }
    }

    void CalculateSuspensionForce()
    {
        //Hooke's Law
        float springDisplacement = restLength - currentLength;
        float springForce = springDisplacement * springStiffness;

        //Damping Equation
        float springVelocity = (lastLength - currentLength) / deltaTime;
        float damperForce = springVelocity * damperStiffness;

        float suspensionForce = springForce + damperForce;
        fZ = hit.normal.normalized * suspensionForce; //Suspension force acts perpendicular to the contact patch

        //Set the lastLength for the next frame
        lastLength = currentLength; 
    }
    
    void ApplySuspensionForce()
    {
        //Apply the suspension force to the vehicle at the toplink position
        vehicleBody.AddForceAtPosition(fZ, transform.position); 
    }

    void GetWheelMotionOnGround()
    {
        //Get the velocity of the wheel relative to the ground
        //RB.GetPointVelocity Does Not Update w/ Substeps, If There's A Way To Get This Value Without The Use Of RB Functions, We Can Substep The Whole VP Implementation And Keep The Timestep @ 0.02
        linearVelocityLocal = transform.InverseTransformDirection(vehicleBody.GetPointVelocity(hit.point));
        angularVelocityLocal = linearVelocityLocal / wheelRadius; // omega = v / r

        //Lateral and longitudinal directions of motion of the wheel
        longitudinalDir = Vector3.ProjectOnPlane(transform.forward, hit.normal).normalized;
        lateralDir = Vector3.ProjectOnPlane(transform.right, hit.normal).normalized;
    }

    
    /*
     * Tire Stuff
     */

    float CalculateLongitudinal(float sX, float sY)
    {
        var dZ = fZ.y / kZ;
        var dZ_0 = fZ_0 / kZ;
        var a = Mathf.Sqrt(dZ * (2.0f * wheelRadius - dZ));
        var a_0 = Mathf.Sqrt(dZ_0 * (2.0f * wheelRadius - dZ_0));
    
        var kX = cX / (2.0f * Mathf.Pow(a_0, 2.0f));
        var kY = cY / (2.0f * Mathf.Pow(a_0, 2.0f));
    
        var x_c_top = 4.0f * Mathf.Pow(a, 3.0f) * Mathf.Sqrt(Mathf.Pow(kX * sX * uY_max, 2) + Mathf.Pow(kY * sY * uX_max, 2));
        var x_c_bottom = 3 * fZ.y * uX_max * uY_max;
        var x_c = (x_c_top / x_c_bottom) - a;
    
        var v_sx = sX * v_f;
        var u_x = uX_min + ((uX_max - uX_min) / 1 + Mathf.Pow(((v_sx * sX) / v_s), 2));
    
        var slipMag = Mathf.Sqrt((sX * sX) + (sY * sY));
        var frictionTermX = (slipMag > 0.0001f) ? (u_x * sX) / slipMag : 0.0f;

        if (x_c < a)
        {
            var term1 = 0.5f * kX * sX * Mathf.Pow(a - x_c, 2.0f);
            var term2 = frictionTermX * ((fZ.y * (2.0f * a - x_c) * Mathf.Pow(a + x_c, 2.0f)) / (4.0f * Mathf.Pow(a, 3.0f)));
            return term1 + term2;
        }
        else
        {
            return frictionTermX * fZ.y;
        }
    }
    
    float CalculateLateral(float sX, float sY)
    {
        var dZ = fZ.y / kZ;
        var dZ_0 = fZ_0 / kZ;
        var a = Mathf.Sqrt(dZ * (2.0f * wheelRadius - dZ));
        var a_0 = Mathf.Sqrt(dZ_0 * (2.0f * wheelRadius - dZ_0));
    
        var kX = cX / (2.0f * Mathf.Pow(a_0, 2.0f));
        var kY = cY / (2.0f * Mathf.Pow(a_0, 2.0f));
    
        var x_c_top = 4.0f * Mathf.Pow(a, 3.0f) * Mathf.Sqrt(Mathf.Pow(kX * sX * uY_max, 2) + Mathf.Pow(kY * sY * uX_max, 2));
        var x_c_bottom = 3 * fZ.y * uX_max * uY_max;
        var x_c = (x_c_top / x_c_bottom) - a;
    
        var v_sy = sY * v_f;
        var u_y = uY_min + ((uY_max - uY_min) / 1 + Mathf.Pow(((v_sy * sY) / v_s), 2));
    
        var slipMag = Mathf.Sqrt((sX * sX) + (sY * sY));
        var frictionTermY = (slipMag > 0.0001f) ? (u_y * sY) / slipMag : 0.0f;

        if (x_c < a)
        {
            var term1 = 0.5f * kY * sY * Mathf.Pow(a - x_c, 2.0f);
            var term2 = frictionTermY * ((fZ.y * (2.0f * a - x_c) * Mathf.Pow(a + x_c, 2.0f)) / (4.0f * Mathf.Pow(a, 3.0f)));
            return term1 + term2;
        }
        else
        {
            return frictionTermY * fZ.y;
        }
    }
    
    float CalculateMoment(float sX, float sY)
    {
        var dZ = fZ.y / kZ;
        var dZ_0 = fZ_0 / kZ;
        var a = Mathf.Sqrt(dZ * (2.0f * wheelRadius - dZ));
        var a_0 = Mathf.Sqrt(dZ_0 * (2.0f * wheelRadius - dZ_0));
    
        var kX = cX / (2.0f * Mathf.Pow(a_0, 2.0f));
        var kY = cY / (2.0f * Mathf.Pow(a_0, 2.0f));
    
        var x_c_top = 4.0f * Mathf.Pow(a, 3.0f) * Mathf.Sqrt(Mathf.Pow(kX * sX * uY_max, 2) + Mathf.Pow(kY * sY * uX_max, 2));
        var x_c_bottom = 3 * fZ.y * uX_max * uY_max;
        var x_c = (x_c_top / x_c_bottom) - a;
    
        var v_sx = sX * v_f;
        var u_x = uX_min + ((uX_max - uX_min) / 1 + Mathf.Pow(((v_sx * sX) / v_s), 2)); // Using u_x as per original formula
    
        var slipMag = Mathf.Sqrt((sX * sX) + (sY * sY));
        var frictionTermMz = (slipMag > 0.0001f) ? (u_x * sY) / slipMag : 0.0f;

        if (x_c < a)
        {
            var term1 = (1.0f / 6.0f) * kY * sY * Mathf.Pow(a - x_c, 2.0f) * (a + 2.0f * x_c);
            var term2 = frictionTermMz * ((-3.0f * fZ.y * Mathf.Pow((a * a) - (x_c * x_c), 2.0f)) / (16.0f * Mathf.Pow(a, 3.0f)));
            return term1 + term2;
        }
        else
        {
            return 0.0f;
        }
    }
        
        
    void ApplyFrictionForce()
    {
        fX = lateralDir * muX * Mathf.Max(fZ.y, 0.0f); //F_lat = u * N * -latDir
        fY = longitudinalDir * muY * Mathf.Max(fZ.y, 0.0f); // F_long = u * N * -longDir
        outputForce = (fX + fY);
        vehicleBody.AddForceAtPosition(outputForce, hit.point); //Apply the friction force at the wheel's contact patch
    }

    void ResetValues()
    {
        lastLength = currentLength = restLength; //Fully extend suspension

        slipAngle = slipSpeed = 0.0f; //Set wheel slip to zero
        muX = muY = 0.0f; //Set friction coefficients to zero
        fX = fY = fZ = Vector3.zero; //Set forces to zero
    }

    void GetWheelMotionInAir()
    {
        float netTorque = driveTorque;

        float wheelAngularAcceleration = netTorque / wheelInertia;
        wheelAngularVelocity += wheelAngularAcceleration * deltaTime;
    }
}
