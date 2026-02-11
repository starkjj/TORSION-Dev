using System;
using UnityEngine;

// float B = 10.0f;	// Stiffness
// float C = 1.9f;		// Shape
// float E = 0.97f;	// Curvature
// float D = 1.0f;     // Peak 

[Serializable]
public struct HitData
{
    public float distance;
    public Vector3 normal;
    public Vector3 point;
    public int totalHit;

    public void Reset()
    {
        distance = 0;
        totalHit = 0;
        normal = Vector3.zero;
        point = Vector3.zero;
    }
}

public class Wheel : MonoBehaviour
{
    private float deltaTime;
    public Rigidbody vehicleBody;

    [Header("Hit Detection - Inputs")]
    public LayerMask layerMask;
    [Header("Hit Detection - Outputs")]
    public bool isGrounded;
    
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

    public HitData hitData;
    private float _patchSize = 0.05f;
    
    public void UpdatePhysicsPre(float argDeltaTime)
    {
        hitData.Reset();
        deltaTime = argDeltaTime;

        var p = transform.position;
        Vector3[] points = new Vector3[5]
        {
            p,
            new Vector3(p.x - _patchSize, p.y, p.z + _patchSize),
            new Vector3(p.x + _patchSize, p.y, p.z + _patchSize),
            new Vector3(p.x - _patchSize, p.y, p.z - _patchSize),
            new Vector3(p.x + _patchSize, p.y, p.z - _patchSize)
        };

        foreach (var point in points)
        {
            if (Physics.Raycast(point, -transform.up, out var hit, restLength + wheelRadius, layerMask))
            {
                hitData.totalHit++;
                hitData.normal += hit.normal;
                hitData.point += hit.point;
                hitData.distance += hit.distance;
                
                Debug.DrawLine(point, hit.point, Color.green);
            }
            else
            {
                Debug.DrawRay(point, -transform.up * (restLength + wheelRadius), Color.red);
            }
        }

        if (hitData.totalHit == 0)
        {
            isGrounded = false;
            hitData.Reset();
            ResetValues();
            return;
        }

        isGrounded = true;
        
        hitData.normal /= hitData.totalHit;
        hitData.point /= hitData.totalHit;
        hitData.distance /= hitData.totalHit;
        currentLength = hitData.distance - wheelRadius;
        
        Debug.DrawRay(hitData.point, hitData.normal.normalized, Color.blue);
        
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
            CalculateLateralFriction();
            CalculateLongitudinalFriction();
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
        fZ = hitData.normal.normalized * suspensionForce; //Suspension force acts perpendicular to the contact patch

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
        linearVelocityLocal = transform.InverseTransformDirection(vehicleBody.GetPointVelocity(hitData.point));
        angularVelocityLocal = linearVelocityLocal / wheelRadius; // omega = v / r

        //Lateral and longitudinal directions of motion of the wheel
        longitudinalDir = Vector3.ProjectOnPlane(transform.forward, hitData.normal).normalized;
        lateralDir = Vector3.ProjectOnPlane(transform.right, hitData.normal).normalized;
    }

    float EvaluatePacejka(float k, float b, float c, float e)
    {
        float phi = (1 - e) * k + (e / b) * Mathf.Atan(b * k);
        return Mathf.Sin(c * Mathf.Atan(b * phi));
    }
    
    void CalculateLateralFriction()
    {
        slipAngle = Mathf.Atan2(-linearVelocityLocal.x, linearVelocityLocal.z); // slip angle
        muX = EvaluatePacejka(slipAngle, 10.0f, 1.9f, 0.97f);
    }

    void CalculateLongitudinalFriction()
    {
        //Calculate Torque Acting On Wheel
        float frictionTorque = muY * Mathf.Max(fZ.y, 0.0f) * wheelRadius;
        totalTorque = driveTorque - frictionTorque;

        //Integrate Angular Velocity
        float wheelAngularAcceleration = totalTorque / wheelInertia;
        wheelAngularVelocity += wheelAngularAcceleration * deltaTime;

        slipSpeed = wheelAngularVelocity - angularVelocityLocal.z;

        //Map Wheel Slip To Friction Curve
        muY = EvaluatePacejka(slipSpeed, 10.0f, 1.9f, 0.97f);
    }

    void ApplyFrictionForce()
    {
        fX = lateralDir * muX * Mathf.Max(fZ.y, 0.0f); //F_lat = u * N * -latDir
        fY = longitudinalDir * muY * Mathf.Max(fZ.y, 0.0f); // F_long = u * N * -longDir
        outputForce = (fX + fY);
        vehicleBody.AddForceAtPosition(outputForce, hitData.point); //Apply the friction force at the wheel's contact patch
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
