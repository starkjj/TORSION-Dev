using UnityEngine;

public class TorqueConverter : MonoBehaviour
{
    public AnimationCurve capacityFactor;
    public AnimationCurve torqueRatio;

    public float speedIn = 0;
    public float speedOut = 0;
    public float torqueIn = 0;
    public float torqueOut = 0;
    public bool isReverseFlow = false;

    public float outputTorque = 0;

    public float GetTorqueRatio(float speedRatio)
    {
        // At stall (0.0 ratio), torque is multiplied (approx 2.0x)
        // At coupling (0.9+ ratio), torque ratio drops to 1.0x
        return Mathf.Max(1.0f, 2.0f - (speedRatio * 1.1f));
    }

    public float CalculateOutputTorque(float engineTorque, float engineRPM, float transmissionRPM)
    {
        // 1. Prevent division by zero
        if (engineRPM <= 0) return 0;

        // 2. Calculate Speed Ratio
        float speedRatio = transmissionRPM / engineRPM;
        speedRatio = Mathf.Clamp(speedRatio, 0f, 1f);

        // 3. Get Torque Multiplication Factor
        float torqueMult = GetTorqueRatio(speedRatio);

        // 4. Calculate final torque output to transmission
        return engineTorque * torqueMult;
    }
    
    public void UpdatePhysics(float engineTorque, float engineRPM, float transmissionRPM)
    {
        if (engineRPM.Equals(transmissionRPM))
        {
            // Direct mechanical connection
            outputTorque = engineTorque;
        }
        else
        {
            // Use the fluid logic from above
            outputTorque = CalculateOutputTorque(engineTorque, engineRPM, transmissionRPM);
        }
    }
}