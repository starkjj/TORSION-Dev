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
    
    float GetSpeedRatio()
    {
        if (Mathf.Abs(speedIn) < 10e-9 || Mathf.Abs(speedOut) < 10e-9)
        {
            return 0;
        }

        return speedOut / speedIn;
    }

    public void UpdatePhysics(float inShaftSpeed, float outShaftSpeed)
    {
        isReverseFlow = false;
        
        // torque_in = input_torque;
        speedIn = inShaftSpeed;
        speedOut = outShaftSpeed;

        // might also calculate with:
        // speed_in = FMath::Sqrt(input_torque)*mK
        // but I guess I need the speeds to calculate the K factor?
    
        // compute the speed ratio
        var mR = GetSpeedRatio();

        // speed ratio is always in the 0-1 range,
        // correct just in case 

        if (mR > 1) {
            mR = 1 - (mR - 1);
            isReverseFlow = true;
        }

        // If in reverse, then stall
        if (mR < 0) { mR = 0; }

        // if spinning in a negative direction, set to zero and bail
        if (inShaftSpeed < 0) {
            torqueIn = 0;
            torqueOut = 0;
        }

        // compute actual capacity factor
        var mK = capacityFactor.Evaluate(mR);

        // compute actual torque factor
        var mT = torqueRatio.Evaluate(mR);

        // compute input torque (with minus sign because applied to input shaft)
        torqueIn = -Mathf.Pow((inShaftSpeed / mK), 2);

        // compute output torque (with negative b/c applied to output shaft, same direction of input shaft)
        if (isReverseFlow) {
            // in reverse flow situation, the convert is always in clutch mode (TR = 1)
            // so the torque cannot be increased
            torqueOut = -torqueIn;
        } else {
            torqueOut = -mT * torqueIn;
        }

        outputTorque = torqueOut;
    }
}