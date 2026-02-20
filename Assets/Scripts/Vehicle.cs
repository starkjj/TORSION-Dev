using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.TextCore.LowLevel;
using UnityEngine.UI;

public class Vehicle : MonoBehaviour
{
    float deltaTime;
    const int SUBSTEPS = 100; //(physics stable @ physics freq. * substeps = 5000+ Hz)
    float subDeltaTime;

    public Engine engine;
    public Clutch clutch;
    public Gearbox gearbox;
    public Differential differential;
    public Wheel[] wheels;
    public Steering[] steerings;
    public Visuals[] visuals;
    public EngineAudio engineAudio;

    [Header("Inputs")]
    public float throttleInput;
    public float throttleSensitivity;
    public float clutchInput;
    public float clutchSensitivity;
    public float steeringInput;
    public float steeringSensitivity;
    public float starterInput;
    public bool shiftUpInput;
    public bool shiftDownInput;

    [Header("Dimensions")]
    public float wheelbase;
    public float rearTrackLength;
    public float turningRadius;

    [Header("UI")] 
    public UIManager uiManager;
    
    private Rigidbody rb;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        
        for (int i = 0; i < wheels.Length; i++)
        {
            steerings[i].Initialize(wheelbase, rearTrackLength, turningRadius);
            visuals[i].Initialize(wheels[i], steerings[i]);
        }

        engine.Initialize();
        gearbox.Initialize();
        engineAudio.Initialize();
    }

    void Update()
    {
        //Update inputs
        throttleInput = Mathf.MoveTowards(throttleInput, Mathf.Max(Input.GetAxisRaw("Vertical"), 0.0f), Time.deltaTime * throttleSensitivity);
        clutchInput = Mathf.MoveTowards(clutchInput, System.Convert.ToSingle(Input.GetKey(KeyCode.LeftShift)), Time.deltaTime * clutchSensitivity);
        steeringInput = Mathf.MoveTowards(steeringInput, Input.GetAxisRaw("Horizontal"), Time.deltaTime * steeringSensitivity);
        starterInput = System.Convert.ToSingle(Input.GetKey(KeyCode.X));
        
        if (Input.GetKeyDown(KeyCode.E))
        {
            StartCoroutine(gearbox.ShiftUp());
        }
        if (Input.GetKeyDown(KeyCode.Q))
        {
            StartCoroutine(gearbox.ShiftDown());
        }

        if (engine.engineRPM >= 3000)
        {
            StartCoroutine(gearbox.ShiftUp());
        }

        if (engine.engineRPM <= engine.idleRPM * 1.2f)
        {
            if (gearbox.currentGear > 3)
            {
                StartCoroutine(gearbox.ShiftDown());
            } 
        }

        if (gearbox.shifting)
        {
            throttleInput = 0;
        }

        uiManager.SetVehicleProperties(MapRangeClamped(engine.engineRPM - 200, engine.idleRPM, engine.redlineRPM, 0, 1), gearbox.currentGear);
    }

    void FixedUpdate()
    {
        deltaTime = Time.fixedDeltaTime;
        subDeltaTime = deltaTime / (float)SUBSTEPS;
        
        //Pre-Drivetrain loop
        for (int i = 0; i < wheels.Length; i++)
        {
            wheels[i].UpdatePhysicsPre(deltaTime);
            steerings[i].UpdatePhysics(steeringInput);
        }

        //Drivetrain loop (RWD)
        for (int i = 0; i < SUBSTEPS; i++)
        {
            var differentialW = differential.GetUpstreamAngularVelocity(new Vector2(wheels[2].wheelAngularVelocity, wheels[3].wheelAngularVelocity));
            var gearboxW = gearbox.GetUpstreamAngularVelocity(differentialW);

            engine.UpdatePhysics(subDeltaTime, throttleInput, starterInput, clutch.clutchTorque);
            clutch.UpdatePhysics(clutchInput,gearbox.inGear,engine.angularVelocity,gearboxW);
            gearbox.UpdatePhysics();
            wheels[0].UpdatePhysicsDrivetrain(subDeltaTime, 0.0f);
            wheels[1].UpdatePhysicsDrivetrain(subDeltaTime, 0.0f);
            wheels[2].UpdatePhysicsDrivetrain(subDeltaTime, differential.GetDownstreamTorque(gearbox.GetDownstreamTorque(clutch.clutchTorque)).x);
            wheels[3].UpdatePhysicsDrivetrain(subDeltaTime, differential.GetDownstreamTorque(gearbox.GetDownstreamTorque(clutch.clutchTorque)).y);
        }

        //Post-Drivetrain loop
        for (int i = 0; i < wheels.Length; i++)
        {
            wheels[i].UpdatePhysicsPost();
        }
    }

    void LateUpdate()
    {
        //Update misc.
        for (int i = 0; i < visuals.Length; i++)
        {
            visuals[i].UpdateVisuals(Time.deltaTime);
        }
        engineAudio.UpdateAudio();
        
        
    }

    void OnGUI()
    {
        GUI.BeginGroup(new Rect(10, 10, 800, 400));
        GUILayout.Label("RPM: " + engine.engineRPM);
        GUILayout.Label("Throttle: " + throttleInput);
        GUILayout.Label("Clutch: " + clutchInput);
        GUILayout.Label("Gear: " + gearbox.indicator);
        GUILayout.Label("Gear Ratio: " + gearbox.currentGearRatio);
        GUILayout.Label("Speed: " + rb.linearVelocity.magnitude * 2.2369362921f);
        GUI.EndGroup();
    }
    
    float MapRangeClamped(float value, float inRangeA, float inRangeB, float outRangeA, float outRangeB) //Maps a value from one range to another
    {
        float result = Mathf.Lerp(outRangeA, outRangeB, Mathf.InverseLerp(inRangeA, inRangeB, value));
        return (result);
    }
}
