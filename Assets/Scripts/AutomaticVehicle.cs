using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AutomaticVehicle : MonoBehaviour
{
    float deltaTime;
    const int SUBSTEPS = 100; //(physics stable @ physics freq. * substeps = 5000+ Hz)
    float subDeltaTime;

    public Engine engine;
    public TorqueConverter torqueConverter;
    public Gearbox gearbox;
    public Differential differential;
    public Wheel[] wheels;
    public Steering[] steerings;
    public Visuals[] visuals;
    public EngineAudio engineAudio;

    [Header("Inputs")]
    public float throttleInput;
    public float throttleSensitivity;
    public float steeringInput;
    public float steeringSensitivity;
    public float starterInput;

    [Header("Dimensions")]
    public float wheelbase;
    public float rearTrackLength;
    public float turningRadius;

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
        steeringInput = Mathf.MoveTowards(steeringInput, Input.GetAxisRaw("Horizontal"), Time.deltaTime * steeringSensitivity);
        starterInput = System.Convert.ToSingle(Input.GetKey(KeyCode.K));
        if (Input.GetKeyDown(KeyCode.G))
        {
            StartCoroutine(gearbox.ShiftUp());
        }
        if (Input.GetKeyDown(KeyCode.B))
        {
            StartCoroutine(gearbox.ShiftDown());
        }
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
            engine.UpdatePhysics(subDeltaTime, throttleInput, starterInput, torqueConverter.outputTorque);
            torqueConverter.UpdatePhysics(engine.GetCurrentTorque(), engine.engineRPM, gearbox.currentGearRatio * engine.engineRPM);
            gearbox.UpdatePhysics();
            wheels[0].UpdatePhysicsDrivetrain(subDeltaTime, 0.0f);
            wheels[1].UpdatePhysicsDrivetrain(subDeltaTime, 0.0f);
            var drivenWheelTorque = differential.GetDownstreamTorque(gearbox.GetDownstreamTorque(torqueConverter.outputTorque));
            wheels[2].UpdatePhysicsDrivetrain(subDeltaTime, drivenWheelTorque.x);
            wheels[3].UpdatePhysicsDrivetrain(subDeltaTime, drivenWheelTorque.y);
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
        GUI.BeginGroup(new Rect(10, 10, 500, 500));
        GUILayout.Label("RPM: " + engine.engineRPM);
        GUILayout.Label("Throttle: " + throttleInput);
        GUILayout.Label("TC In Torque: " + torqueConverter.torqueIn);
        GUILayout.Label("TC Out Torque: " + torqueConverter.torqueOut);
        GUILayout.Label("TC Speed In: " + torqueConverter.speedIn);
        GUILayout.Label("TC Speed Out: " + torqueConverter.speedOut);
        GUILayout.Label("Gear: " + gearbox.indicator);
        GUILayout.Label("Gear Ratio: " + gearbox.currentGearRatio);
        GUILayout.Label("Speed: " + rb.linearVelocity.magnitude);
    }
}
