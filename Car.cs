using Godot;
using System;

public class Car : RigidBody
{
    [Export]
    public NodePath ChaseCamNode;

    [Export]
    public NodePath RearAxleNode;

    [Export]
    public NodePath FrontAxleNode;

    [Export]
    public NodePath CarChassisNode;

    private ImmediateGeometry DebugGeometry;

    protected Vector3 RearAxle, FrontAxle;
    protected Spatial CarChassis;

    private Camera ChaseCam;

    private Vector3 ForwardVector { get => -Transform.basis.Column2; }

    public float EngineSpeed = 330f;

    // test value
    // todo: compute an aerodynamic drag coefficient based on car frontal area
    public float AeroDrag = 0.4257f;

    public float RollingResistance = 1f;

    private float LatestEngineInput = 0f;

    private float CamDistance = 7f;

    private int ActiveColliders = 0;

    // Tyre friction coefficient (mu).
    private float TyreFriction = 1f;

    private float RunningTime = 0f;

    private float WheelBase = 0f;

    private Vector2 LastMousePosition = Vector2.Zero;

    private Vector3 Acceleration = Vector3.Zero, VelocityLastFrame = Vector3.Zero;

    private Vector3 ChassisAngularVelocity = Vector3.Zero;

    [Export]
    public float WeightTransferDamping = 0.6f;

    // Declare member variables here. Examples:
    // private int a = 2;
    // private string b = "text";

    // Called when the node enters the scene tree for the first time.
    public override void _Ready()
    {
        // rough approximation
        RollingResistance = AeroDrag * 30f;

        ContactMonitor = true;
        ContactsReported = 8;

        ChaseCam = GetNode<Camera>(ChaseCamNode);

        CamDistance = ChaseCam == null ? 7f : (Transform.origin - ChaseCam.Transform.origin).Length();

        Input.SetMouseMode(Input.MouseMode.Captured);

        CarChassis = GetNode<Spatial>(CarChassisNode);

        FrontAxle = GetNode<Spatial>(FrontAxleNode).Translation - CarChassis.Translation;
        RearAxle = GetNode<Spatial>(RearAxleNode).Translation - CarChassis.Translation;

        WheelBase = (FrontAxle - RearAxle).Length();

        DebugGeometry = GetNodeOrNull<ImmediateGeometry>("ImmediateGeometry");
    }

    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _Process(float delta)
    {
        RunningTime += delta;

        LatestEngineInput = Input.GetAxis(InputBindings.Names.Decelerate, InputBindings.Names.Accelerate);

        if(ChaseCam != null)
        {
            //CameraFreeLook(Input.GetLastMouseSpeed());
        }

        DebugDraw();
    }

    protected void DebugDraw()
    {
        DebugGeometry.Clear();
        DebugGeometry.Begin(Mesh.PrimitiveType.Lines);

        // Draw rear axle
        DebugGeometry.SetColor(Colors.Red);
        DebugGeometry.AddVertex(CarChassis.Translation);
        DebugGeometry.SetColor(Colors.Red);
        DebugGeometry.AddVertex(RearAxle);

        // Draw front axle
        DebugGeometry.SetColor(Colors.DarkRed);
        DebugGeometry.AddVertex(CarChassis.Translation);
        DebugGeometry.SetColor(Colors.DarkRed);
        DebugGeometry.AddVertex(FrontAxle);

        // Draw rear axle weight
        DebugGeometry.SetColor(Colors.Cyan);
        DebugGeometry.AddVertex(RearAxle);
        DebugGeometry.SetColor(Colors.Cyan);
        DebugGeometry.AddVertex(RearAxle + Vector3.Up * GetRearWeight(Acceleration.Length()) * 0.4f);

        // Draw front axle weight
        DebugGeometry.SetColor(Colors.DarkCyan);
        DebugGeometry.AddVertex(FrontAxle);
        DebugGeometry.SetColor(Colors.DarkCyan);
        DebugGeometry.AddVertex(FrontAxle + Vector3.Up * GetFrontWeight(Acceleration.Length()) * 0.4f);

        DebugGeometry.End();
    }

    public override void _Input(InputEvent @event)
    {
        if(@event is InputEventMouseMotion)
        {
            if (ChaseCam != null)
            {
                Vector2 mousePos = (@event as InputEventMouseMotion).Relative;

                CameraFreeLook(mousePos);
                LastMousePosition = mousePos;
            }
        }
    }

    public void CameraFreeLook(Vector2 cameraInput)
    {
        ChaseCam.Rotate(Vector3.Up, Mathf.Deg2Rad(-cameraInput.x) / 10f);
        ChaseCam.Rotate(ChaseCam.Transform.basis.x, Mathf.Deg2Rad(-cameraInput.y) / 10f);

        float yaw = ChaseCam.Rotation.y;
        float pitch = ChaseCam.Rotation.x;
        ChaseCam.Translation = new Vector3(Mathf.Sin(yaw) * Mathf.Cos(pitch), -Mathf.Sin(pitch), Mathf.Cos(yaw) * Mathf.Cos(pitch)) * CamDistance;
    }

    public override void _PhysicsProcess(float delta)
    {
        VelocityLastFrame = LinearVelocity;

        Vector3 longAccel = (GetLongitudinalForce(LatestEngineInput) / Mass);

        if (ActiveColliders != 0)
        {
            LinearVelocity += delta * longAccel;
        }

        AnimateWeightTransfer(Acceleration.z, delta);

        Acceleration = (LinearVelocity - VelocityLastFrame) / delta;
    }

    protected void AnimateWeightTransfer(float acceleration, float delta)
    {
        float frontWeight = GetFrontWeight(acceleration);
        float rearWeight = GetRearWeight(acceleration);
        float carWeight = GetCarWeight();

        float frontInertia = Mass * FrontAxle.LengthSquared();
        float rearInertia = Mass * RearAxle.LengthSquared();

        // ignore sin theta since we'll be always applying the force perpendicularly, thus 1.
        Vector3 frontTorque = Vector3.Right * (frontWeight - GetFrontWeight(0f)) * FrontAxle.Length();
        Vector3 rearTorque = Vector3.Right * (rearWeight - GetRearWeight(0f)) * RearAxle.Length();

        Vector3 angularAccelRear = rearTorque / rearInertia;
        Vector3 angularAccelFront = frontTorque / frontInertia;

        ChassisAngularVelocity += angularAccelRear + angularAccelFront;
        ChassisAngularVelocity -= ChassisAngularVelocity * WeightTransferDamping;

        const float maxRadians = Mathf.Pi / 16f;

        CarChassis.Rotation += ChassisAngularVelocity * delta;
        CarChassis.Rotation = new Vector3(Mathf.Clamp(CarChassis.Rotation.x, -maxRadians, maxRadians), CarChassis.Rotation.y, CarChassis.Rotation.z);

        float weightRatioRear = Mathf.InverseLerp(GetRearWeight(0), carWeight, rearWeight);
        float weightRatioFront = Mathf.InverseLerp(GetFrontWeight(0), carWeight, frontWeight);

        //GD.Print($"FrontWeight: {frontWeight}, RearWeight: {rearWeight}, CarWeight: {GetCarWeight()}");
        GD.Print($"Acceleration: {Acceleration.z}, RearAngularAcceleration: {angularAccelRear}, FrontAngularAcceleration: {angularAccelFront}, ChassisAngularVelocity: {ChassisAngularVelocity}");


        //CarChassis.Rotation = (new Quat(Vector3.Right, maxRadians * Mathf.Max(0f, -weightRatioRear)) * new Quat(-Vector3.Right, maxRadians * Mathf.Max(0f, -weightRatioFront))).GetEuler();
    }

    public void _on_RigidBody_body_entered(Node body)
    {
        GD.Print("Collided");
        ActiveColliders++;
    }

    public void _on_RigidBody_body_exited(Node body)
    {
        GD.Print("Exited collision");
        ActiveColliders = Mathf.Max(ActiveColliders - 1, 0);
    }

    // TODO: This only works for a stationary car without weight transfer in play.
    protected Vector3 GetMaxTraction()
    {
        return TyreFriction * Mass * Vector3.Down * GetCarWeight();
    }

    protected float GetCarWeight()
    {
        return Mass * GravityScale * 9.81f;
    }

    protected float GetFrontWeight(float acceleration)
    {
        return GetAxleWeight(-acceleration, FrontAxle.Length(), Mathf.Abs((FrontAxle - CarChassis.Translation).y));
    }
    protected float GetRearWeight(float acceleration)
    {
        return GetAxleWeight(acceleration, RearAxle.Length(), Mathf.Abs((RearAxle - CarChassis.Translation).y));
    }

    protected float GetAxleWeight(float acceleration, float axleDistance, float rideHeight)
    {
        return (axleDistance / WheelBase) * GetCarWeight() + (rideHeight / WheelBase) * Mass * acceleration;
    }

    protected Vector3 GetTractionForce(float engineInput)
    {
        return ForwardVector * engineInput * EngineSpeed;
    }

    /// <summary>
    /// Calculates an air resistance force based on <see cref="AeroDrag"/> constant and current velocity.
    /// </summary>
    /// <returns></returns>
    protected Vector3 GetDragForce()
    {
        return -AeroDrag * LinearVelocity * LinearVelocity.Length();
    }

    /// <summary>
    /// Calculates resistance force caused by friction between tire rubber and road surface, based on <see cref="RollingResistance"/> constant and current velocity.
    /// </summary>
    /// <returns></returns>
    protected Vector3 GetRollingResistance()
    {
        return -RollingResistance * LinearVelocity;
    }

    /// <summary>
    /// Longitudinal force exerted by the car wheels combined, including engine and resistance forces.
    /// </summary>
    /// <returns></returns>
    protected Vector3 GetLongitudinalForce(float engineInput)
    {
        return GetTractionForce(engineInput) + GetDragForce() + GetRollingResistance();
    }
}
