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

    [Export]
    public Vector2 GamepadFreeLookSensitivity = Vector2.One;

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

    [Export]
    public float PeakAcceleration = 20f;

    private Vector3 ChassisAngularVelocity = Vector3.Zero;

    [Export]
    public float WeightTransferDamping = 0.6f;

    [Export]
    public NodePath DebugTextNode;

    protected Label DebugText;

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
        DebugText = GetNodeOrNull<Label>(DebugTextNode ?? "");
    }

    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _Process(float delta)
    {
        RunningTime += delta;

        LatestEngineInput = Input.GetAxis(InputBindings.Names.Decelerate, InputBindings.Names.Accelerate);

        AnimateWeightTransfer(Acceleration.z, delta);

        if(ChaseCam != null)
        {
            //CameraFreeLook(Input.GetLastMouseSpeed());
        }

        CameraFreeLook(new Vector2(Input.GetActionStrength(InputBindings.Names.LookRight) - Input.GetActionStrength(InputBindings.Names.LookLeft), Input.GetActionStrength(InputBindings.Names.LookUp) - Input.GetActionStrength(InputBindings.Names.LookDown)) * GamepadFreeLookSensitivity);

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

        if(DebugText != null)
        {
            DebugText.Text = "";

            DebugText.Text += $"Chassis Angular Velocity: {ChassisAngularVelocity.ToString("f")}";
            DebugText.Text += $"\nChassis Rotation: {CarChassis.RotationDegrees.ToString("f")}";
            DebugText.Text += $"\nAcceleration: {Acceleration.ToString("f")}";
            DebugText.Text += $"\nVelocity: {LinearVelocity.ToString("f")}";
            DebugText.Text += $"\nRearWeight: {(GetRearWeight(Acceleration.Length()) - GetRearWeight(0f)).ToString("f")}";
            DebugText.Text += $"\nFrontWeight: {(GetFrontWeight(Acceleration.Length()) - GetFrontWeight(0f)).ToString("f")}";
            DebugText.Text += $"\nAcceleration %: {Acceleration.z / GetPeakAcceleration().z:f}";
        }
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

    public Vector3 GetPeakAcceleration()
    {
        return GetLongitudinalForce(1f, true) / Mass;
    }

    public override void _PhysicsProcess(float delta)
    {
        VelocityLastFrame = LinearVelocity;

        Vector3 longAccel = (GetLongitudinalForce(LatestEngineInput) / Mass);

        if (ActiveColliders != 0)
        {
            LinearVelocity += delta * longAccel;
        }

        Acceleration = (LinearVelocity - VelocityLastFrame) / delta;

        //AnimateWeightTransfer(Acceleration.z, delta);
    }

    protected void AnimateWeightTransfer(float acceleration, float delta)
    {
        // Weight force acting on each axle
        float frontWeight = GetFrontWeight(acceleration);
        float rearWeight = GetRearWeight(acceleration);

        // Calculate inertia at each axle.
        // I = mr^2
        float frontInertia = Mass * FrontAxle.LengthSquared();
        float rearInertia = Mass * RearAxle.LengthSquared();

        float sinTheta = 1f - Mathf.Sin(CarChassis.Rotation.x);
        float cosTheta = Mathf.Cos(CarChassis.Rotation.x);

        Vector3 peakAcceleration = GetPeakAcceleration();

        // Calculate torque from force and radius.
        // ignore sin theta since we'll be always applying the force perpendicularly, thus 1.
        // TODO: actually, maybe do sin theta of car chassis x axis?
        Vector3 frontTorque = Vector3.Right * FrontAxle.Length() * frontWeight * sinTheta;
        Vector3 rearTorque = Vector3.Right * RearAxle.Length() * rearWeight * sinTheta;

        // Calculate angular acceleration coming from each axle.
        Vector3 angularAcceleration = (rearTorque / rearInertia) - (frontTorque / frontInertia);

        float maxRadians = (Mathf.Pi / 128f) * Mathf.Min(1f, Mathf.Abs((Acceleration.z / peakAcceleration.z)));

        ChassisAngularVelocity -= angularAcceleration * delta;
        ChassisAngularVelocity -= ChassisAngularVelocity * delta * WeightTransferDamping;

        CarChassis.Rotation += ChassisAngularVelocity * delta;

        CarChassis.Rotation = new Vector3(Mathf.Clamp(CarChassis.Rotation.x, -maxRadians, maxRadians), CarChassis.Rotation.y, CarChassis.Rotation.z);

        //GD.Print($"FrontTorque: {frontTorque}, RearTorque: {rearTorque}, AngularAcceleration: {angularAcceleration}");

        //GD.Print($"FrontWeight: {frontWeight}, RearWeight: {rearWeight}, CarWeight: {GetCarWeight()}");
        //GD.Print($"Acceleration: {Acceleration.z}, Chassis rotation: {CarChassis.RotationDegrees}");
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
        float rideHeight = Mathf.Abs((FrontAxle - CarChassis.Translation).y) * Mathf.Abs(CarChassis.Transform.basis.z.Dot(Vector3.Forward));
        return GetAxleWeight(-acceleration, FrontAxle.Length(), rideHeight);
    }
    protected float GetRearWeight(float acceleration)
    {
        float rideHeight = Mathf.Abs((RearAxle - CarChassis.Translation).y) * Mathf.Abs(CarChassis.Transform.basis.z.Dot(Vector3.Forward));
        return GetAxleWeight(acceleration, RearAxle.Length(), rideHeight);
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
    protected Vector3 GetLongitudinalForce(float engineInput, bool ignoreResistance = false)
    {
        return ignoreResistance ? GetTractionForce(engineInput) : GetTractionForce(engineInput) + GetDragForce() + GetRollingResistance();
    }
}
