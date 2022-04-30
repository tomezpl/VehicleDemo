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

    protected Vector3 FrontAxleBaseRotation = Vector3.Zero;

    protected float FrontAxleSpan = 0f;

    private Camera ChaseCam;

    private Vector3 ForwardVector { get => -Transform.basis.Column2; }

    private Vector3 RightVector { get => Transform.basis.Column0; }
    private Vector3 UpVector { get => Transform.basis.Column1; }

    [Export]
    public Vector2 GamepadFreeLookSensitivity = Vector2.One;

    public float EngineSpeed = 130f;

    // test value
    // todo: compute an aerodynamic drag coefficient based on car frontal area
    public float AeroDrag = 0.4257f;

    public float RollingResistance = 1f;

    private float LatestEngineInput = 0f;

    public float LatestCorneringInput = 0f;

    [Export]
    public float MaxWheelYaw = Mathf.Pi / 8f;

    private float CamDistance = 7f;

    private int ActiveColliders = 0;

    // Tyre friction coefficient (mu).
    private float TyreFriction = 1f;

    private float RunningTime = 0f;

    private float WheelBase = 0f;

    private Vector2 LastMousePosition = Vector2.Zero;

    public Vector3 Acceleration = Vector3.Zero, VelocityLastFrame = Vector3.Zero;

    [Export]
    public float PeakAcceleration = 20f;

    private Vector3 ChassisAngularVelocity = Vector3.Zero;

    [Export]
    public float WeightTransferDamping = 0.6f;

    [Export]
    public NodePath DebugTextNode;

    protected Label DebugText;

    [Export]
    public float CorneringStiffness = 0.4f;

    public float WheelTurn = 0f;

    [Export]
    public float WheelTurnRate = 0.2f;

    [Export]
    public float WheelRecoverRate = 0.5f;

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

        FrontAxleBaseRotation = GetNode<Spatial>(FrontAxleNode).Rotation;
    }

    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _Process(float delta)
    {
        RunningTime += delta;

        LatestEngineInput = Input.GetAxis(InputBindings.Names.Decelerate, InputBindings.Names.Accelerate);
        LatestCorneringInput = Input.GetAxis(InputBindings.Names.TurnLeft, InputBindings.Names.TurnRight);
        if (Mathf.Abs(LatestCorneringInput) < 0.0001f)
        {
            WheelTurn = Mathf.Clamp(WheelTurn - WheelTurn * WheelRecoverRate * delta, -1f, 1f);
        }
        else
        {
            float maxTurnValue = Mathf.Sign(LatestCorneringInput) == -1f ? MaxWheelYaw : Mathf.Abs(LatestCorneringInput);
            float minTurnValue = Mathf.Sign(LatestCorneringInput) == 1f ? -MaxWheelYaw : -Mathf.Abs(LatestCorneringInput);
            WheelTurn = Mathf.Clamp(WheelTurn + WheelTurnRate * delta * Mathf.Sign(LatestCorneringInput), minTurnValue, maxTurnValue);
        }

        Vector3 localAcceleration = GetLocalAcceleration();
        AnimateWeightTransfer(localAcceleration.z, delta);

        if(ChaseCam != null)
        {
            //CameraFreeLook(Input.GetLastMouseSpeed());
        }

        CameraFreeLook(new Vector2(Input.GetActionStrength(InputBindings.Names.LookRight) - Input.GetActionStrength(InputBindings.Names.LookLeft), Input.GetActionStrength(InputBindings.Names.LookUp) - Input.GetActionStrength(InputBindings.Names.LookDown)) * GamepadFreeLookSensitivity);

        DebugDraw();
    }

    protected void DrawLine(Vector3 start, Vector3 end, Color color)
    {
        DebugGeometry.SetColor(color);
        DebugGeometry.AddVertex(start);
        DebugGeometry.SetColor(color);
        DebugGeometry.AddVertex(end);
    }

    protected void DebugDraw()
    {
        DebugGeometry.Clear();
        DebugGeometry.Begin(Mesh.PrimitiveType.Lines);

        // Draw rear axle
        //DrawLine(CarChassis.Translation, RearAxle, Colors.Red);

        // Draw front axle
        //DrawLine(CarChassis.Translation, FrontAxle, Colors.DarkRed);

        // Draw rear axle weight
        //DrawLine(RearAxle, RearAxle + Vector3.Up * GetRearWeight(Acceleration.Length()) * 0.4f, Colors.Cyan);

        // Draw front axle weight
        //DrawLine(FrontAxle, FrontAxle + Vector3.Up * GetRearWeight(Acceleration.Length()) * 0.4f, Colors.DarkCyan);

        (float lng, float lat) = GetVelocitySplit();
        DrawLine(Vector3.Zero, Vector3.Forward * lng, Colors.Blue);
        DrawLine(Vector3.Zero, Vector3.Right * lat, Colors.Red);

        //DrawLine(Vector3.Zero, ChassisAngularVelocity * 3f, Colors.Pink);

        (float front, float rear) alpha = GetSlipAngles();
        Vector3 rearLat = GetLateralForce(alpha.rear, GetRearWeight(GetLocalAcceleration().z));
        Vector3 frontLat = GetLateralForce(alpha.front, GetFrontWeight(GetLocalAcceleration().z));

        float deltaAngle = GetDeltaAngle();

        float direction = Mathf.Sign(GetVelocitySplit().lng);

        Vector3 torque = GetCorneringTorque(rearLat, frontLat, deltaAngle);
        torque *= direction;
        var torqueSplit = GetCorneringTorqueSplit(rearLat, frontLat, deltaAngle);
        DrawLine(Vector3.Forward * 0.3f, (Vector3.Forward * 0.3f) + torque, Colors.Purple);

        Vector3 netCorneringForce = -GetNetCorneringForce(rearLat, frontLat, deltaAngle);

        netCorneringForce *= direction;

        DrawLine(Vector3.Up * 0.2f, Vector3.Up * 0.2f + (Vector3.Right * netCorneringForce.y), Colors.Cornflower);

        Vector3 wheelOrigin = Vector3.Forward * 0.3f + Vector3.Right * 0.1f;
        //DrawLine(wheelOrigin, wheelOrigin + Vector3.Right * Mathf.Sin(deltaAngle), Colors.Cyan);

        DebugGeometry.End();

        if(DebugText != null)
        {
            DebugText.Text = "";

            //DebugText.Text += $"Chassis Angular Velocity: {ChassisAngularVelocity.ToString("f")}";
            //DebugText.Text += $"\nChassis Rotation: {CarChassis.RotationDegrees.ToString("f")}";
            //DebugText.Text += $"\nAcceleration: {Acceleration.ToString("f")}";
            DebugText.Text += $"\nVelocity: {LinearVelocity.ToString("f")}";
            //DebugText.Text += $"\nRearWeight: {(GetRearWeight(Acceleration.Length()) - GetRearWeight(0f)).ToString("f")}";
            //DebugText.Text += $"\nFrontWeight: {(GetFrontWeight(Acceleration.Length()) - GetFrontWeight(0f)).ToString("f")}";
            //DebugText.Text += $"\nAcceleration %: {Acceleration.z / GetPeakAcceleration().z:f}";
            DebugText.Text += $"\nfCornering: {netCorneringForce.ToString("f")}";
            DebugText.Text += $"\ntorque: {torque.ToString("f")}";
            DebugText.Text += $"\ntorque REAR: {torqueSplit.rear.ToString("f")}";
            DebugText.Text += $"\ntorque FRONT: {torqueSplit.front.ToString("f")}";
            DebugText.Text += $"\nfLateral, front: {frontLat.ToString("f")}";
            DebugText.Text += $"\nfLateral, rear: {rearLat.ToString("f")}";
            DebugText.Text += $"\nDelta Angle: {GetDeltaAngle():f}";
            DebugText.Text += $"\nSlip Angle: {GetSlipAngles().front:f}rad, {Mathf.Rad2Deg(GetSlipAngles().front):f}deg";
            DebugText.Text += $"\nTyre load: {GetTyreLoad():f}";
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

    /// <summary>
    /// Splits the velocity vector into longitudinal and lateral.
    /// </summary>
    /// <returns></returns>
    public (float lng, float lat) GetVelocitySplit()
    {
        float magnitude = LinearVelocity.Length();
        if(magnitude < 0.2f)
        {
            return (0f, 0f);
        }

        Vector3 normV = LinearVelocity.Normalized();
        return (-Transform.basis.z.Dot(normV) * magnitude, Transform.basis.x.Dot(normV) * magnitude);
    }

    public Vector3 GetTyreLoad()
    {
        return Vector3.Down * Weight / 4f;
    }

    public float SlipAngleCurve(float slipAngleDeg)
    {
        float absAngle = Mathf.Abs(slipAngleDeg);
        if(absAngle < 90f)
        {
            const float curveLoad = 5000f;
            const float peakLat = 5750f / curveLoad;
            const float minLat = 5400f / curveLoad;

            float lateralForce = Mathf.Lerp(minLat, peakLat, absAngle / 90f) * Mathf.Sign(slipAngleDeg);
            return lateralForce;
        }
        else
        {
            return 0f;
        }
    }

    public Vector3 GetLateralForce(float slipAngle, float tyreLoad)
    {
        float slipAngleDeg = Mathf.Rad2Deg(slipAngle);

        if (Mathf.Abs(slipAngleDeg) < 15f)
        {
            return UpVector * CorneringStiffness * slipAngle;
        }
        else
        {
            return UpVector * SlipAngleCurve(slipAngle) * tyreLoad;
        }
    }

    public float GetTurnRadius(float deltaAngle)
    {
        if (Mathf.Abs(deltaAngle) < 0.002f)
        {
            return 0f;
        }
        else
        {
            return WheelBase / Mathf.Sin(deltaAngle);
        }
    }

    public float GetTurnRate(float turnRadius)
    {
        if (turnRadius == 0f)
        {
            return 0f;
        }
        else
        {
            return Mathf.Abs(GetVelocitySplit().lat) / turnRadius;
        }
    }

    /// <summary>
    /// Calculates slip angle (alpha) for the front and rear wheels.
    /// </summary>
    /// <returns></returns>
    public (float front, float rear) GetSlipAngles()
    {
        (float lng, float lat) = GetVelocitySplit();

        float delta = GetDeltaAngle();

        float yawRate = GetTurnRate(GetTurnRadius(delta));

        float front = (lng == 0f ? 0f : Mathf.Atan((lat + yawRate * FrontAxle.Length()) / Mathf.Abs(lng))) - delta * Mathf.Sign(lng);

        float rear = lng == 0f ? 0f : Mathf.Atan((lat - yawRate * RearAxle.Length()) / Mathf.Abs(lng));

        return (front, rear);
    }

    public float GetDeltaAngle()
    {
        return WheelTurn * MaxWheelYaw;
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

        (float front, float rear) alpha = GetSlipAngles();
        Vector3 rearLat = GetLateralForce(alpha.rear, GetRearWeight(GetLocalAcceleration().z));
        Vector3 frontLat = GetLateralForce(alpha.front, GetFrontWeight(GetLocalAcceleration().z));

        float deltaAngle = GetDeltaAngle();
        Vector3 corneringForce = GetNetCorneringForce(rearLat, frontLat, deltaAngle);
        Vector3 torque = GetCorneringTorque(rearLat, frontLat, deltaAngle);

        float direction = Mathf.Sign(GetVelocitySplit().lng);

        corneringForce *= direction;
        //torque *= direction;

        //if (Mathf.Abs(GetVelocitySplit().lng) > 0.01f)
        {
            /*AddCentralForce(Transform.basis.x * (Mathf.Cos(GetDeltaAngle()) * frontLat.y * 2f * direction));

            AddTorque((rearLat.Normalized() * GetTyreLoad()) * RearAxle.Length() * direction);
            AddTorque((rearLat.Normalized() * GetTyreLoad()) * RearAxle.Length() * direction);
            AddTorque(frontLat.Normalized() * GetTyreLoad() * FrontAxle.Length() * Mathf.Cos(GetDeltaAngle()) * direction);
            AddTorque(frontLat.Normalized() * GetTyreLoad() * FrontAxle.Length() * Mathf.Cos(GetDeltaAngle()) * direction);*/

            AddCentralForce(RightVector * corneringForce.y);
            AddCentralForce(-ForwardVector * direction * corneringForce.y);
            AddTorque(torque);
        }
    }

    protected Vector3 GetLocalAcceleration()
    {
        return (RightVector * Acceleration) + (UpVector * Acceleration) + (ForwardVector * Acceleration) * -1f;
    }

    protected Vector3 GetNetCorneringForce(Vector3 rearLat, Vector3 frontLat, float deltaAngle)
    {
        Vector3 localAcceleration = GetLocalAcceleration();
        return (rearLat.Normalized() * Mathf.Abs(GetRearWeight(localAcceleration.z))) - (Mathf.Cos(deltaAngle) * frontLat.Normalized() * Mathf.Abs(GetFrontWeight(localAcceleration.z)));
    }

    protected Vector3 GetCorneringTorque(Vector3 rearLat, Vector3 frontLat, float deltaAngle)
    {
        var torque = GetCorneringTorqueSplit(rearLat, frontLat, deltaAngle);
        return torque.rear + torque.front;
    }
    protected (Vector3 rear, Vector3 front) GetCorneringTorqueSplit(Vector3 rearLat, Vector3 frontLat, float deltaAngle)
    {
        return ((-rearLat * RearAxle.Length()), (Mathf.Cos(deltaAngle) * frontLat * FrontAxle.Length()));
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

        float cosTheta = Mathf.Cos(CarChassis.Rotation.x);

        Vector3 peakAcceleration = GetPeakAcceleration();

        // Calculate torque from force and radius.
        // ignore sin theta since we'll be always applying the force perpendicularly, thus 1.
        // TODO: actually, maybe do sin theta of car chassis x axis?
        Vector3 frontTorque = Vector3.Right * FrontAxle.Length() * frontWeight * cosTheta;
        Vector3 rearTorque = Vector3.Right * RearAxle.Length() * rearWeight * cosTheta;

        // Calculate angular acceleration coming from each axle.
        Vector3 angularAcceleration = (rearTorque / rearInertia) - (frontTorque / frontInertia);

        float maxRadians = (Mathf.Pi / 64f) * Mathf.Min(1f, Mathf.Abs((Acceleration.z / peakAcceleration.z)));

        ChassisAngularVelocity -= angularAcceleration * delta;
        // TODO: Maybe check if the change since last frame was too drastic, to smooth out the velocity changes?
        ChassisAngularVelocity = new Vector3(Mathf.Clamp(ChassisAngularVelocity.x, -maxRadians, maxRadians), ChassisAngularVelocity.y, ChassisAngularVelocity.z);
        ChassisAngularVelocity -= ChassisAngularVelocity * WeightTransferDamping * delta;

        CarChassis.Rotation += ChassisAngularVelocity * delta;

        CarChassis.Rotation = new Vector3(Mathf.Clamp(CarChassis.Rotation.x, -maxRadians, maxRadians), CarChassis.Rotation.y, CarChassis.Rotation.z);
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
        // Scale the ride height by angle from resting position.
        float rideHeight = Mathf.Abs(((RearAxle + FrontAxle) / 2f).y - CarChassis.Translation.y);
        return GetAxleWeight(-acceleration, FrontAxle.Length(), rideHeight);
    }
    protected float GetRearWeight(float acceleration)
    {
        // Scale the ride height by angle from resting position.
        float rideHeight = Mathf.Abs(((RearAxle + FrontAxle) / 2f).y - CarChassis.Translation.y);
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
