using Godot;
using System;

public class Car : RigidBody
{
    [Export]
    public NodePath ChaseCamNode;

    private Camera ChaseCam;

    private Vector3 ForwardVector { get => -Transform.basis.Column2; }

    public float EngineSpeed = 170f;

    // test value
    // todo: compute an aerodynamic drag coefficient based on car frontal area
    public float AeroDrag = 0.4257f;

    public float RollingResistance = 1f;

    private float LatestEngineInput = 0f;

    private float CamDistance = 7f;

    int ActiveColliders = 0;

    float RunningTime = 0f;

    Vector2 LastMousePosition = Vector2.Zero;

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
    }

    public override void _Input(InputEvent @event)
    {
        if(@event is InputEventMouseMotion)
        {
            if (ChaseCam != null)
            {
                Vector2 mousePos = (@event as InputEventMouseMotion).Relative;

                GD.Print($"{mousePos}");
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
        if (ActiveColliders != 0)
        {
            LinearVelocity += delta * (GetLongitudinalForce(LatestEngineInput) / Mass);
        }
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
