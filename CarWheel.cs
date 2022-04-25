using Godot;
using System;

public class CarWheel : Spatial
{
    [Export]
    public NodePath ParentCarNode;

    protected Car ParentCar;

    [Export]
    public bool FlippedYAxis = false;

    [Export]
    public bool IsDriveWheel = false;

    [Export]
    public bool IsTurningWheel = false;

    public float WheelRadius = 1f;

    public float MaxSuspensionY = 0f;

    private Vector3 BaseRotation = Vector3.Zero;

    private float BaseOffset = 0f;

    [Export]
    public Vector3 RotationAxis = Vector3.Right;

    [Export]
    public NodePath ParentAxleNode;

    public Spatial ParentAxle;

    // Declare member variables here. Examples:
    // private int a = 2;
    // private string b = "text";

    // Called when the node enters the scene tree for the first time.
    public override void _Ready()
    {
        ParentCar = GetNode<Car>(ParentCarNode);
        GD.Print($"AABB: {GetChild<MeshInstance>(0).GetAabb()}");

        MeshInstance wheelMesh = GetChildOrNull<MeshInstance>(0);
        if(wheelMesh != null)
        {
            AABB wheelAabb = wheelMesh.GetAabb();
            WheelRadius = Mathf.Max(Mathf.Abs(wheelAabb.Size.y), Mathf.Abs(wheelAabb.Size.z));
        }

        ParentAxle = GetNode<Spatial>(ParentAxleNode);

        // Store the predefined wheel local origin's Y coord as maximum spring length.
        MaxSuspensionY = Translation.y - ParentAxle.Translation.y;
        Translation -= Vector3.Up * MaxSuspensionY;

        BaseRotation = Rotation;

        BaseOffset = Translation.y;
    }

    /// <summary>
    /// Utility method to set the Y coord, useful if the coordinate space is different.
    /// </summary>
    /// <param name="distance">Distance along the wheel spring.</param>
    public void SetSpringDistance(float distance)
    {
        Translation = new Vector3(distance, Translation.y, Translation.z);
    }

    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _Process(float delta)
    {
        float carLinearVelocity = ParentCar.LinearVelocity.Length();

        if(IsDriveWheel)
        {
            carLinearVelocity += ParentCar.Acceleration.z * delta * 5f;
        }
        
        float angularVelocity = (carLinearVelocity / WheelRadius) * ParentCar.Transform.basis.z.Normalized().Dot(-ParentCar.LinearVelocity.Normalized());
        GlobalRotate(GlobalTransform.basis.x.Normalized(), angularVelocity * delta * (FlippedYAxis ? -1f : 1f));
    }

    public override void _PhysicsProcess(float delta)
    {
        Transform axle = GetParent<Spatial>().GlobalTransform;
        Godot.Collections.Dictionary raycast = GetWorld().DirectSpaceState.IntersectRay(axle.origin, axle.origin + axle.basis.z * -MaxSuspensionY);
        if(raycast != null && raycast.Contains("position"))
        {
            Vector3 raycastHit = (Vector3)raycast["position"];
            //SetSpringDistance((raycastHit - axle.origin).Length());
        }

        if (IsTurningWheel)
        {
            Rotation = BaseRotation + Vector3.Up * ParentCar.LatestCorneringInput * ParentCar.MaxWheelYaw * -1f;
        }
    }
}
