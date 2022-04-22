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

    public float WheelRadius = 1f;

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
}
