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

    [Export]
    public float WheelRadius = 1f;

    [Export]
    public float MaxSuspensionCompression = 0f;

    [Export]
    public float MaxSuspensionDroop = 0f;

    private Quat BaseTurnRotation = Quat.Identity, BaseRollRotation = Quat.Identity;

    private Vector3 BaseOffset = Vector3.Zero;

    [Export]
    public Vector3 RotationAxis = Vector3.Right;

    [Export]
    public NodePath ParentAxleNode;

    public Spatial ParentAxle;

    private Vector3 LastFramePos = Vector3.Zero;

    [Export]
    public float WheelMass = 12000f;

    [Export]
    public NodePath DebugGeometryNode;

    protected ImmediateGeometry DebugGeometry;

    private Vector3 LastRaycastHit = Vector3.Zero;

    // Called when the node enters the scene tree for the first time.
    public override void _Ready()
    {
        ParentCar = GetNode<Car>(ParentCarNode);
        GD.Print($"AABB: {GetChild<MeshInstance>(0).GetAabb()}");

        MeshInstance wheelMesh = GetChildOrNull<MeshInstance>(0);
        if(wheelMesh != null)
        {
            AABB wheelAabb = wheelMesh.GetAabb();

            WheelRadius = Mathf.Max(Mathf.Abs(wheelAabb.Size.y), Mathf.Abs(wheelAabb.Size.z)) / 2f;
        }

        ParentAxle = GetNode<Spatial>(ParentAxleNode);

        // Store the predefined wheel local origin's Y coord as maximum spring length.
        //MaxSuspensionY = Translation.y - ParentAxle.Translation.y;
        //Translation -= Vector3.Up * MaxSuspensionY;

        BaseTurnRotation = new Quat(Rotation) * new Quat(Vector3.Up, FlippedYAxis ? 0f : -Mathf.Pi);
        BaseRollRotation = new Quat(Rotation);

        BaseOffset = Translation;

        LastFramePos = GlobalTransform.origin;

        DebugGeometry = GetNode<ImmediateGeometry>(DebugGeometryNode);
    }

    /// <summary>
    /// Utility method to set the Y coord, useful if the coordinate space is different.
    /// </summary>
    /// <param name="distance">Distance along the wheel spring.</param>
    public void SetSpringDistance(float distance)
    {
        distance -= WheelRadius;
        distance = Mathf.Max(WheelRadius, distance);
        Translation = GetParentSpatial().GlobalTransform.XformInv(GetGlobalMaxCompressionPoint()) + Vector3.Down * distance * Scale.y;
    }

    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _Process(float delta)
    {
        float carLinearVelocity = (LastFramePos - GlobalTransform.origin).Length() / delta;

        if(IsDriveWheel)
        {
            carLinearVelocity += ParentCar.GetLocalAcceleration().z * delta * 5f;
        }

        carLinearVelocity *= 3f;
        
        float angularVelocity = (carLinearVelocity / WheelRadius) * ParentCar.Transform.basis.z.Normalized().Dot(-ParentCar.LinearVelocity.Normalized());

        BaseRollRotation *= new Quat(Vector3.Right, angularVelocity * delta * (FlippedYAxis ? -1f : 1f));

        Quat newTurnRot = BaseTurnRotation;

        if (IsTurningWheel)
        {
            newTurnRot *= new Quat(Vector3.Up * ParentCar.WheelTurn * ParentCar.MaxWheelYaw * -1f);
        }

        Rotation = (newTurnRot * BaseRollRotation).Normalized().GetEuler();

        LastFramePos = GlobalTransform.origin;

        //DebugDraw.DrawLine3D(GetParentSpatial().GlobalTransform.Xform(BaseOffset), GetParentSpatial().GlobalTransform.Xform(BaseOffset + Vector3.Up * 30f), Colors.DarkSeaGreen);

        DebugGeometry.Begin(Mesh.PrimitiveType.Lines);
        DebugGeometry.DrawLine(GetGlobalMaxCompressionPoint(), GetGlobalMaxDroopPoint(), Colors.DarkSeaGreen);
        //DebugGeometry.DrawLine(origin + Vector3.Down * MaxSuspensionY * 0.5f, origin + Vector3.Up * MaxSuspensionY * 0.5f, Colors.LightYellow);
        DebugGeometry.DrawLine(LastRaycastHit - Vector3.Right * 0.3f, LastRaycastHit + Vector3.Right * 0.3f, Colors.Purple);
        DebugGeometry.DrawLine(GlobalTransform.origin, GlobalTransform.Xform(BaseOffset + Vector3.Down * WheelRadius * Scale.y), Colors.OrangeRed);
        DebugGeometry.End();
    }

    public Vector3 GetGlobalRelaxedPoint() => GetParentSpatial().GlobalTransform.Xform(BaseOffset);

    public Vector3 GetGlobalMaxCompressionPoint()
    {
        Spatial parentSpatial = GetParentSpatial(); 
        return parentSpatial.GlobalTransform.Xform(BaseOffset + Vector3.Up * MaxSuspensionCompression * parentSpatial.Scale.y);
    }
    public Vector3 GetGlobalMaxDroopPoint()
    {
        Spatial parentSpatial = GetParentSpatial();
        return parentSpatial.GlobalTransform.Xform(BaseOffset + Vector3.Down * MaxSuspensionDroop * parentSpatial.Scale.y);
    }

    public override void _PhysicsProcess(float delta)
    {
        Vector3 origin = GetGlobalMaxCompressionPoint();
        Godot.Collections.Dictionary raycast = GetWorld().DirectSpaceState.IntersectRay(origin, GetGlobalMaxDroopPoint());
        if(raycast != null && raycast.Contains("position"))
        {
            Vector3 raycastHit = (Vector3)raycast["position"];
            LastRaycastHit = raycastHit;
            SetSpringDistance((raycastHit - origin).Length() / Scale.y);
        }
        else
        {
            SetSpringDistance((MaxSuspensionCompression + MaxSuspensionDroop) / Scale.y);
        }
    }
}
