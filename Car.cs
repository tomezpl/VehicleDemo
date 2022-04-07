using Godot;
using System;

public class Car : RigidBody
{
    private Vector3 ForwardVector { get => -Transform.basis.Column2; }

    public float EngineSpeed = 25f;

    public float AeroDrag = 1f;

    public float RollingResistance = 1f;

    private float LatestEngineInput = 0f;

    // Declare member variables here. Examples:
    // private int a = 2;
    // private string b = "text";

    // Called when the node enters the scene tree for the first time.
    public override void _Ready()
    {
        
    }

    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _Process(float delta)
    {
        LatestEngineInput = Input.GetAxis(InputBindings.Names.Decelerate, InputBindings.Names.Accelerate);
    }

    public override void _PhysicsProcess(float delta)
    {
        LinearVelocity += delta * (GetLongitudinalForce(LatestEngineInput) / Mass);
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
