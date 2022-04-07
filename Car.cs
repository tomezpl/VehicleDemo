using Godot;
using System;

public class Car : RigidBody
{
    private Vector3 ForwardVector { get => -Transform.basis.Column2; }

    public float EngineSpeed = 3f;

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
        float engine = Input.GetAxis(InputBindings.Names.Decelerate, InputBindings.Names.Accelerate);
        GD.Print($"Engine input: {engine}");
        AddCentralForce(ForwardVector * engine * EngineSpeed);
    }
}
