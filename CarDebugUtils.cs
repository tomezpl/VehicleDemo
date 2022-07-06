using Godot;

public partial class Car
{
    /// <summary>
    /// Utility method to draw a line from a starting point to an end point with a given color.
    /// </summary>
    /// <param name="start">Start point.</param>
    /// <param name="end">End point.</param>
    /// <param name="color">Color to draw the line with.</param>
    protected void DrawLine(Vector3 start, Vector3 end, Color color)
    {
        DebugGeometry.SetColor(color);
        DebugGeometry.AddVertex(start);
        DebugGeometry.SetColor(color);
        DebugGeometry.AddVertex(end);
    }

    /// <summary>
    /// Draws various debug info.
    /// </summary>
    /// <param name="velocity">Longitudinal and lateral velocity of the car (ie. local Z and X)</param>
    /// <param name="rearLat"></param>
    /// <param name="frontLat"></param>
    /// <param name="torque"></param>
    /// <param name="deltaAngle"></param>
    /// <param name="netCorneringForce"></param>
    protected void DebugDraw((float lng, float lat) velocity, Vector3 rearLat, Vector3 frontLat, Vector3 torque, float deltaAngle, Vector3 netCorneringForce)
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

        DrawLine(Vector3.Zero, Vector3.Forward * velocity.lng, Colors.Blue);
        DrawLine(Vector3.Zero, Vector3.Right * velocity.lat, Colors.Red);

        //DrawLine(Vector3.Zero, ChassisAngularVelocity * 3f, Colors.Pink);

        var torqueSplit = GetCorneringTorqueSplit(rearLat, frontLat, deltaAngle);
        DrawLine(Vector3.Forward * 0.3f, (Vector3.Forward * 0.3f) + torque, Colors.Purple);

        DrawLine(Vector3.Up * 0.2f, Vector3.Up * 0.2f + (Vector3.Right * netCorneringForce.y), Colors.Cornflower);

        Vector3 wheelOrigin = Vector3.Forward * 0.3f + Vector3.Right * 0.1f;
        //DrawLine(wheelOrigin, wheelOrigin + Vector3.Right * Mathf.Sin(deltaAngle), Colors.Cyan);

        DebugGeometry.End();

        if (DebugText != null)
        {
            DebugText.Text = "";

            //DebugText.Text += $"Chassis Angular Velocity: {ChassisAngularVelocity.ToString("f")}";
            //DebugText.Text += $"\nChassis Rotation: {CarChassis.RotationDegrees.ToString("f")}";
            //DebugText.Text += $"\nAcceleration: {Acceleration.ToString("f")}";
            DebugText.Text += $"\nVelocity: {new Vector2(velocity.lat, velocity.lng).ToString("f")}";
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
            DebugText.Text += $"\nSlip Angle FRONT: {GetSlipAngles().front:f}rad, {Mathf.Rad2Deg(GetSlipAngles().front):f}deg";
            DebugText.Text += $"\nSlip Angle REAR: {GetSlipAngles().rear:f}rad, {Mathf.Rad2Deg(GetSlipAngles().rear):f}deg";
            DebugText.Text += $"\nTyre load: {GetTyreLoad():f}";
        }
    }
}