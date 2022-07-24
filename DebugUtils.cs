using Godot;

public static class DebugUtils
{

    /// <summary>
    /// Utility method to draw a line from a starting point to an end point with a given color.
    /// </summary>
    /// <param name="start">Start point.</param>
    /// <param name="end">End point.</param>
    /// <param name="color">Color to draw the line with.</param>
    public static void DrawLine(this ImmediateGeometry debugGeometry, Vector3 start, Vector3 end, Color color)
    {
        debugGeometry.SetColor(color);
        debugGeometry.AddVertex(debugGeometry.GetParentSpatial().GlobalTransform.XformInv(start));
        debugGeometry.SetColor(color);
        debugGeometry.AddVertex(debugGeometry.GetParentSpatial().GlobalTransform.XformInv(end));
    }
}