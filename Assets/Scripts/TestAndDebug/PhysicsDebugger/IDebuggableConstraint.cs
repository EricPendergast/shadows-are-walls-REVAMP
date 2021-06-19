using UnityEngine;

public interface IDebuggableConstraint {
    IDebuggableConstraint Clone();
    void DrawGizmos(DrawGizmosSettings settings);
    public struct Constants {
        public float beta;
        public float delta_slop;
        public float softness;
    }
    void SetConstants(Constants constants);
    IConstraint GetConstraint();
    [System.Serializable]
    public class DrawGizmosSettings {
        public Color color;
        public float drawRadius;
        public Color springStretchColor;
    }
}

