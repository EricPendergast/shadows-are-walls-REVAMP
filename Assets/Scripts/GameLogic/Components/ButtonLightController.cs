using Unity.Entities;


public struct ButtonLightController : IComponentData {
    public Entity controlledLight;
    public float pressedAngVel;
    public float unpressedAngVel;
    public float softness;
}
