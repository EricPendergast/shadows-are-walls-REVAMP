using Unity.Entities;

[GenerateAuthoringComponent]
public struct PlayerSettings : IComponentData {
    public float groundMoveSoftness;
    public float groundMoveSpeed;

    public float airMoveSoftness;
    public float airMoveSpeed;

    public float rotationCorrectionSoftness;
    public float rotationCorrectionSpeed;

    public float minJumpDotProd;
    public float jumpSpeed;
    public float jumpSoftness;

    public float swapDetectorBorder;
}
