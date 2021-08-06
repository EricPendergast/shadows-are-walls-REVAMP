using Unity.Entities;
using Unity.Mathematics;
using UnityEngine.InputSystem;
using UnityEngine;

[UpdateInGroup(typeof(PostContactGenerationGroup))]
[UpdateAfter(typeof(ButtonSystem))]
public class ButtonLightControllerSystem : SystemBase {
    protected override void OnUpdate() {

        var output = World.GetOrCreateSystem<ConstraintGatherSystem>().GetOneWayOneDOFConstraintsInput();
        var masses = GetComponentDataFromEntity<Mass>();

        float dt = Time.DeltaTime;

        Entities.ForEach((Entity buttonEntity, in Button button, in ButtonLightController controller) => {
            float targetAngVel = button.pressed ? controller.pressedAngVel : controller.unpressedAngVel;

            output.Add(
                new OneWayOneDOFConstraint(
                    new TargetAngularVelocityManifold {
                        e = controller.controlledLight,
                        id = buttonEntity.GetHashCode() ^ 117956690,
                        softness = controller.softness,
                        targetAngVel = targetAngVel
                    },
                    masses,
                    dt
                )
            );
        }).Run();
    }
}
