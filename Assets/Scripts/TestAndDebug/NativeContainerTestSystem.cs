using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

using Physics.Math;
using UnityEngine;

using Rect = Physics.Math.Rect;

[DisableAutoCreation]
public class NativeContainerTestSystem : SystemBase {
    protected override void OnUpdate() {
        NativeList<int> list = new NativeList<int>(0, Allocator.TempJob);
        
        Job.WithBurst().WithCode(() => {
            for (int i = 0; i < 1000; i++) {
                list.Add(i);
            }
        }).Schedule();

        Dependency.Complete();

        int sum = 0;
        int checkSum = 0;
        for (int i = 0; i < 1000; i++) {
            sum += list[i];
            checkSum += i;
        }
        Debug.Log("Result of NativeContainerTestSystem: " + (sum == checkSum));
        Dependency = list.Dispose(Dependency);
    }
}
