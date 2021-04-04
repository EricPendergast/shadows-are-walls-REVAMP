using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

using Unity.Mathematics;
using Physics.Math;

using Rect = Physics.Math.Rect;

public class GeometryTests
{
    // A Test behaves as an ordinary method
    [Test]
    public void GeometryTestsSimplePasses()
    {
        // Use the Assert class to test conditions
    }

    [Test]
    public void ProjectionOverlap() {
        Assert.IsTrue(new Projection(0, 4).Overlaps(new Projection(2, 6)));
        Assert.IsTrue(new Projection(0, 4).Overlaps(new Projection(-2, 2)));
        Assert.IsTrue(new Projection(0, 4).Overlaps(new Projection(-2, 6)));
        Assert.IsTrue(new Projection(4, 0).Overlaps(new Projection(-1, 3)));

        Assert.IsFalse(new Projection(0, 4).Overlaps(new Projection(5, 6)));
        Assert.IsFalse(new Projection(0, 4).Overlaps(new Projection(-2, -1)));

        Assert.AreEqual(new Projection(0, 4).GetOverlap(new Projection(3, 8)), 1);
        Assert.AreEqual(new Projection(0, 4).GetOverlap(new Projection(2, 3)), 2);
        Assert.AreEqual(new Projection(0, 4).GetOverlap(new Projection(0, 1)), 1);
        Assert.AreEqual(new Projection(0, 4).GetOverlap(new Projection(1, 2)), 2);

        Assert.AreEqual(new Projection(0, 4).GetOverlap(new Projection(5, 6)), -1);
        Assert.AreEqual(new Projection(0, 4).GetOverlap(new Projection(-6, -1)), -1);

        Assert.AreEqual(new Projection(0, 4).GetOverlap(new Projection(-1, 10)), 5);
        Assert.AreEqual(new Projection(0, 4).GetOverlap(new Projection(-10, 5)), 5);
    }

    [Test]
    public void GetContacts1() {
        Assert.AreSame(Geometry.GetIntersectData(
                new Rect(new float2(0,0), new float2(2, 0), new float2(0, 2)),
                new Rect(new float2(0,2), new float2(1, 0), new float2(0, 1))
                ), 
                new Geometry.Manifold());
    }
}
