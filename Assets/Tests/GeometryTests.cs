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
        Compare((Geometry.Manifold)Geometry.GetIntersectData(
                new Rect(new float2(0,0), new float2(2, 0), new float2(0, 2)),
                new Rect(new float2(0,2), new float2(1, 0), new float2(0, 1))
                ), 
                new Geometry.Manifold{normal = new float2(0, 1), contact1 = new float2(-1, 1), contact2 = new float2(1, 1)});
    }

    private void Compare(Geometry.Manifold m1, Geometry.Manifold m2) {
        Compare(m1.normal, m2.normal);
        if (m1.contact2 == null || m2.contact2 == null) {
            Assert.IsNull(m2.contact2);
        } else {
            Assert.IsTrue(
                    (Equal(m1.contact1, m2.contact1) && Equal((float2)m1.contact2, (float2)m2.contact2)) ||
                    (Equal(m1.contact1, (float2)m2.contact2) && Equal((float2)m1.contact2, m2.contact1)));
        }
    }

    private bool Equal(float2 v1, float2 v2, float delta = .0001f) {
        return math.abs(v1.x - v2.x) < delta && math.abs(v1.y - v2.y) < delta;
    }

    private void Compare(float2 v1, float2 v2, float delta=.0001f) {
        Assert.AreEqual(v1.x, v2.x, delta);
        Assert.AreEqual(v1.y, v2.y, delta);
    }
}
