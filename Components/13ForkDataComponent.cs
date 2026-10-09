using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino;
using Rhino.DocObjects;
using Rhino.Geometry;

namespace Stickbug
{
    public class ForkAnalysisComponent : GH_Component
    {
        public ForkAnalysisComponent()
          : base("Fork Data", "ForkData",
              "Analyze forked log centerlines for angle and offset information",
              "Stickbug", "Log Data")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Centerlines", "C", "Fork centerline curves", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Branch Spread Angle", "BSA", "Angle between the two branch directions (degrees)", GH_ParamAccess.item);
            pManager.AddNumberParameter("Fork Offset Angle", "FOA", "Angle between trunk and branch bisector (degrees)", GH_ParamAccess.item);
            pManager.AddCurveParameter("Trunk Curve", "TC", "Detected trunk curve", GH_ParamAccess.item);
            pManager.AddCurveParameter("Branch Curves", "BC", "Detected branch curves", GH_ParamAccess.list);
            pManager.AddLineParameter("Fork Midline", "FM", "Line from fork to midpoint of branches", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<object> C = new List<object>();
            if (!DA.GetDataList(0, C)) return;

            double BSA = 0;
            double FOA = 0;
            Curve TC = null;
            List<Curve> BC = null;
            Line FM = Line.Unset;

            if (C == null || C.Count < 3)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Need exactly 3 input curves.");
                return;
            }

            List<Curve> curves = new List<Curve>();

            for (int i = 0; i < C.Count; i++)
            {
                Curve crv;
                if (TryCoerceCurve(C[i], out crv) && crv != null)
                    curves.Add(crv);
            }

            if (curves.Count != 3)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Could not resolve exactly 3 valid curves.");
                return;
            }

            double tol = RhinoDoc.ActiveDoc != null
              ? RhinoDoc.ActiveDoc.ModelAbsoluteTolerance
              : 0.001;

            Point3d fork;
            if (!TryGetSharedPoint(curves[0], curves[1], curves[2], tol, out fork))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Could not determine shared fork point.");
                return;
            }

            Vector3d[] vecs = new Vector3d[3];
            Point3d[] ends = new Point3d[3];

            for (int i = 0; i < 3; i++)
            {
                if (!TryGetOutwardVectorAndEnd(curves[i], fork, tol, out vecs[i], out ends[i]))
                    return;

                if (!vecs[i].Unitize())
                    return;
            }

            double s01 = Vector3d.Multiply(vecs[0], vecs[1]);
            double s02 = Vector3d.Multiply(vecs[0], vecs[2]);
            double s12 = Vector3d.Multiply(vecs[1], vecs[2]);

            double score0 = s01 + s02;
            double score1 = s01 + s12;
            double score2 = s02 + s12;

            int trunkIndex = 0;
            double minScore = score0;
            if (score1 < minScore) { minScore = score1; trunkIndex = 1; }
            if (score2 < minScore) { minScore = score2; trunkIndex = 2; }

            int b1, b2;
            if (trunkIndex == 0) { b1 = 1; b2 = 2; }
            else if (trunkIndex == 1) { b1 = 0; b2 = 2; }
            else { b1 = 0; b2 = 1; }

            Vector3d v1 = vecs[b1];
            Vector3d v2 = vecs[b2];

            Point3d trunkEnd = ends[trunkIndex];
            Point3d e1 = ends[b1];
            Point3d e2 = ends[b2];

            double spreadRad = Vector3d.VectorAngle(v1, v2);
            double spreadDeg = RhinoMath.ToDegrees(spreadRad);

            double len1 = fork.DistanceTo(e1);
            double len2 = fork.DistanceTo(e2);

            Point3d longEnd, shortEndExtended;

            if (len1 >= len2)
            {
                longEnd = e1;
                shortEndExtended = fork + (v2 * len1);
            }
            else
            {
                longEnd = e2;
                shortEndExtended = fork + (v1 * len2);
            }

            Point3d mid = new Point3d(
              0.5 * (longEnd.X + shortEndExtended.X),
              0.5 * (longEnd.Y + shortEndExtended.Y),
              0.5 * (longEnd.Z + shortEndExtended.Z)
            );

            Line centerLine = new Line(fork, mid);

            Vector3d vc = mid - fork;
            if (vc.IsTiny(tol)) return;
            if (!vc.Unitize()) return;

            Vector3d trunkLineDir = trunkEnd - fork;
            if (trunkLineDir.IsTiny(tol)) return;
            if (!trunkLineDir.Unitize()) return;

            double kRad = Vector3d.VectorAngle(trunkLineDir, vc);
            double kDeg = RhinoMath.ToDegrees(kRad);
            if (kDeg > 90.0) kDeg = 180.0 - kDeg;

            BSA = spreadDeg;
            FOA = kDeg;
            TC = curves[trunkIndex];
            BC = new List<Curve>() { curves[b1], curves[b2] };
            FM = centerLine;

            DA.SetData(0, BSA);
            DA.SetData(1, FOA);
            DA.SetData(2, TC);
            DA.SetDataList(3, BC);
            DA.SetData(4, FM);
        }

        private bool TryCoerceCurve(object obj, out Curve crv)
        {
            crv = null;
            if (obj == null) return false;

            if (obj is Curve) { crv = obj as Curve; return true; }
            if (obj is GH_Curve) { crv = (obj as GH_Curve).Value; return true; }

            if (obj is Guid && RhinoDoc.ActiveDoc != null)
            {
                var ro = RhinoDoc.ActiveDoc.Objects.FindId((Guid)obj);
                if (ro != null) crv = ro.Geometry as Curve;
                return crv != null;
            }

            if (obj is IGH_Goo goo)
            {
                Curve temp;
                if (goo.CastTo<Curve>(out temp))
                {
                    crv = temp;
                    return true;
                }
            }

            return false;
        }

        private bool TryGetSharedPoint(Curve c0, Curve c1, Curve c2, double tol, out Point3d p)
        {
            Point3d[] e0 = { c0.PointAtStart, c0.PointAtEnd };
            Point3d[] e1 = { c1.PointAtStart, c1.PointAtEnd };
            Point3d[] e2 = { c2.PointAtStart, c2.PointAtEnd };

            foreach (Point3d test in e0)
            {
                bool hit1 = (test.DistanceTo(e1[0]) <= tol) || (test.DistanceTo(e1[1]) <= tol);
                bool hit2 = (test.DistanceTo(e2[0]) <= tol) || (test.DistanceTo(e2[1]) <= tol);
                if (hit1 && hit2)
                {
                    p = test;
                    return true;
                }
            }

            p = Point3d.Unset;
            return false;
        }

        private bool TryGetOutwardVectorAndEnd(Curve c, Point3d fork, double tol, out Vector3d v, out Point3d farEnd)
        {
            v = Vector3d.Unset;
            farEnd = Point3d.Unset;

            Point3d s = c.PointAtStart;
            Point3d e = c.PointAtEnd;

            bool atStart = s.DistanceTo(fork) <= tol;
            bool atEnd = e.DistanceTo(fork) <= tol;

            if (atStart && !atEnd) farEnd = e;
            else if (atEnd && !atStart) farEnd = s;
            else farEnd = (s.DistanceToSquared(fork) > e.DistanceToSquared(fork)) ? s : e;

            v = farEnd - fork;
            return !v.IsTiny(tol);
        }

        public override GH_Exposure Exposure => GH_Exposure.secondary;
        public override Guid ComponentGuid => new Guid("244C4A2A-3C0C-4F6A-ABE6-6CB2EABEB9B2");

        protected override System.Drawing.Bitmap Icon => Properties.Resources.ForkData;
    }
}