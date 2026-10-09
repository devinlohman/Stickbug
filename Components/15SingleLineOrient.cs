using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Parameters;
using Rhino;
using Rhino.Geometry;

namespace Stickbug
{
    public class CurveFramePlanesComponent : GH_Component
    {
        private bool _useDegreesA = false;
        private bool _useDegreesRP = false;

        public CurveFramePlanesComponent()
          : base("Single Line Orient", "SLOrient",
              "Orient aligned planes from a single line with a specified rotation",
              "Stickbug", "Non-Planar")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Curve", "BC", "Input base curve", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Count", "N", "Number of planes", GH_ParamAccess.item, 10);
            pManager.AddAngleParameter("Direction", "A", "Reference vector rotation angle around curve axis", GH_ParamAccess.item, 0.0);
            pManager.AddAngleParameter("Rotate", "RP", "Plane rotation angle around local Z axis", GH_ParamAccess.item, 0.0);
            pManager.AddBooleanParameter("Flip Planes", "FP", "Flip planes", GH_ParamAccess.item, false);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddPlaneParameter("Planes", "PL", "Generated planes", GH_ParamAccess.list);
        }

        protected override void BeforeSolveInstance()
        {
            _useDegreesA = false;
            _useDegreesRP = false;

            Param_Number aParam = Params.Input[2] as Param_Number;
            if (aParam != null)
                _useDegreesA = aParam.UseDegrees;

            Param_Number rpParam = Params.Input[3] as Param_Number;
            if (rpParam != null)
                _useDegreesRP = rpParam.UseDegrees;
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Curve BC = null;
            int N = 1;
            double A = 0.0;
            double RP = 0.0;
            bool FP = false;

            if (!DA.GetData(0, ref BC)) return;
            if (!DA.GetData(1, ref N)) return;
            if (!DA.GetData(2, ref A)) return;
            if (!DA.GetData(3, ref RP)) return;
            if (!DA.GetData(4, ref FP)) return;

            if (_useDegreesA)
                A = RhinoMath.ToRadians(A);

            if (_useDegreesRP)
                RP = RhinoMath.ToRadians(RP);

            if (BC == null || !BC.IsValid || N <= 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Input curve is null, invalid, or count is less than 1.");
                return;
            }

            double tol = RhinoDoc.ActiveDoc != null ? RhinoDoc.ActiveDoc.ModelAbsoluteTolerance : 0.01;
            double eps = Math.Max(10.0 * tol, 1e-6);

            Curve C = BC.DuplicateCurve();
            if (C == null || !C.IsValid)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Could not duplicate input curve.");
                return;
            }

            Point3d ps = C.PointAtStart;
            Point3d pe = C.PointAtEnd;

            if (ps.DistanceTo(pe) <= eps)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Curve start and end points are too close together.");
                return;
            }

            Vector3d ax = pe - ps;
            if (!ax.Unitize())
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Could not determine curve axis.");
                return;
            }

            BoundingBox bb = C.GetBoundingBox(true);
            double d = bb.Diagonal.Length * 0.05;
            if (d <= eps) d = 10.0;

            Vector3d refVec;
            if (Math.Abs(ax.Z) < 0.99)
                refVec = Vector3d.CrossProduct(ax, Vector3d.ZAxis);
            else
                refVec = Vector3d.CrossProduct(ax, Vector3d.XAxis);

            if (!refVec.Unitize())
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Could not determine initial reference vector.");
                return;
            }

            refVec.Rotate(A, ax);
            if (!refVec.Unitize())
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Could not rotate reference vector.");
                return;
            }

            Vector3d mv = refVec * d;

            Curve C0 = C.DuplicateCurve();
            Curve C1 = C.DuplicateCurve();

            C0.Transform(Transform.Translation(mv));
            C1.Transform(Transform.Translation(-mv));

            Brep[] loft = Brep.CreateFromLoft(
                new List<Curve> { C0, C1 },
                Point3d.Unset,
                Point3d.Unset,
                LoftType.Normal,
                false
            );

            if (loft == null || loft.Length == 0 || loft[0] == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Could not create loft helper surface.");
                return;
            }

            double[] ts;
            if (N == 1)
                ts = new double[] { C.Domain.Mid };
            else
                ts = C.DivideByCount(N - 1, true);

            if (ts == null || ts.Length != N)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Could not divide curve into requested plane count.");
                return;
            }

            List<Plane> planes = new List<Plane>();

            for (int i = 0; i < ts.Length; i++)
            {
                double t = ts[i];
                Point3d p = C.PointAt(t);

                Vector3d tan = C.TangentAt(t);
                if (!tan.Unitize())
                    tan = ax;

                Vector3d y = refVec;
                y -= tan * Vector3d.Multiply(y, tan);

                if (!y.Unitize())
                {
                    y = Vector3d.CrossProduct(ax, tan);
                    if (!y.Unitize())
                    {
                        if (Math.Abs(tan.Z) < 0.99)
                            y = Vector3d.CrossProduct(Vector3d.ZAxis, tan);
                        else
                            y = Vector3d.CrossProduct(Vector3d.XAxis, tan);

                        if (!y.Unitize())
                            y = Vector3d.YAxis;
                    }
                }

                Plane pl = new Plane(p, tan, y);
                pl.Rotate(RP, pl.ZAxis, pl.Origin);

                if (FP)
                    pl.Flip();

                planes.Add(pl);
            }

            DA.SetDataList(0, planes);
        }

        public override GH_Exposure Exposure => GH_Exposure.quarternary;
        public override Guid ComponentGuid => new Guid("8E4160EC-F0E3-47D7-81D1-418565782807");

        protected override System.Drawing.Bitmap Icon => Properties.Resources.SingleLine;
    }
}