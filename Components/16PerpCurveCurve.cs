using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Parameters;
using Rhino;
using Rhino.Geometry;

namespace Stickbug
{
    public class OffsetCurvePlanesComponent : GH_Component
    {
        private bool _useDegreesR = false;

        public OffsetCurvePlanesComponent()
          : base("Perp Orient (Curve/Curve)", "POrientCC",
              "Orient planes perpendicular to a base curve, with rotation around the curve determined by an orientation curve",
              "Stickbug", "Non-Planar")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Base Curve", "BC", "Input base curve", GH_ParamAccess.item);
            pManager.AddCurveParameter("Orientation Curve", "OC", "Input orientation curve", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Count", "N", "Number of planes", GH_ParamAccess.item, 10);
            pManager.AddAngleParameter("Rotate", "R", "Plane rotation around local Z axis", GH_ParamAccess.item, 0.0);
            pManager.AddBooleanParameter("Flip Planes", "FP", "Flip planes", GH_ParamAccess.item, false);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddPlaneParameter("Planes", "A", "Generated planes", GH_ParamAccess.list);
        }

        protected override void BeforeSolveInstance()
        {
            _useDegreesR = false;

            Param_Number rParam = Params.Input[3] as Param_Number;
            if (rParam != null)
                _useDegreesR = rParam.UseDegrees;
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Curve BC = null;
            Curve OC = null;
            int N = 2;
            double R = 0.0;
            bool FP = false;

            if (!DA.GetData(0, ref BC)) return;
            if (!DA.GetData(1, ref OC)) return;
            if (!DA.GetData(2, ref N)) return;
            if (!DA.GetData(3, ref R)) return;
            if (_useDegreesR) R = RhinoMath.ToRadians(R);
            if (!DA.GetData(4, ref FP)) return;

            if (BC == null || OC == null || !BC.IsValid || !OC.IsValid || N < 2)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Inputs are null, invalid, or count is less than 2.");
                return;
            }

            int segCount = N - 1;

            double[] tBC = BC.DivideByCount(segCount, true);
            double[] tOC = OC.DivideByCount(segCount, true);

            if (tBC == null || tOC == null || tBC.Length == 0 || tOC.Length == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Could not divide one or both curves.");
                return;
            }

            int count = Math.Min(Math.Min(tBC.Length, tOC.Length), N);
            List<Plane> planes = new List<Plane>(count);

            const double eps = 1e-12;

            for (int i = 0; i < count; i++)
            {
                double tb = tBC[i];
                double to = tOC[i];

                Point3d pB = BC.PointAt(tb);
                Point3d pO = OC.PointAt(to);

                Vector3d z = BC.TangentAt(tb);
                if (!z.Unitize())
                    z = Vector3d.ZAxis;

                Vector3d v = pO - pB;
                Vector3d x = v - (Vector3d.Multiply(v, z) * z);

                if (x.SquareLength < eps)
                {
                    x = Vector3d.CrossProduct(Vector3d.ZAxis, z);
                    if (x.SquareLength < eps)
                        x = Vector3d.CrossProduct(Vector3d.XAxis, z);
                }

                if (!x.Unitize())
                    x = Vector3d.XAxis;

                Vector3d y = Vector3d.CrossProduct(z, x);
                if (!y.Unitize())
                    y = Vector3d.YAxis;

                Plane pl = new Plane(pB, x, y);
                pl.Rotate(R, pl.ZAxis, pl.Origin);

                if (FP)
                    pl.Flip();

                planes.Add(pl);
            }

            DA.SetDataList(0, planes);
        }
        public override GH_Exposure Exposure => GH_Exposure.quarternary;

        public override Guid ComponentGuid => new Guid("B045DC95-4DB0-40D4-86BE-16350E13C1A9");

        protected override System.Drawing.Bitmap Icon => Properties.Resources.PurpleCC;
    }
}