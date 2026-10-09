using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Parameters;
using Rhino;
using Rhino.Geometry;

namespace Stickbug
{
    public class SideOrientCurvePointComponent : GH_Component
    {
        private bool _useDegreesR = false;

        public SideOrientCurvePointComponent()
          : base("Aligned Orient (Curve/Point)", "AOrientCP",
              "Orient planes using a base curve and an orientation point. Planes are aligned with the base curve's direction, and the plane faces an orientation point",
              "Stickbug", "Non-Planar")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Base Curve", "BC", "Input base curve", GH_ParamAccess.item);
            pManager.AddPointParameter("Orientation Point", "OP", "Input orientation point", GH_ParamAccess.item);
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
            Point3d OP = Point3d.Unset;
            int N = 2;
            double R = 0.0;
            bool FP = false;

            if (!DA.GetData(0, ref BC)) return;
            if (!DA.GetData(1, ref OP)) return;
            if (!DA.GetData(2, ref N)) return;
            if (!DA.GetData(3, ref R)) return;
            if (_useDegreesR) R = RhinoMath.ToRadians(R);
            if (!DA.GetData(4, ref FP)) return;

            if (BC == null || !BC.IsValid || N < 2)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Input curve is null, invalid, or count is less than 2.");
                return;
            }

            int segCount = N - 1;

            double[] tBC = BC.DivideByCount(segCount, true);

            if (tBC == null || tBC.Length == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Could not divide the curve.");
                return;
            }

            int count = Math.Min(tBC.Length, N);
            List<Plane> planes = new List<Plane>(count);

            const double eps = 1e-12;

            for (int i = 0; i < count; i++)
            {
                double tb = tBC[i];

                Point3d pB = BC.PointAt(tb);

                Vector3d x = BC.TangentAt(tb);
                if (!x.Unitize())
                    x = Vector3d.XAxis;

                Vector3d v = OP - pB;
                Vector3d z = v - (Vector3d.Multiply(v, x) * x);

                if (z.SquareLength < eps)
                {
                    z = Vector3d.CrossProduct(x, Vector3d.ZAxis);
                    if (z.SquareLength < eps)
                        z = Vector3d.CrossProduct(x, Vector3d.XAxis);
                }

                if (!z.Unitize())
                    z = Vector3d.ZAxis;

                Vector3d y = Vector3d.CrossProduct(z, x);
                if (!y.Unitize())
                    y = Vector3d.YAxis;

                x = Vector3d.CrossProduct(y, z);
                if (!x.Unitize())
                    x = Vector3d.XAxis;

                Plane pl = new Plane(pB, x, y);
                pl.Rotate(R, pl.ZAxis, pl.Origin);

                if (FP)
                    pl.Flip();

                planes.Add(pl);
            }

            DA.SetDataList(0, planes);
        }

        public override GH_Exposure Exposure => GH_Exposure.primary;
        public override Guid ComponentGuid => new Guid("798F1D66-C8EB-445C-A5EB-57057D690C0A");

        protected override System.Drawing.Bitmap Icon => Properties.Resources.BlueCP;
    }
}