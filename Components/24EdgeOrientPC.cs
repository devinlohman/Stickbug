using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Parameters;
using Rhino;
using Rhino.Geometry;

namespace Stickbug
{
    public class EdgeOrientPointCurveComponent : GH_Component
    {
        private bool _useDegreesR = false;

        public EdgeOrientPointCurveComponent()
          : base("Edge Orient (Point/Curve)", "EOrientPC",
              "Orient planes using a base point and an orientation point. Planes are aligned with the base point, and the edge faces points along an orientation curve",
              "Stickbug", "Non-Planar")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddPointParameter("Base Point", "BP", "Input base point", GH_ParamAccess.item);
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
            Point3d BP = Point3d.Unset;
            Curve OC = null;
            int N = 2;
            double R = 0.0;
            bool FP = false;

            if (!DA.GetData(0, ref BP)) return;
            if (!DA.GetData(1, ref OC)) return;
            if (!DA.GetData(2, ref N)) return;
            if (!DA.GetData(3, ref R)) return;
            if (_useDegreesR) R = RhinoMath.ToRadians(R);
            if (!DA.GetData(4, ref FP)) return;

            if (OC == null || !OC.IsValid || N < 2)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Input curve is null, invalid, or count is less than 2.");
                return;
            }

            int segCount = N - 1;
            double[] tOC = OC.DivideByCount(segCount, true);

            if (tOC == null || tOC.Length == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Could not divide the curve.");
                return;
            }

            int count = Math.Min(tOC.Length, N);
            List<Plane> planes = new List<Plane>(count);

            const double eps = 1e-12;

            for (int i = 0; i < count; i++)
            {
                Point3d pO = OC.PointAt(tOC[i]);

                Vector3d v = pO - BP;
                Vector3d z = v;

                if (z.SquareLength < eps)
                {
                    Vector3d tan = OC.TangentAt(tOC[i]);
                    if (!tan.Unitize())
                        z = Vector3d.ZAxis;
                    else
                    {
                        z = Vector3d.CrossProduct(tan, Vector3d.ZAxis);
                        if (!z.Unitize())
                            z = Vector3d.YAxis;
                    }
                }

                if (!z.Unitize())
                    z = Vector3d.ZAxis;

                Vector3d x = Vector3d.CrossProduct(z, Vector3d.ZAxis);
                if (x.SquareLength < eps)
                    x = Vector3d.CrossProduct(z, Vector3d.XAxis);

                if (!x.Unitize())
                    x = Vector3d.XAxis;

                Vector3d y = Vector3d.CrossProduct(z, x);
                if (!y.Unitize())
                    y = Vector3d.YAxis;

                x = Vector3d.CrossProduct(y, z);
                if (!x.Unitize())
                    x = Vector3d.XAxis;

                Plane pl = new Plane(BP, x, y);
                pl.Rotate(R, pl.ZAxis, pl.Origin);

                if (FP)
                    pl.Flip();

                planes.Add(pl);
            }

            DA.SetDataList(0, planes);
        }

        public override GH_Exposure Exposure => GH_Exposure.secondary;
        public override Guid ComponentGuid => new Guid("33B85B1B-7027-4094-80FB-0764DAB54D09");

        protected override System.Drawing.Bitmap Icon => Properties.Resources.GreenPC;
    }
}