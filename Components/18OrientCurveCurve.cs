using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Parameters;
using Rhino;
using Rhino.Geometry;

namespace Stickbug
{
    public class FaceOrientCurveCurveComponent : GH_Component
    {
        private bool _useDegreesR = false;

        public FaceOrientCurveCurveComponent()
          : base("Face Orient (Curve/Curve)", "FOrientCC",
              "Orient planes using a base curve and an orientation curve",
              "Stickbug", "Non-Planar")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Base Curve", "BC", "Input base curve", GH_ParamAccess.item);
            pManager.AddCurveParameter("Orientation Curve", "OC", "Input orientation curve", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Count", "N", "Number of planes", GH_ParamAccess.item, 10);
            pManager.AddBooleanParameter("Lock Rotation", "LR", "Keep the plane X axis as consistent as possible along the sequence", GH_ParamAccess.item, false);
            pManager.AddAngleParameter("Rotate", "R", "Plane rotation around local Z axis", GH_ParamAccess.item, 0.0);
            pManager.AddBooleanParameter("Flip Planes", "FP", "Flip planes", GH_ParamAccess.item, false);
        }

        protected override void BeforeSolveInstance()
        {
            _useDegreesR = false;

            Param_Number rParam = Params.Input[4] as Param_Number;
            if (rParam != null)
                _useDegreesR = rParam.UseDegrees;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddPlaneParameter("Planes", "A", "Generated planes", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Curve BC = null;
            Curve OC = null;
            int N = 2;
            bool LR = false;
            double R = 0.0;
            bool FP = false;

            if (!DA.GetData(0, ref BC)) return;
            if (!DA.GetData(1, ref OC)) return;
            if (!DA.GetData(2, ref N)) return;
            if (!DA.GetData(3, ref LR)) return;
            if (!DA.GetData(4, ref R)) return;
            if (_useDegreesR) R = RhinoMath.ToRadians(R);
            if (!DA.GetData(5, ref FP)) return;

            if (BC == null || OC == null || !BC.IsValid || !OC.IsValid || N < 2)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Input curves are null, invalid, or count is less than 2.");
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

            Vector3d refX = Vector3d.Unset;
            bool refSet = false;

            for (int i = 0; i < count; i++)
            {
                Point3d pB = BC.PointAt(tBC[i]);
                Point3d pO = OC.PointAt(tOC[i]);

                Vector3d z = pO - pB;

                if (!z.Unitize())
                {
                    Vector3d tan = BC.TangentAt(tBC[i]);
                    if (!tan.Unitize())
                    {
                        z = Vector3d.ZAxis;
                    }
                    else
                    {
                        z = Vector3d.CrossProduct(tan, Vector3d.ZAxis);
                        if (!z.Unitize())
                            z = Vector3d.YAxis;
                    }
                }

                Vector3d x;
                Vector3d y;

                if (!LR)
                {
                    x = Vector3d.CrossProduct(Vector3d.ZAxis, z);
                    if (x.SquareLength < 1e-12)
                        x = Vector3d.CrossProduct(Vector3d.XAxis, z);

                    if (!x.Unitize())
                        x = Vector3d.XAxis;

                    y = Vector3d.CrossProduct(z, x);
                    if (!y.Unitize())
                        y = Vector3d.YAxis;

                    Plane pl = new Plane(pB, x, y);
                    pl.Rotate(R, pl.ZAxis, pl.Origin);

                    if (FP)
                        pl.Flip();

                    planes.Add(pl);
                }
                else
                {
                    if (!refSet)
                    {
                        x = Vector3d.CrossProduct(Vector3d.ZAxis, z);
                        if (x.SquareLength < 1e-12)
                            x = Vector3d.CrossProduct(Vector3d.XAxis, z);

                        if (!x.Unitize())
                            x = Vector3d.XAxis;

                        y = Vector3d.CrossProduct(z, x);
                        if (!y.Unitize())
                            y = Vector3d.YAxis;

                        Plane first = new Plane(pB, x, y);

                        refX = first.XAxis;
                        refSet = true;

                        first.Rotate(R, first.ZAxis, first.Origin);

                        if (FP)
                            first.Flip();

                        planes.Add(first);
                    }
                    else
                    {
                        x = refX - (Vector3d.Multiply(refX, z) * z);

                        if (x.SquareLength < 1e-12)
                        {
                            x = Vector3d.CrossProduct(Vector3d.ZAxis, z);
                            if (x.SquareLength < 1e-12)
                                x = Vector3d.CrossProduct(Vector3d.XAxis, z);
                        }

                        if (!x.Unitize())
                            x = Vector3d.XAxis;

                        y = Vector3d.CrossProduct(z, x);
                        if (!y.Unitize())
                            y = Vector3d.YAxis;

                        Plane pl = new Plane(pB, x, y);
                        pl.Rotate(R, pl.ZAxis, pl.Origin);

                        if (FP)
                            pl.Flip();

                        planes.Add(pl);
                    }
                }
            }

            DA.SetDataList(0, planes);
        }

        public override GH_Exposure Exposure => GH_Exposure.tertiary;
        public override Guid ComponentGuid => new Guid("311F8851-6C09-44D7-A195-9A4B4F15B09C");

        protected override System.Drawing.Bitmap Icon => Properties.Resources.OrangeCC;
    }
}