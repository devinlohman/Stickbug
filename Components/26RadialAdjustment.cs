using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Parameters;
using Rhino;
using Rhino.Geometry;

namespace Stickbug
{
    public class RadialAdjustmentComponent : GH_Component
    {
        private bool _useDegreesA = false;

        public RadialAdjustmentComponent()
          : base("Radial TCP Adjustment", "RadAdjust",
              "Adjust planes with reference to a radial end effector in order to cut with a different point on the blade",
              "Stickbug", "Non-Planar")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddPlaneParameter("Planes", "P", "Input planes", GH_ParamAccess.list);
            pManager.AddNumberParameter("Radius", "R", "Blade radius", GH_ParamAccess.item, 100.0);
            pManager.AddAngleParameter("Angle", "A", "Adjustment angle", GH_ParamAccess.item, 0.0);
            pManager.AddBooleanParameter("Flip Up", "FU", "Flip the blade orientation", GH_ParamAccess.item, false);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddPlaneParameter("Planes", "P", "Adjusted planes", GH_ParamAccess.list);
        }

        protected override void BeforeSolveInstance()
        {
            _useDegreesA = false;

            Param_Number aParam = Params.Input[2] as Param_Number;
            if (aParam != null)
                _useDegreesA = aParam.UseDegrees;
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Plane> P = new List<Plane>();
            double R = 0.0;
            double A = 0.0;
            bool FU = false;

            if (!DA.GetDataList(0, P)) return;
            if (!DA.GetData(1, ref R)) return;
            if (!DA.GetData(2, ref A)) return;
            if (!DA.GetData(3, ref FU)) return;

            if (_useDegreesA)
                A = RhinoMath.ToRadians(A);

            if (P == null || P.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Input plane list is empty.");
                return;
            }

            if (R <= 0.0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Radius must be greater than zero.");
                return;
            }

            List<Plane> adjusted = new List<Plane>(P.Count);

            for (int i = 0; i < P.Count; i++)
            {
                Plane pl = P[i];

                if (!pl.IsValid)
                    continue;

                Vector3d up = FU ? -pl.ZAxis : pl.ZAxis;

                Point3d center = pl.Origin + up * R;
                Transform rot = Transform.Rotation(A, pl.YAxis, center);

                Plane newPlane = pl;
                newPlane.Transform(rot);

                adjusted.Add(newPlane);
            }

            DA.SetDataList(0, adjusted);
        }

        public override GH_Exposure Exposure => GH_Exposure.quinary;

        public override Guid ComponentGuid => new Guid("6AEDC5ED-FEA8-4F81-8973-CC274AA8ECB0");

        protected override System.Drawing.Bitmap Icon => Properties.Resources.radialadjust;
    }
}