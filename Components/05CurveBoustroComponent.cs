using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Stickbug
{
    public class FlipAlternateCurvesComponent : GH_Component
    {
        public FlipAlternateCurvesComponent()
          : base("Boustrophedon (Curve)", "BoustroC",
              "Reverse the order of every other curve to create a boustrophedon path",
              "Stickbug", "Utility")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Curves", "C", "Input curves", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Flip Start", "FS", "Flip the start direction", GH_ParamAccess.item, false);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Curves", "A", "Curves with alternating directions", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var curves = new List<Curve>();
            bool flipStart = false;

            if (!DA.GetDataList(0, curves)) return;
            if (!DA.GetData(1, ref flipStart)) return;

            var output = new List<Curve>();

            for (int i = 0; i < curves.Count; i++)
            {
                Curve crv = curves[i];

                if (crv == null)
                {
                    output.Add(null);
                    continue;
                }

                Curve dup = crv.DuplicateCurve();

                bool flip = ((i % 2) == 0) == flipStart;

                if (flip)
                    dup.Reverse();

                output.Add(dup);
            }

            DA.SetDataList(0, output);
        }

        public override GH_Exposure Exposure => GH_Exposure.primary;

        protected override System.Drawing.Bitmap Icon => Properties.Resources.BoustroCurve;

        public override Guid ComponentGuid => new Guid("{B2447528-6694-4357-B0F2-980BB50203A6}");
    }
}