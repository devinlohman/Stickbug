using System;
using Grasshopper.Kernel;

namespace Stickbug
{
    public class MMStoIPMComponent : GH_Component
    {
        public MMStoIPMComponent()
          : base("mm/s to IPM", "mmstoIPM",
              "Converts millimeters per second to inches per minute",
              "Stickbug", "Utility")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddNumberParameter("Millimeters Per Second", "M", "Feed rate in millimeters per second", GH_ParamAccess.item, 0.0);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Inches Per Minute", "IPM", "Feed rate in inches per minute", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            double mmPerSecond = 0.0;

            if (!DA.GetData(0, ref mmPerSecond)) return;

            double inchesPerMinute = (mmPerSecond * 60.0) / 25.4;

            DA.SetData(0, inchesPerMinute);
        }

        public override GH_Exposure Exposure => GH_Exposure.tertiary;

        protected override System.Drawing.Bitmap Icon => Properties.Resources.Ipm;

        public override Guid ComponentGuid => new Guid("028E0AC7-B8B6-4B85-A5AA-6027B7A3A763");
    }
}