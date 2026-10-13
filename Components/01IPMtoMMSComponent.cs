using System;
using Grasshopper.Kernel;

namespace Stickbug
{
    public class IPMtoMMSComponent : GH_Component
    {
        public IPMtoMMSComponent()
          : base("IPM to mm/s", "IPMtomms",
              "Converts inches per minute to millimeters per second",
              "Stickbug", "Utility")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddNumberParameter("Inches Per Minute", "IPM", "Feed rate in inches per minute", GH_ParamAccess.item, 0.0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Millimeters Per Second", "M", "Feed rate in millimeters per second", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            double inchesPerMinute = 0.0;

            if (!DA.GetData(0, ref inchesPerMinute)) return;

            double mmPerSecond = (inchesPerMinute * 25.4) / 60.0;

            DA.SetData(0, mmPerSecond);
        }

        public override GH_Exposure Exposure => GH_Exposure.tertiary;

        protected override System.Drawing.Bitmap Icon => Properties.Resources.Mms;

        public override Guid ComponentGuid => new Guid("6bb42f1a-4468-4598-ae1b-4abc4828a0dd");
    }
}