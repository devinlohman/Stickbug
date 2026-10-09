using System;
using Grasshopper.Kernel;

namespace Stickbug
{
    public class HelixEntrySettingsComponent : GH_Component
    {
        public HelixEntrySettingsComponent()
          : base("Helical Entry", "HelixE",
              "Creates entry settings for helical descents",
              "Stickbug", "Planar")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddNumberParameter("Helix Radius", "HR", "Helix radius", GH_ParamAccess.item, 10.0);
            pManager.AddNumberParameter("Helix Pitch", "HP", "Helix pitch in mm/rev", GH_ParamAccess.item, 3.0);
            pManager.AddNumberParameter("Helix Height", "HH", "Helix height", GH_ParamAccess.item, 10.0);
            pManager.AddBooleanParameter("Apply To All Descents", "ALL", "Apply helical entry to all detected descents", GH_ParamAccess.item, true);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Entry Options", "EO", "Helical entry settings", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            double HR = 0.0;
            double HP = 0.0;
            double HH = 0.0;
            bool ALL = true;

            if (!DA.GetData(0, ref HR)) return;
            if (!DA.GetData(1, ref HP)) return;
            if (!DA.GetData(2, ref HH)) return;
            if (!DA.GetData(3, ref ALL)) return;

            if (HR < 0.0) HR = 0.0;
            if (HP < 0.0) HP = 0.0;
            if (HH < 0.0) HH = 0.0;

            EntrySettings settings = new EntrySettings(HR, HP, ALL, HH);

            DA.SetData(0, settings);
        }

        public override GH_Exposure Exposure => GH_Exposure.secondary;

        protected override System.Drawing.Bitmap Icon => Properties.Resources.Helix;

        public override Guid ComponentGuid => new Guid("C27BAD0F-A237-4269-8F4E-5F2A7F92B96C");
    }

    public class EntrySettings
    {
        public double HR;
        public double HP;
        public double HH;
        public bool ALL;

        public EntrySettings(double radius, double pitch, bool applyAll, double helixHeight)
        {
            HR = radius;
            HP = pitch;
            HH = helixHeight;
            ALL = applyAll;
        }

        public override string ToString()
        {
            return
                "Helix Entry Enabled" +
                "\nHelix Radius: " + HR.ToString("0.###") + " mm" +
                "\nHelix Pitch: " + HP.ToString("0.###") + " mm/rev" +
                "\nApply To All Descents: " + ALL.ToString() +
                "\nHelix Height: " + HH.ToString("0.###") + " mm";
        }
    }
}