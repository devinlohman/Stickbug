using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Stickbug
{
    public class EntryExitPlanesComponent : GH_Component
    {
        public EntryExitPlanesComponent()
          : base("Safe Entry/Exit", "EntryExit",
              "Adds safe entry and exit planes to a plane list",
              "Stickbug", "Non-Planar")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddPlaneParameter("Planes", "P", "Input planes", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Mode", "MO", "0 = use local plane Z axes, 1 = world Zaxis", GH_ParamAccess.item, 0);
            pManager.AddNumberParameter("Entry Height", "ENH", "Entry offset distance", GH_ParamAccess.item, 10.0);
            pManager.AddNumberParameter("Exit Height", "EXH", "Exit offset distance", GH_ParamAccess.item, 10.0);
            pManager.AddBooleanParameter("Entry On", "ENO", "Add entry plane", GH_ParamAccess.item, true);
            pManager.AddBooleanParameter("Exit On", "EXO", "Add exit plane", GH_ParamAccess.item, true);
            pManager.AddBooleanParameter("Flip Up", "FLU", "Flip entry and exit directions", GH_ParamAccess.item, false);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddPlaneParameter("All Planes", "A", "Full ordered plane list including optional entry and exit planes", GH_ParamAccess.list);
            pManager.AddPlaneParameter("Entry Plane", "B", "Optional entry plane", GH_ParamAccess.item);
            pManager.AddPlaneParameter("Exit Plane", "C", "Optional exit plane", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Plane> P = new List<Plane>();
            int MO = 0;
            double ENH = 0.0;
            double EXH = 0.0;
            bool ENO = false;
            bool EXO = false;
            bool FLU = false;

            if (!DA.GetDataList(0, P)) return;
            if (!DA.GetData(1, ref MO)) return;
            if (!DA.GetData(2, ref ENH)) return;
            if (!DA.GetData(3, ref EXH)) return;
            if (!DA.GetData(4, ref ENO)) return;
            if (!DA.GetData(5, ref EXO)) return;
            if (!DA.GetData(6, ref FLU)) return;

            if (P == null || P.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Input plane list is empty.");
                return;
            }

            List<Plane> validPlanes = new List<Plane>();
            for (int i = 0; i < P.Count; i++)
            {
                Plane pl = P[i];
                if (pl.IsValid)
                    validPlanes.Add(pl);
            }

            if (validPlanes.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "No valid planes found.");
                return;
            }

            Plane first = validPlanes[0];
            Plane last = validPlanes[validPlanes.Count - 1];

            Vector3d entryDir;
            Vector3d exitDir;

            if (MO == 0)
            {
                entryDir = first.ZAxis;
                exitDir = last.ZAxis;
            }
            else
            {
                entryDir = Vector3d.ZAxis;
                exitDir = Vector3d.ZAxis;
            }

            if (FLU)
            {
                entryDir = -entryDir;
                exitDir = -exitDir;
            }

            if (!entryDir.Unitize())
                entryDir = FLU ? -Vector3d.ZAxis : Vector3d.ZAxis;

            if (!exitDir.Unitize())
                exitDir = FLU ? -Vector3d.ZAxis : Vector3d.ZAxis;

            Plane entryPlane = first;
            Plane exitPlane = last;

            bool hasEntry = false;
            bool hasExit = false;

            if (ENO)
            {
                entryPlane.Origin = first.Origin + entryDir * ENH;
                hasEntry = true;
            }

            if (EXO)
            {
                exitPlane.Origin = last.Origin + exitDir * EXH;
                hasExit = true;
            }

            List<Plane> result = new List<Plane>();

            if (hasEntry)
                result.Add(entryPlane);

            result.AddRange(validPlanes);

            if (hasExit)
                result.Add(exitPlane);

            DA.SetDataList(0, result);

            if (hasEntry)
                DA.SetData(1, entryPlane);

            if (hasExit)
                DA.SetData(2, exitPlane);
        }
        public override GH_Exposure Exposure => GH_Exposure.quinary;
        
        public override Guid ComponentGuid => new Guid("4FD90AC7-9215-4098-9729-7572AB595FD7");

        protected override System.Drawing.Bitmap Icon => Properties.Resources.SafeExitEnt;
    }
}