using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

namespace Stickbug
{
    public class ReverseAlternateBranchesComponent : GH_Component
    {
        public ReverseAlternateBranchesComponent()
          : base("Boustrophedon (List)", "BoustroL",
              "Reverse the order of alternating lists to create a boustrophedon path from points or planes",
              "Stickbug", "Utility")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Tree", "Tree", "Ordered lists of points or planes to be reordered", GH_ParamAccess.tree);
            pManager.AddBooleanParameter("Flip Start", "FS", "Flip start direction", GH_ParamAccess.item, false);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Tree", "A", "Output tree with alternating branches reversed", GH_ParamAccess.tree);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_Structure<IGH_Goo> inputTree;
            bool flipStart = false;

            if (!DA.GetDataTree(0, out inputTree)) return;
            if (!DA.GetData(1, ref flipStart)) return;

            var result = new GH_Structure<IGH_Goo>();

            for (int i = 0; i < inputTree.Branches.Count; i++)
            {
                GH_Path path = inputTree.Paths[i];
                List<IGH_Goo> branch = new List<IGH_Goo>(inputTree.Branches[i]);

                bool reverse = flipStart ? (i % 2 == 0) : (i % 2 != 0);

                if (reverse)
                    branch.Reverse();

                result.AppendRange(branch, path);
            }

            DA.SetDataTree(0, result);
        }

        public override GH_Exposure Exposure => GH_Exposure.primary;

        protected override System.Drawing.Bitmap Icon => Properties.Resources.BoustroList;

        public override Guid ComponentGuid => new Guid("162BCFFE-9F89-4734-A34D-4393CB621DDA");
    }
}