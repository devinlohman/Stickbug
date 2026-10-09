using System;
using System.Text;
using Grasshopper.Kernel;

namespace Stickbug
{
    public class AddConfLOffComponent : GH_Component
    {
        public AddConfLOffComponent()
          : base("Add ConfL \\Off", "ConfLOff",
              "Inserts 'ConfL \\Off;' before MoveL lines when not already present",
              "Stickbug", "Utility")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("RAPID", "R", "Input RAPID code", GH_ParamAccess.item, string.Empty);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("Edited RAPID", "ER", "RAPID code with 'ConfL \\Off;' inserted before MoveL lines where needed", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            string input = string.Empty;
            if (!DA.GetData(0, ref input)) return;

            string src = input ?? string.Empty;
            string nl = src.Contains("\r\n") ? "\r\n" : "\n";
            string[] lines = src.Replace("\r\n", "\n").Split('\n');
            var sb = new StringBuilder(src.Length + lines.Length * 12);

            for (int i = 0; i < lines.Length; i++)
            {
                string line = lines[i];
                string trimmed = line.TrimStart();

                bool isMoveL = trimmed.StartsWith("MoveL", StringComparison.Ordinal);

                if (isMoveL && !trimmed.Contains("ConfL \\Off"))
                {
                    bool alreadyHasConfLBefore = false;

                    for (int j = i - 1; j >= 0; j--)
                    {
                        string prev = lines[j].TrimStart();

                        if (string.IsNullOrWhiteSpace(prev))
                            continue;

                        if (prev.StartsWith("!", StringComparison.Ordinal))
                            continue;

                        if (prev.StartsWith("ConfL \\Off", StringComparison.Ordinal))
                            alreadyHasConfLBefore = true;

                        break;
                    }

                    if (!alreadyHasConfLBefore)
                    {
                        int indentLen = line.Length - trimmed.Length;
                        string indent = indentLen > 0 ? line.Substring(0, indentLen) : string.Empty;

                        sb.Append(indent)
                          .Append("ConfL \\Off;")
                          .Append(nl);
                    }
                }

                sb.Append(line);

                if (i < lines.Length - 1)
                    sb.Append(nl);
            }

            DA.SetData(0, sb.ToString());
        }

        public override GH_Exposure Exposure => GH_Exposure.secondary;

        protected override System.Drawing.Bitmap Icon => Properties.Resources.ConfOff;

        public override Guid ComponentGuid => new Guid("206F90CE-4957-4E00-8B45-53D3551C0583");
    }
}