using System;
using System.IO;
using Grasshopper;
using Grasshopper.Kernel;
using Rhino;
using Rhino.UI;

namespace Stickbug
{
    public class WriteMODFileComponent : GH_Component
    {
        private bool _wasPressed = false;

        public WriteMODFileComponent()
          : base("Save as .MOD", "SaveMOD",
              "Write RAPID code to a .mod file",
              "Stickbug", "Utility")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("RAPID", "R", "RAPID code to write", GH_ParamAccess.item, string.Empty);
            pManager.AddBooleanParameter("Button", "B", "Write file", GH_ParamAccess.item, false);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            // no outputs
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            string textToWrite = string.Empty;
            bool pressed = false;

            if (!DA.GetData(0, ref textToWrite)) return;
            if (!DA.GetData(1, ref pressed)) return;

            if (!pressed)
            {
                _wasPressed = false;
                return;
            }

            if (_wasPressed)
                return;

            _wasPressed = true;

            var doc = OnPingDocument();
            if (doc == null) return;

            doc.ScheduleSolution(1, _ =>
            {
                RhinoApp.InvokeOnUiThread(new Action(() =>
                {
                    var dlg = new Eto.Forms.SaveFileDialog
                    {
                        Title = "Save .mod file",
                        FileName = "program.mod"
                    };

                    dlg.Filters.Add(new Eto.Forms.FileFilter("MOD files", ".mod"));

                    var result = dlg.ShowDialog(RhinoEtoApp.MainWindow);
                    if (result != Eto.Forms.DialogResult.Ok || string.IsNullOrWhiteSpace(dlg.FileName))
                        return;

                    string path = Path.ChangeExtension(dlg.FileName, ".mod");

                    File.WriteAllText(path, textToWrite);
                }));
            });
        }

        public override GH_Exposure Exposure => GH_Exposure.secondary;

        protected override System.Drawing.Bitmap Icon => Properties.Resources.Savemod;

        public override Guid ComponentGuid => new Guid("29A9A37E-3203-47BB-BD40-4BC60F191303");
    }
}