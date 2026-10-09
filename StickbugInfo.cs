using System;
using System.Drawing;
using Grasshopper;
using Grasshopper.Kernel;

namespace Stickbug
{
    public class StickbugInfo : GH_AssemblyInfo
    {
        public override string Name => "Stickbug";

        //Return a 24x24 pixel bitmap to represent this GHA library.
        public override Bitmap Icon => Properties.Resources.StickbugFinal;

        //Return a short string describing the purpose of this GHA library.
        public override string Description => "Non-standard roundtimber evaluation and subtractive 6-axis robotic toolpathing. Developed as a teaching tool in the Decon/Recon Lab at the IIT College of Architecture in Chicago.";

        public override Guid Id => new Guid("ab0c6f6a-0a4c-4c07-8b0e-2e21d0e62dcb");

        //Return a string identifying you or your company.
        public override string AuthorName => "Devin Lohman";

        //Return a string representing your preferred contact details.
        public override string AuthorContact => "devinlohman@gmail.com";

        //Return a string representing the version.  This returns the same version as the assembly.
        public override string AssemblyVersion => GetType().Assembly.GetName().Version.ToString();
    }
}