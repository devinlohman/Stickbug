using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino;
using Rhino.Geometry;

namespace Stickbug
{
    public class CurveDeviationComponent : GH_Component
    {
        public CurveDeviationComponent()
          : base("Log Data", "Crook",
              "Analyzes crookedness of log using centerline approximation curve",
              "Stickbug", "Log Data")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Centerline", "C", "Centerline of unforked log", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Samples", "N", "Number of samples", GH_ParamAccess.item, 10);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddLineParameter("Line", "L", "Reference line between curve endpoints", GH_ParamAccess.item);
            pManager.AddNumberParameter("Deviations", "D", "Distances from sampled curve points to the endpoint line segment", GH_ParamAccess.list);
            pManager.AddNumberParameter("Max Deviation", "M", "Maximum sampled deviation", GH_ParamAccess.item);
            pManager.AddPointParameter("Sampled Points", "P", "Sampled points on the curve", GH_ParamAccess.list);
            pManager.AddPointParameter("Max Point", "X", "Sample point with maximum deviation", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Curve C = null;
            int N = 2;

            if (!DA.GetData(0, ref C)) return;
            if (!DA.GetData(1, ref N)) return;

            if (C == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Input curve is null.");
                return;
            }

            if (N < 2) N = 2;

            Point3d start = C.PointAtStart;
            Point3d end = C.PointAtEnd;

            if (start.DistanceToSquared(end) <= RhinoMath.ZeroTolerance)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Curve start and end points are coincident.");
                return;
            }

            Line refLine = new Line(start, end);

            List<double> devs = new List<double>();
            List<Point3d> curvePts = new List<Point3d>();

            double maxDev = -1.0;
            Point3d maxPt = Point3d.Unset;

            Interval dom = C.Domain;

            for (int i = 0; i <= N; i++)
            {
                double t = dom.T0 + (dom.T1 - dom.T0) * ((double)i / N);
                Point3d cp = C.PointAt(t);

                Point3d lp = ClosestPointOnSegment(start, end, cp);
                double d = cp.DistanceTo(lp);

                curvePts.Add(cp);
                devs.Add(d);

                if (d > maxDev)
                {
                    maxDev = d;
                    maxPt = cp;
                }
            }

            DA.SetData(0, refLine);
            DA.SetDataList(1, devs);
            DA.SetData(2, maxDev);
            DA.SetDataList(3, curvePts);
            DA.SetData(4, maxPt);
        }

        private static Point3d ClosestPointOnSegment(Point3d a, Point3d b, Point3d p)
        {
            Vector3d ab = b - a;
            double ab2 = ab * ab;

            if (ab2 <= RhinoMath.ZeroTolerance)
                return a;

            double t = ((p - a) * ab) / ab2;

            if (t < 0.0) t = 0.0;
            if (t > 1.0) t = 1.0;

            return a + t * ab;
        }

        public override GH_Exposure Exposure => GH_Exposure.secondary;
        public override Guid ComponentGuid => new Guid("97D7D091-6D63-474C-AF81-750DD0C1635A");

        protected override System.Drawing.Bitmap Icon => Properties.Resources.LogData;
    }
}