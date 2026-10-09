using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Stickbug
{
    public class OrientScaleGeometryThreePointAutoComponent : GH_Component
    {
        public OrientScaleGeometryThreePointAutoComponent()
          : base("3pt Orient/Scale", "3ptOrientScale",
              "Orient and uniformly scale geometry from three source points to three target points",
              "Stickbug", "Log Data")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddGeometryParameter("Geometry", "G", "Input geometry", GH_ParamAccess.item);
            pManager.AddPointParameter("Source Points", "SP", "Three source points attached to the geometry", GH_ParamAccess.list);
            pManager.AddPointParameter("Target Points", "TP", "Three target points", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Scale Object", "SO", "If true, uniformly scale the geometry to match the target points", GH_ParamAccess.item, true);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddGeometryParameter("Geometry", "G", "Transformed geometry", GH_ParamAccess.item);
            pManager.AddPointParameter("Points", "P", "Transformed source points", GH_ParamAccess.list);
            pManager.AddTransformParameter("Transform", "X", "Best fit similarity transform", GH_ParamAccess.item);
            pManager.AddNumberParameter("Scale", "S", "Uniform scale factor", GH_ParamAccess.item);
            pManager.AddNumberParameter("Error", "E", "RMS error between transformed source points and target points", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GeometryBase G = null;
            List<Point3d> SP = new List<Point3d>();
            List<Point3d> TP = new List<Point3d>();
            bool SO = true;

            if (!DA.GetData(0, ref G)) return;
            if (!DA.GetDataList(1, SP)) return;
            if (!DA.GetDataList(2, TP)) return;
            if (!DA.GetData(3, ref SO)) return;

            if (G == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Input geometry is null.");
                return;
            }

            if (SP == null || TP == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Source Points and Target Points cannot be null.");
                return;
            }

            if (SP.Count != 3 || TP.Count != 3)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Source Points and Target Points must each contain exactly 3 points.");
                return;
            }

            if (!AreThreePointsUsable(SP))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Source points are degenerate or nearly collinear.");
                return;
            }

            if (!AreThreePointsUsable(TP))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Target points are degenerate or nearly collinear.");
                return;
            }

            int[][] perms = new int[][]
            {
                new int[] { 0, 1, 2 },
                new int[] { 0, 2, 1 },
                new int[] { 1, 0, 2 },
                new int[] { 1, 2, 0 },
                new int[] { 2, 0, 1 },
                new int[] { 2, 1, 0 }
            };

            bool found = false;
            double bestError = double.MaxValue;
            double bestScale = 1.0;
            Transform bestXform = Transform.Identity;
            List<Point3d> bestPts = null;

            for (int i = 0; i < perms.Length; i++)
            {
                int[] perm = perms[i];

                List<Point3d> orderedTargets = new List<Point3d>
                {
                    TP[perm[0]],
                    TP[perm[1]],
                    TP[perm[2]]
                };

                Transform xform;
                double scale;
                double error;
                List<Point3d> transformed;

                if (!TrySolveSimilarityTransform(SP, orderedTargets, SO, out xform, out scale, out transformed, out error))
                    continue;

                if (error < bestError)
                {
                    found = true;
                    bestError = error;
                    bestScale = scale;
                    bestXform = xform;
                    bestPts = transformed;
                }
            }

            if (!found)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Could not compute a valid transform.");
                return;
            }

            GeometryBase geometryOut = G.Duplicate();
            if (geometryOut == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Could not duplicate input geometry.");
                return;
            }

            if (!geometryOut.Transform(bestXform))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Could not transform input geometry.");
                return;
            }

            DA.SetData(0, geometryOut);
            DA.SetDataList(1, bestPts);
            DA.SetData(2, bestXform);
            DA.SetData(3, bestScale);
            DA.SetData(4, bestError);
        }

        private static bool TrySolveSimilarityTransform(
            List<Point3d> source,
            List<Point3d> target,
            bool scaleObject,
            out Transform xform,
            out double scale,
            out List<Point3d> transformed,
            out double error)
        {
            xform = Transform.Identity;
            scale = 1.0;
            transformed = null;
            error = double.MaxValue;

            Point3d sourceCentroid = GetCentroid(source);
            Point3d targetCentroid = GetCentroid(target);

            Plane sourcePlane;
            Plane targetPlane;

            if (!TryCreateFrame(source[0], source[1], source[2], sourceCentroid, out sourcePlane))
                return false;

            if (!TryCreateFrame(target[0], target[1], target[2], targetCentroid, out targetPlane))
                return false;

            double sourceScale = GetRmsRadius(source, sourceCentroid);
            double targetScale = GetRmsRadius(target, targetCentroid);

            if (sourceScale <= 1e-12 || targetScale <= 1e-12)
                return false;

            scale = scaleObject ? (targetScale / sourceScale) : 1.0;

            Plane sourcePlane0 = new Plane(Point3d.Origin, sourcePlane.XAxis, sourcePlane.YAxis);
            Plane targetPlane0 = new Plane(Point3d.Origin, targetPlane.XAxis, targetPlane.YAxis);

            Transform moveToOrigin = Transform.Translation(-sourceCentroid.X, -sourceCentroid.Y, -sourceCentroid.Z);
            Transform uniformScale = Transform.Scale(Point3d.Origin, scale);
            Transform rotate = Transform.PlaneToPlane(sourcePlane0, targetPlane0);
            Transform moveToTarget = Transform.Translation(targetCentroid.X, targetCentroid.Y, targetCentroid.Z);

            xform = moveToTarget * rotate * uniformScale * moveToOrigin;

            transformed = new List<Point3d>(source.Count);
            for (int i = 0; i < source.Count; i++)
            {
                Point3d p = source[i];
                p.Transform(xform);
                transformed.Add(p);
            }

            error = GetRmsError(transformed, target);
            return true;
        }

        private static bool AreThreePointsUsable(List<Point3d> pts)
        {
            if (pts == null || pts.Count != 3)
                return false;

            Vector3d a = pts[1] - pts[0];
            Vector3d b = pts[2] - pts[0];
            Vector3d n = Vector3d.CrossProduct(a, b);

            return a.Length > 1e-12 && b.Length > 1e-12 && n.Length > 1e-12;
        }

        private static Point3d GetCentroid(List<Point3d> pts)
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;

            for (int i = 0; i < pts.Count; i++)
            {
                x += pts[i].X;
                y += pts[i].Y;
                z += pts[i].Z;
            }

            double n = pts.Count;
            return new Point3d(x / n, y / n, z / n);
        }

        private static bool TryCreateFrame(Point3d p1, Point3d p2, Point3d p3, Point3d origin, out Plane plane)
        {
            plane = Plane.Unset;

            Vector3d x = p2 - p1;
            if (!x.Unitize())
                return false;

            Vector3d v = p3 - p1;
            Vector3d z = Vector3d.CrossProduct(x, v);
            if (!z.Unitize())
                return false;

            Vector3d y = Vector3d.CrossProduct(z, x);
            if (!y.Unitize())
                return false;

            plane = new Plane(origin, x, y);
            return plane.IsValid;
        }

        private static double GetRmsRadius(List<Point3d> pts, Point3d centroid)
        {
            double sum = 0.0;

            for (int i = 0; i < pts.Count; i++)
                sum += pts[i].DistanceToSquared(centroid);

            return Math.Sqrt(sum / pts.Count);
        }

        private static double GetRmsError(List<Point3d> a, List<Point3d> b)
        {
            int count = Math.Min(a.Count, b.Count);
            double sum = 0.0;

            for (int i = 0; i < count; i++)
                sum += a[i].DistanceToSquared(b[i]);

            return Math.Sqrt(sum / count);
        }

        public override GH_Exposure Exposure => GH_Exposure.tertiary;
        public override Guid ComponentGuid => new Guid("0261A6E1-3E79-45FF-B866-471A43B55F3D");

        protected override System.Drawing.Bitmap Icon => Properties.Resources.Threepoint;
    }
}