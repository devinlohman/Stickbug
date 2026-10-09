using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino;
using Rhino.Geometry;

namespace Stickbug
{ 
    public class MeshForkCenterlinesComponent : GH_Component
    {
        public MeshForkCenterlinesComponent()
          : base("Fork Evaluation", "ForkEval",
              "Evaluate a forked log mesh",
              "Stickbug", "Log Data")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input mesh", GH_ParamAccess.item);
            pManager.AddPointParameter("Trunk Point", "TP", "Trunk endpoint pith", GH_ParamAccess.item);
            pManager.AddPointParameter("Branch Point 1", "BP1", "First branch endpoint pith", GH_ParamAccess.item);
            pManager.AddPointParameter("Branch Point 2", "BP2", "Second branch endpoint pith", GH_ParamAccess.item);
            pManager.AddPointParameter("Fork Guide Point", "FGP", "Guide point near the fork region", GH_ParamAccess.item);
            pManager.AddNumberParameter("Step", "STP", "Contour step distance", GH_ParamAccess.item, 100.0);
            pManager.AddIntegerParameter("Smooth Passes", "SP", "Number of smoothing passes", GH_ParamAccess.item, 0);
            pManager.AddNumberParameter("Move Fork Point", "MFP", "Adjustment distance from detected fork point", GH_ParamAccess.item, 0.0);
            pManager.AddIntegerParameter("End Cull Values", "ECV", "Number of contours to cull from each end", GH_ParamAccess.item, 0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Contours", "CON", "Utilized contour curves", GH_ParamAccess.list);
            pManager.AddCurveParameter("Centerlines", "CNTRS", "Estimated trunk and branch centerline curves", GH_ParamAccess.list);
            pManager.AddPointParameter("Fork Point", "FKP", "Estimated fork point", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh M = null;
            Point3d TP = Point3d.Unset;
            Point3d BP1 = Point3d.Unset;
            Point3d BP2 = Point3d.Unset;
            Point3d FGP = Point3d.Unset;
            double STP = 25.0;
            int SP = 0;
            double MFP = 0.0;
            int ECV = 0;

            if (!DA.GetData(0, ref M)) return;
            if (!DA.GetData(1, ref TP)) return;
            if (!DA.GetData(2, ref BP1)) return;
            if (!DA.GetData(3, ref BP2)) return;
            if (!DA.GetData(4, ref FGP)) return;
            if (!DA.GetData(5, ref STP)) return;
            if (!DA.GetData(6, ref SP)) return;
            if (!DA.GetData(7, ref MFP)) return;
            if (!DA.GetData(8, ref ECV)) return;

            List<Curve> CON = null;
            List<Curve> CNTRS = null;
            Point3d FKP = Point3d.Unset;

            if (M == null || !M.IsValid)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Input mesh is null or invalid.");
                return;
            }

            Point3d tipA = BP1;
            Point3d tipB = BP2;

            if (STP <= 0) STP = 25.0;
            if (SP < 0) SP = 0;
            if (MFP < 0) MFP = 0;
            if (ECV < 0) ECV = 0;

            double tol = 0.01;
            if (RhinoDoc.ActiveDoc != null)
                tol = RhinoDoc.ActiveDoc.ModelAbsoluteTolerance;

            double maxDist = Math.Max(
                TP.DistanceTo(tipA),
                TP.DistanceTo(tipB));

            double searchWindow = Math.Max(STP * 4.0, maxDist / 8.0);

            Point3d branchMid = new Point3d(
                (tipA.X + tipB.X) / 2.0,
                (tipA.Y + tipB.Y) / 2.0,
                (tipA.Z + tipB.Z) / 2.0);

            Vector3d axis = branchMid - TP;
            if (!axis.Unitize())
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Could not define fork axis from trunk point and branch midpoint.");
                return;
            }

            double tTrunk = Project(TP, TP, axis);
            double tA = Project(tipA, TP, axis);
            double tB = Project(tipB, TP, axis);
            double tGuide = Project(FGP, TP, axis);

            double tMin = Math.Min(tTrunk, Math.Min(tA, tB)) - STP * 2.0;
            double tMax = Math.Max(tTrunk, Math.Max(tA, tB)) + STP * 2.0;

            int count = (int)Math.Ceiling((tMax - tMin) / STP) + 1;

            List<Station> stations = new List<Station>();
            List<Curve> allCON = new List<Curve>();

            for (int i = 0; i < count; i++)
            {
                double t = tMin + i * STP;
                Point3d origin = TP + axis * t;
                Plane plane = new Plane(origin, axis);

                Curve[] raw = Mesh.CreateContourCurves(M, plane);

                Station st = new Station();
                st.T = t;

                if (raw != null && raw.Length > 0)
                {
                    List<Curve> cleaned = PrepareCON(raw, tol);

                    List<Section> candidates = new List<Section>();
                    double largestArea = 0.0;

                    foreach (Curve c in cleaned)
                    {
                        if (c == null) continue;

                        double area = CurveArea(c);
                        if (area <= tol * tol) continue;

                        Point3d center = CurveCentroid(c);

                        Section s = new Section();
                        s.Curve = c;
                        s.Area = area;
                        s.Center = center;

                        candidates.Add(s);
                        if (area > largestArea) largestArea = area;
                    }

                    if (candidates.Count > 0)
                    {
                        double minKeepArea = Math.Max(2.0, largestArea * 0.02);

                        candidates.Sort((a, b) => b.Area.CompareTo(a.Area));

                        foreach (Section s in candidates)
                        {
                            if (s.Area >= minKeepArea)
                                st.Sections.Add(s);
                        }

                        if (st.Sections.Count == 0 && candidates.Count > 0)
                            st.Sections.Add(candidates[0]);

                        if (st.Sections.Count > 3)
                            st.Sections = st.Sections.GetRange(0, 3);

                        foreach (Section s in st.Sections)
                            allCON.Add(s.Curve);
                    }
                }

                stations.Add(st);
            }

            if (stations.Count < 2)
                return;

            int barkSplit = FindForkStation(stations, tGuide, searchWindow);
            if (barkSplit < 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Could not detect fork station.");
                return;
            }

            int forkStation = Math.Max(1, barkSplit - 1);

            Section forkSection = PickTrunkSectionAtStation(stations, forkStation, TP, axis);
            if (forkSection == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Could not identify fork trunk section.");
                return;
            }

            List<Section> trunkSections = TraceSections(stations, TP, tTrunk, forkStation, true, TP, axis);
            List<Section> branchASections = TraceSections(stations, tipA, tA, forkStation, false, tipA, axis);
            List<Section> branchBSections = TraceSections(stations, tipB, tB, forkStation, false, tipB, axis);

            if (trunkSections.Count < 1 || branchASections.Count < 1 || branchBSections.Count < 1)
                return;

            HashSet<Section> culled = new HashSet<Section>();

            if (ECV > 0)
            {
                CullFromStart(trunkSections, ECV, culled);
                CullFromStart(branchASections, ECV, culled);
                CullFromStart(branchBSections, ECV, culled);
            }

            ApplySectionCull(stations, culled);

            allCON = CollectCON(stations);

            List<Point3d> trunkPath = SectionsToPath(TP, trunkSections);
            List<Point3d> branchA = SectionsToPath(tipA, branchASections);
            List<Point3d> branchB = SectionsToPath(tipB, branchBSections);

            if (trunkPath.Count < 2 || branchA.Count < 2 || branchB.Count < 2)
                return;

            trunkPath = TrimPathBackAlongItself(trunkPath, MFP);
            if (trunkPath.Count < 2)
                return;

            Point3d finalFork = trunkPath[trunkPath.Count - 1];

            branchA = TrimPathToForkSide(branchA, TP, axis, finalFork, false);
            branchB = TrimPathToForkSide(branchB, TP, axis, finalFork, false);

            if (trunkPath.Count < 2 || branchA.Count < 2 || branchB.Count < 2)
                return;

            trunkPath = SmoothStable(trunkPath, SP);
            branchA = SmoothStable(branchA, SP);
            branchB = SmoothStable(branchB, SP);

            trunkPath = ConvergeToFork(trunkPath, finalFork, true);
            branchA = ConvergeToFork(branchA, finalFork, false);
            branchB = ConvergeToFork(branchB, finalFork, false);

            trunkPath = CullTinySTPs(trunkPath, tol);
            branchA = CullTinySTPs(branchA, tol);
            branchB = CullTinySTPs(branchB, tol);

            Vector3d trunkForkTangent = ComputeTrunkForkTangent(trunkPath, finalFork);
            Vector3d branchForkTangent = -trunkForkTangent;

            List<Curve> curves = new List<Curve>();
            Curve c0 = MakeCurveWithEndTangent(trunkPath, trunkForkTangent);
            Curve c1 = MakeCurveWithEndTangent(branchA, branchForkTangent);
            Curve c2 = MakeCurveWithEndTangent(branchB, branchForkTangent);

            if (c0 != null) curves.Add(c0);
            if (c1 != null) curves.Add(c1);
            if (c2 != null) curves.Add(c2);

            CON = allCON;
            CNTRS = curves;
            FKP = finalFork;

            DA.SetDataList(0, CON);
            DA.SetDataList(1, CNTRS);
            DA.SetData(2, FKP);
        }

        private static double Project(Point3d p, Point3d origin, Vector3d axis)
        {
            Vector3d v = p - origin;
            return v * axis;
        }

        private static List<Section> TraceSections(
            List<Station> stations,
            Point3d start,
            double startT,
            int stop,
            bool trunk,
            Point3d targetRef,
            Vector3d axis)
        {
            List<Section> picked = new List<Section>();

            int idx = ClosestStation(stations, startT);
            int dir = (stop > idx) ? 1 : -1;

            Point3d current = start;

            while (true)
            {
                if (idx < 0 || idx >= stations.Count) break;

                Station st = stations[idx];
                Section best = null;
                double bestScore = double.MaxValue;

                foreach (Section s in st.Sections)
                {
                    double score = s.Center.DistanceTo(current);

                    if (!trunk)
                        score -= s.Area * 0.00005;
                    else
                        score -= s.Area * 0.00002;

                    Vector3d v = s.Center - targetRef;
                    double axial = Math.Abs(v * axis);
                    double radial = Math.Max(0.0, v.Length - axial);
                    score += radial * 0.1;

                    if (score < bestScore)
                    {
                        bestScore = score;
                        best = s;
                    }
                }

                if (best != null)
                {
                    if (picked.Count == 0 || !ReferenceEquals(picked[picked.Count - 1], best))
                        picked.Add(best);

                    current = best.Center;
                }

                if (idx == stop) break;
                idx += dir;
            }

            return picked;
        }

        private static void CullFromStart(List<Section> sections, int count, HashSet<Section> culled)
        {
            if (sections == null || culled == null) return;
            if (count <= 0 || sections.Count == 0) return;

            int n = Math.Min(count, sections.Count);

            for (int i = 0; i < n; i++)
            {
                Section s = sections[i];
                if (s != null)
                    culled.Add(s);
            }

            sections.RemoveRange(0, n);
        }

        private static void ApplySectionCull(List<Station> stations, HashSet<Section> culled)
        {
            if (stations == null || culled == null || culled.Count == 0) return;

            foreach (Station st in stations)
            {
                if (st == null || st.Sections == null || st.Sections.Count == 0) continue;
                st.Sections.RemoveAll(s => culled.Contains(s));
            }
        }

        private static List<Curve> CollectCON(List<Station> stations)
        {
            List<Curve> all = new List<Curve>();
            if (stations == null) return all;

            foreach (Station st in stations)
            {
                if (st == null || st.Sections == null) continue;

                foreach (Section s in st.Sections)
                {
                    if (s != null && s.Curve != null)
                        all.Add(s.Curve);
                }
            }

            return all;
        }

        private static List<Point3d> SectionsToPath(Point3d start, List<Section> sections)
        {
            List<Point3d> pts = new List<Point3d>();
            pts.Add(start);

            if (sections == null) return pts;

            for (int i = 0; i < sections.Count; i++)
            {
                Section s = sections[i];
                if (s == null) continue;

                if (pts[pts.Count - 1].DistanceTo(s.Center) > 1e-9)
                    pts.Add(s.Center);
            }

            return pts;
        }

        private static List<Point3d> TrimPathBackAlongItself(List<Point3d> pts, double backDist)
        {
            List<Point3d> result = new List<Point3d>();
            if (pts == null || pts.Count == 0) return result;
            if (pts.Count == 1)
            {
                result.Add(pts[0]);
                return result;
            }

            if (backDist <= 1e-9)
                return new List<Point3d>(pts);

            for (int i = 0; i < pts.Count - 1; i++)
                result.Add(pts[i]);

            double remaining = backDist;
            Point3d endPt = pts[pts.Count - 1];

            for (int i = pts.Count - 2; i >= 0; i--)
            {
                Point3d a = pts[i];
                Point3d b = endPt;
                double segLen = a.DistanceTo(b);

                if (segLen > remaining + 1e-9)
                {
                    double t = (segLen - remaining) / segLen;
                    Point3d newEnd = new Point3d(
                        a.X + (b.X - a.X) * t,
                        a.Y + (b.Y - a.Y) * t,
                        a.Z + (b.Z - a.Z) * t
                    );

                    result.Add(newEnd);
                    return result;
                }

                remaining -= segLen;
                if (result.Count > 0)
                    result.RemoveAt(result.Count - 1);

                endPt = a;
            }

            result.Clear();
            result.Add(pts[0]);
            if (pts.Count > 1)
                result.Add(pts[1]);

            return result;
        }

        private static Vector3d ComputeTrunkForkTangent(List<Point3d> trunkPath, Point3d fork)
        {
            if (trunkPath != null && trunkPath.Count >= 2)
            {
                Vector3d v = fork - trunkPath[trunkPath.Count - 2];
                if (v.IsValid && v.Length > 1e-9)
                {
                    v.Unitize();
                    return v;
                }
            }

            return Vector3d.ZAxis;
        }

        private static int ClosestStation(List<Station> s, double t)
        {
            int best = 0;
            double bestD = Math.Abs(s[0].T - t);

            for (int i = 1; i < s.Count; i++)
            {
                double d = Math.Abs(s[i].T - t);
                if (d < bestD)
                {
                    bestD = d;
                    best = i;
                }
            }
            return best;
        }

        private static int FindForkStation(List<Station> stations, double tGuide, double searchWindow)
        {
            for (int i = 1; i < stations.Count - 1; i++)
            {
                if (Math.Abs(stations[i].T - tGuide) > searchWindow) continue;

                int prevCount = stations[i - 1].Sections.Count;
                int currCount = stations[i].Sections.Count;
                int nextCount = stations[i + 1].Sections.Count;

                if (prevCount <= 1 && currCount >= 2 && nextCount >= 2)
                    return i;
            }

            for (int i = 1; i < stations.Count; i++)
            {
                if (Math.Abs(stations[i].T - tGuide) > searchWindow) continue;

                if (stations[i - 1].Sections.Count <= 1 && stations[i].Sections.Count >= 2)
                    return i;
            }

            return -1;
        }

        private static Section PickTrunkSectionAtStation(List<Station> stations, int idx, Point3d TP, Vector3d axis)
        {
            if (idx < 0 || idx >= stations.Count) return null;
            if (stations[idx].Sections.Count == 0) return null;

            Section best = null;
            double bestScore = double.MaxValue;

            foreach (Section s in stations[idx].Sections)
            {
                Vector3d v = s.Center - TP;
                double t = v * axis;
                Point3d proj = TP + axis * t;
                double radial = s.Center.DistanceTo(proj);

                double score = radial - s.Area * 0.00005;

                if (score < bestScore)
                {
                    bestScore = score;
                    best = s;
                }
            }

            return best;
        }

        private static double AxisT(Point3d origin, Vector3d axis, Point3d p)
        {
            Vector3d v = p - origin;
            return v * axis;
        }

        private static List<Point3d> TrimPathToForkSide(
            List<Point3d> pts,
            Point3d axisOrigin,
            Vector3d axis,
            Point3d fork,
            bool isTrunk)
        {
            List<Point3d> outPts = new List<Point3d>();
            if (pts == null || pts.Count == 0) return outPts;

            double forkT = AxisT(axisOrigin, axis, fork);
            double eps = 1e-6;

            outPts.Add(pts[0]);

            for (int i = 1; i < pts.Count; i++)
            {
                double t = AxisT(axisOrigin, axis, pts[i]);

                bool keep;
                if (isTrunk)
                    keep = t <= forkT + eps;
                else
                    keep = t >= forkT - eps;

                if (keep)
                {
                    if (pts[i].DistanceTo(outPts[outPts.Count - 1]) > 1e-9)
                        outPts.Add(pts[i]);
                }
            }

            if (outPts.Count < 2 && pts.Count >= 2)
            {
                Point3d best = pts[1];
                double bestD = best.DistanceTo(fork);

                for (int i = 2; i < pts.Count; i++)
                {
                    double d = pts[i].DistanceTo(fork);
                    if (d < bestD)
                    {
                        bestD = d;
                        best = pts[i];
                    }
                }

                if (outPts[outPts.Count - 1].DistanceTo(best) > 1e-9)
                    outPts.Add(best);
            }

            return outPts;
        }

        private static List<Curve> PrepareCON(Curve[] raw, double tol)
        {
            List<Curve> result = new List<Curve>();
            if (raw == null || raw.Length == 0) return result;

            Curve[] joined = Curve.JoinCurves(raw, tol * 2.0);
            Curve[] source = (joined != null && joined.Length > 0) ? joined : raw;

            foreach (Curve c in source)
            {
                Curve cc = CloseCurveSoft(c, tol);
                if (cc == null) continue;

                if (!cc.IsValid) continue;
                if (!cc.IsClosed) continue;

                result.Add(cc);
            }

            return result;
        }

        private static Curve CloseCurveSoft(Curve c, double tol)
        {
            if (c == null) return null;
            if (!c.IsValid) return null;

            if (c.IsClosed) return c;

            Polyline pl;
            if (c.TryGetPolyline(out pl))
            {
                if (pl.Count < 3) return null;

                if (!pl[0].EpsilonEquals(pl[pl.Count - 1], tol * 4.0))
                    pl.Add(pl[0]);

                PolylineCurve plc = new PolylineCurve(pl);
                if (plc.IsClosed) return plc;
            }

            Point3d a = c.PointAtStart;
            Point3d b = c.PointAtEnd;

            if (a.DistanceTo(b) <= tol * 4.0)
            {
                Curve dup = c.DuplicateCurve();
                dup.MakeClosed(tol * 4.0);
                if (dup.IsClosed) return dup;
            }

            return null;
        }

        private static List<Point3d> SmoothStable(List<Point3d> pts, int passes)
        {
            if (pts == null) return new List<Point3d>();
            if (pts.Count < 3) return new List<Point3d>(pts);

            List<Point3d> current = new List<Point3d>(pts);

            for (int k = 0; k < passes; k++)
            {
                List<Point3d> next = new List<Point3d>(current);

                for (int i = 1; i < current.Count - 1; i++)
                {
                    Point3d a = current[i - 1];
                    Point3d b = current[i];
                    Point3d c = current[i + 1];

                    next[i] = new Point3d(
                        (a.X + 2.0 * b.X + c.X) / 4.0,
                        (a.Y + 2.0 * b.Y + c.Y) / 4.0,
                        (a.Z + 2.0 * b.Z + c.Z) / 4.0
                    );
                }

                current = next;
            }

            return current;
        }

        private static List<Point3d> ConvergeToFork(List<Point3d> pts, Point3d fork, bool isTrunk)
        {
            List<Point3d> p = new List<Point3d>(pts);
            if (p.Count == 0) return p;

            p[p.Count - 1] = fork;

            if (p.Count >= 3)
            {
                int i = p.Count - 2;
                double blend = isTrunk ? 0.25 : 0.20;

                p[i] = new Point3d(
                    p[i].X * (1.0 - blend) + fork.X * blend,
                    p[i].Y * (1.0 - blend) + fork.Y * blend,
                    p[i].Z * (1.0 - blend) + fork.Z * blend
                );
            }

            return p;
        }

        private static List<Point3d> CullTinySTPs(List<Point3d> pts, double tol)
        {
            List<Point3d> clean = new List<Point3d>();
            if (pts == null || pts.Count == 0) return clean;

            clean.Add(pts[0]);

            for (int i = 1; i < pts.Count; i++)
            {
                if (pts[i].DistanceTo(clean[clean.Count - 1]) > tol * 0.5)
                    clean.Add(pts[i]);
            }

            if (clean.Count == 1 && pts.Count > 1)
                clean.Add(pts[pts.Count - 1]);

            return clean;
        }

        private static Curve MakeCurveWithEndTangent(List<Point3d> pts, Vector3d endTangent)
        {
            if (pts == null || pts.Count < 2) return null;

            if (pts.Count == 2)
                return new LineCurve(pts[0], pts[1]);

            Vector3d startTangent = pts[1] - pts[0];
            if (!startTangent.IsValid || startTangent.Length <= 1e-9)
                startTangent = Vector3d.Unset;
            else
                startTangent.Unitize();

            if (!endTangent.IsValid || endTangent.Length <= 1e-9)
                return Curve.CreateInterpolatedCurve(pts, 3);

            endTangent.Unitize();

            try
            {
                return Curve.CreateInterpolatedCurve(
                    pts,
                    3,
                    CurveKnotStyle.Chord,
                    startTangent,
                    endTangent);
            }
            catch
            {
                return Curve.CreateInterpolatedCurve(pts, 3);
            }
        }

        private static double CurveArea(Curve c)
        {
            try
            {
                AreaMassProperties amp = AreaMassProperties.Compute(c);
                if (amp != null) return Math.Abs(amp.Area);
            }
            catch
            {
            }
            return 0.0;
        }

        private static Point3d CurveCentroid(Curve c)
        {
            try
            {
                AreaMassProperties amp = AreaMassProperties.Compute(c);
                if (amp != null) return amp.Centroid;
            }
            catch
            {
            }

            return c.PointAtNormalizedLength(0.5);
        }

        public override GH_Exposure Exposure => GH_Exposure.primary;
        public override Guid ComponentGuid => new Guid("13D43003-C7CF-44F8-8B87-67319CCECDF1");

        protected override System.Drawing.Bitmap Icon => Properties.Resources.ForkEval;

        private class Section
        {
            public Curve Curve;
            public Point3d Center;
            public double Area;
        }

        private class Station
        {
            public double T;
            public List<Section> Sections = new List<Section>();
        }
    }
}