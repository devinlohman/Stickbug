using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Stickbug
{
    public class MeshPithCurveComponent : GH_Component
    {
        public MeshPithCurveComponent()
          : base("Log Evaluation", "LogEval",
              "Evaluate an unforked log mesh",
              "Stickbug", "Log Data")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input mesh", GH_ParamAccess.item);
            pManager.AddPointParameter("End Pith 1", "EP1", "First pith location", GH_ParamAccess.item);
            pManager.AddPointParameter("End Pith 2", "EP2", "Second pith location", GH_ParamAccess.item);
            pManager.AddNumberParameter("Step", "STP", "Contour step distance", GH_ParamAccess.item, 100.0);
            pManager.AddNumberParameter("Branch Distance", "BRD", "Distance beyond which to reject branches", GH_ParamAccess.item, 10.0);
            pManager.AddNumberParameter("Offset Blend", "OB", "Blend factor from baseline toward endpoint offsets (0 to 1)", GH_ParamAccess.item, 1.0);
            pManager.AddIntegerParameter("Smooth Passes", "SP", "Number of smoothing passes", GH_ParamAccess.item, 0);
            pManager.AddIntegerParameter("End Cull Value", "ECV", "Number of contours to cull from each end", GH_ParamAccess.item, 0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Contours", "CON", "Utilized contour curves", GH_ParamAccess.list);
            pManager.AddCurveParameter("Center Curve", "CC", "Interpolated center curve estimate", GH_ParamAccess.item);
            pManager.AddCurveParameter("Pith Curve", "PC", "Final pith curve estimate", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh M = null;
            Point3d EP1 = Point3d.Unset;
            Point3d EP2 = Point3d.Unset;
            double STP = 0.0;
            double BRD = 0.0;
            double OB = 1.0;
            int SP = 0;
            int ECV = 0;

            if (!DA.GetData(0, ref M)) return;
            if (!DA.GetData(1, ref EP1)) return;
            if (!DA.GetData(2, ref EP2)) return;
            if (!DA.GetData(3, ref STP)) return;
            if (!DA.GetData(4, ref BRD)) return;
            if (!DA.GetData(5, ref OB)) return;
            if (!DA.GetData(6, ref SP)) return;
            if (!DA.GetData(7, ref ECV)) return;

            List<Curve> CON = null;
            Curve CC = null;
            Curve PC = null;

            if (M == null || !M.IsValid)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Input mesh is null or invalid");
                return;
            }

            Vector3d axis = EP2 - EP1;
            double axisLen = axis.Length;
            if (axisLen <= 1e-9)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Endpoints are coincident or too close together");
                return;
            }

            axis.Unitize();

            if (STP <= 0.0)
                STP = Math.Max(10.0, axisLen / 40.0);

            if (BRD <= 0.0)
                BRD = STP * 2.0;

            OB = Clamp01(OB);
            if (SP < 0) SP = 0;
            if (ECV < 0) ECV = 0;

            int stationCount = Math.Max(3, (int)Math.Ceiling(axisLen / STP) + 1);

            int maxCull = Math.Max(0, (stationCount - 1) / 2);
            if (ECV > maxCull)
                ECV = maxCull;

            List<StationData> stations = new List<StationData>(stationCount);

            for (int i = 0; i < stationCount; i++)
            {
                double t = (stationCount == 1) ? 0.0 : (double)i / (double)(stationCount - 1);
                Point3d stationPt = EP1 + axis * (axisLen * t);
                Plane sectionPlane = new Plane(stationPt, axis);

                bool culled = (i < ECV) || (i > stationCount - 1 - ECV);

                Curve[] contours = null;
                if (!culled)
                {
                    try
                    {
                        contours = Mesh.CreateContourCurves(M, sectionPlane);
                    }
                    catch
                    {
                        contours = null;
                    }
                }

                StationData sd = new StationData
                {
                    Index = i,
                    T = t,
                    AxisPoint = stationPt,
                    Plane = sectionPlane,
                    Culled = culled,
                    Candidates = !culled ? BuildCandidates(contours, sectionPlane, stationPt, STP) : new List<SectionCandidate>()
                };

                stations.Add(sd);
            }

            if (stations.Count == 0)
                return;

            ChooseBestCandidateChain(stations, EP1, EP2, BRD);

            List<Curve> chosenCurves = new List<Curve>();
            List<Point3d> baselineCenters = new List<Point3d>();

            for (int i = 0; i < stations.Count; i++)
            {
                StationData sd = stations[i];

                if (sd.Chosen != null)
                {
                    chosenCurves.Add(sd.Chosen.Curve);
                    baselineCenters.Add(sd.Chosen.Center);
                }
                else
                {
                    Point3d fallback = EstimateFallbackFromNeighbors(stations, i, EP1, EP2);
                    baselineCenters.Add(fallback);
                }
            }

            baselineCenters = RemoveOutliersLocal(baselineCenters, BRD);
            for (int p = 0; p < SP; p++)
                baselineCenters = SmoothPointChain(baselineCenters, false);

            if (baselineCenters.Count < 2)
                return;

            Curve centerCurve;
            try
            {
                centerCurve = Curve.CreateInterpolatedCurve(baselineCenters, 3);
            }
            catch
            {
                centerCurve = new PolylineCurve(baselineCenters);
            }

            Vector3d startOffset = EP1 - baselineCenters[0];
            Vector3d endOffset = EP2 - baselineCenters[baselineCenters.Count - 1];

            List<Point3d> finalCenters = new List<Point3d>(baselineCenters.Count);

            for (int i = 0; i < baselineCenters.Count; i++)
            {
                double t = (baselineCenters.Count == 1) ? 0.0 : (double)i / (double)(baselineCenters.Count - 1);
                double s = SmoothStep(t);

                Vector3d interpOffset = LerpVector(startOffset, endOffset, s);
                Vector3d applied = interpOffset * OB;

                Point3d pt = baselineCenters[i] + applied;
                finalCenters.Add(pt);
            }

            if (finalCenters.Count >= 2)
            {
                finalCenters[0] = EP1;
                finalCenters[finalCenters.Count - 1] = EP2;
            }

            for (int p = 0; p < SP; p++)
                finalCenters = SmoothPointChain(finalCenters, true);

            if (finalCenters.Count >= 2)
            {
                finalCenters[0] = EP1;
                finalCenters[finalCenters.Count - 1] = EP2;
            }

            Curve pithCurve = null;
            if (finalCenters.Count >= 2)
            {
                try
                {
                    pithCurve = Curve.CreateInterpolatedCurve(finalCenters, 3);
                }
                catch
                {
                    pithCurve = new PolylineCurve(finalCenters);
                }
            }

            CON = chosenCurves;
            CC = centerCurve;
            PC = pithCurve;

            DA.SetDataList(0, CON);
            DA.SetData(1, CC);
            DA.SetData(2, PC);
        }

        private static List<SectionCandidate> BuildCandidates(Curve[] contours, Plane plane, Point3d axisPoint, double STP)
        {
            List<SectionCandidate> result = new List<SectionCandidate>();
            if (contours == null || contours.Length == 0)
                return result;

            for (int i = 0; i < contours.Length; i++)
            {
                Curve crv = contours[i];
                if (crv == null || !crv.IsValid)
                    continue;

                Curve evalCrv = EnsureClosedCurve(crv);
                if (evalCrv == null || !evalCrv.IsValid)
                    continue;

                List<Point3d> samples = SampleCurve(evalCrv, 48);
                if (samples == null || samples.Count < 4)
                    continue;

                BoundingBox bb = new BoundingBox(samples);
                double diag = bb.Diagonal.Length;
                if (diag < STP * 0.10)
                    continue;

                bool isClosed = evalCrv.IsClosed;
                Point3d center = EstimateCenter(evalCrv, plane, axisPoint, true);

                double area = 0.0;
                if (isClosed)
                {
                    try
                    {
                        AreaMassProperties amp = AreaMassProperties.Compute(evalCrv);
                        if (amp != null) area = Math.Abs(amp.Area);
                    }
                    catch
                    {
                        area = 0.0;
                    }
                }

                double compactness = EstimateCompactness(samples, center);

                double u, v;
                plane.ClosestParameter(center, out u, out v);
                double radialToAxis = Math.Sqrt(u * u + v * v);

                SectionCandidate c = new SectionCandidate
                {
                    Curve = evalCrv,
                    Center = center,
                    Area = area,
                    Size = diag,
                    Compactness = compactness,
                    IsClosed = isClosed,
                    RadialToAxis = radialToAxis
                };

                c.LocalScore = LocalCandidateScore(c);
                result.Add(c);
            }

            return result;
        }

        private static Curve EnsureClosedCurve(Curve crv)
        {
            if (crv == null || !crv.IsValid)
                return null;

            if (crv.IsClosed)
                return crv.DuplicateCurve();

            Polyline pl;
            if (crv.TryGetPolyline(out pl))
            {
                if (pl.Count >= 2)
                {
                    Polyline closed = new Polyline(pl);
                    if (!closed[0].EpsilonEquals(closed[closed.Count - 1], 1e-9))
                        closed.Add(closed[0]);

                    PolylineCurve plc = new PolylineCurve(closed);
                    if (plc.IsValid)
                        return plc;
                }
            }

            Point3d start = crv.PointAtStart;
            Point3d end = crv.PointAtEnd;

            LineCurve closingSeg = new LineCurve(end, start);
            Curve[] joined = Curve.JoinCurves(new Curve[] { crv.DuplicateCurve(), closingSeg }, 1e-6);

            if (joined != null && joined.Length > 0)
            {
                for (int i = 0; i < joined.Length; i++)
                {
                    if (joined[i] != null && joined[i].IsValid && joined[i].IsClosed)
                        return joined[i];
                }

                if (joined[0] != null && joined[0].IsValid)
                    return joined[0];
            }

            return crv.DuplicateCurve();
        }

        private static double LocalCandidateScore(SectionCandidate c)
        {
            double closedBonus = c.IsClosed ? 1.5 : 0.0;
            double areaTerm = Math.Sqrt(Math.Max(0.0, c.Area));
            double sizeTerm = c.Size;
            double compactTerm = 3.0 * c.Compactness;
            double radialPenalty = 0.15 * c.RadialToAxis;

            return closedBonus + areaTerm + 0.35 * sizeTerm + compactTerm - radialPenalty;
        }

        private static void ChooseBestCandidateChain(List<StationData> stations, Point3d EP1, Point3d EP2, double BRD)
        {
            int n = stations.Count;
            if (n == 0) return;

            for (int i = 0; i < n; i++)
            {
                StationData s = stations[i];
                if (s.Candidates.Count == 0) continue;

                for (int j = 0; j < s.Candidates.Count; j++)
                {
                    s.Candidates[j].BestCost = double.PositiveInfinity;
                    s.Candidates[j].PrevIndex = -1;
                }
            }

            int firstWithCandidates = -1;
            for (int i = 0; i < n; i++)
            {
                if (stations[i].Candidates.Count > 0)
                {
                    firstWithCandidates = i;
                    break;
                }
            }

            if (firstWithCandidates < 0)
                return;

            for (int j = 0; j < stations[firstWithCandidates].Candidates.Count; j++)
            {
                SectionCandidate c = stations[firstWithCandidates].Candidates[j];
                double startPenalty = c.Center.DistanceTo(EP1);
                c.BestCost = -c.LocalScore + 0.25 * startPenalty;
                c.PrevIndex = -1;
            }

            for (int i = firstWithCandidates + 1; i < n; i++)
            {
                StationData curr = stations[i];
                if (curr.Candidates.Count == 0)
                    continue;

                int prevIdx = FindPreviousStationWithCandidates(stations, i - 1);
                if (prevIdx < 0)
                {
                    for (int j = 0; j < curr.Candidates.Count; j++)
                    {
                        SectionCandidate c = curr.Candidates[j];
                        double startPenalty = c.Center.DistanceTo(EP1);
                        c.BestCost = -c.LocalScore + 0.25 * startPenalty;
                    }
                    continue;
                }

                StationData prev = stations[prevIdx];

                for (int j = 0; j < curr.Candidates.Count; j++)
                {
                    SectionCandidate cj = curr.Candidates[j];

                    double bestCost = double.PositiveInfinity;
                    int bestPrev = -1;

                    for (int k = 0; k < prev.Candidates.Count; k++)
                    {
                        SectionCandidate pk = prev.Candidates[k];
                        if (double.IsInfinity(pk.BestCost))
                            continue;

                        double stepJump = cj.Center.DistanceTo(pk.Center);
                        double continuityPenalty = 1.25 * stepJump;

                        if (stepJump > BRD)
                            continuityPenalty += 2.0 * (stepJump - BRD);

                        double cost = pk.BestCost - cj.LocalScore + continuityPenalty;

                        if (cost < bestCost)
                        {
                            bestCost = cost;
                            bestPrev = k;
                        }
                    }

                    if (bestPrev < 0)
                        bestCost = -cj.LocalScore + 0.35 * cj.Center.DistanceTo(EP1);

                    cj.BestCost = bestCost;
                    cj.PrevIndex = bestPrev;
                }
            }

            int lastWithCandidates = -1;
            for (int i = n - 1; i >= 0; i--)
            {
                if (stations[i].Candidates.Count > 0)
                {
                    lastWithCandidates = i;
                    break;
                }
            }

            if (lastWithCandidates < 0)
                return;

            double bestEndCost = double.PositiveInfinity;
            int bestEndCand = -1;

            for (int j = 0; j < stations[lastWithCandidates].Candidates.Count; j++)
            {
                SectionCandidate c = stations[lastWithCandidates].Candidates[j];
                if (double.IsInfinity(c.BestCost))
                    continue;

                double endPenalty = 0.25 * c.Center.DistanceTo(EP2);
                double total = c.BestCost + endPenalty;

                if (total < bestEndCost)
                {
                    bestEndCost = total;
                    bestEndCand = j;
                }
            }

            if (bestEndCand < 0)
                return;

            int stationIndex = lastWithCandidates;
            int candIndex = bestEndCand;

            while (stationIndex >= 0 && candIndex >= 0)
            {
                StationData s = stations[stationIndex];
                s.Chosen = s.Candidates[candIndex];

                int prevStationIndex = FindPreviousStationWithCandidates(stations, stationIndex - 1);
                int prevCandIndex = s.Candidates[candIndex].PrevIndex;

                stationIndex = prevStationIndex;
                candIndex = prevCandIndex;
            }

            for (int i = 0; i < n; i++)
            {
                if (stations[i].Chosen != null)
                    continue;

                if (stations[i].Candidates.Count == 0)
                    continue;

                Point3d target = EstimateFallbackFromNeighbors(stations, i, EP1, EP2);

                double bestDist = double.MaxValue;
                SectionCandidate best = null;

                for (int j = 0; j < stations[i].Candidates.Count; j++)
                {
                    double d = stations[i].Candidates[j].Center.DistanceTo(target);
                    if (d < bestDist)
                    {
                        bestDist = d;
                        best = stations[i].Candidates[j];
                    }
                }

                stations[i].Chosen = best;
            }
        }

        private static int FindPreviousStationWithCandidates(List<StationData> stations, int startIndex)
        {
            for (int i = startIndex; i >= 0; i--)
            {
                if (stations[i].Candidates.Count > 0)
                    return i;
            }
            return -1;
        }

        private static Point3d EstimateCenter(Curve crv, Plane plane, Point3d axisPoint, bool preferAreaCentroid)
        {
            if (preferAreaCentroid && crv != null && crv.IsClosed)
            {
                try
                {
                    AreaMassProperties amp = AreaMassProperties.Compute(crv);
                    if (amp != null)
                        return amp.Centroid;
                }
                catch
                {
                }
            }

            List<Point3d> pts = SampleCurve(crv, 60);
            if (pts == null || pts.Count == 0)
                return axisPoint;

            double u0, v0;
            plane.ClosestParameter(axisPoint, out u0, out v0);

            double sumW = 0.0;
            double sumU = 0.0;
            double sumV = 0.0;

            for (int i = 0; i < pts.Count; i++)
            {
                double u, v;
                plane.ClosestParameter(pts[i], out u, out v);

                double du = u - u0;
                double dv = v - v0;
                double d2 = du * du + dv * dv;

                double w = 1.0 / (1e-6 + d2);

                sumW += w;
                sumU += u * w;
                sumV += v * w;
            }

            if (sumW <= 1e-12)
                return axisPoint;

            return plane.PointAt(sumU / sumW, sumV / sumW);
        }

        private static List<Point3d> SampleCurve(Curve crv, int divisions)
        {
            List<Point3d> pts = new List<Point3d>();
            if (crv == null || !crv.IsValid)
                return pts;

            divisions = Math.Max(4, divisions);

            double t0 = crv.Domain.T0;
            double t1 = crv.Domain.T1;

            for (int i = 0; i <= divisions; i++)
            {
                double a = (double)i / (double)divisions;
                double t = t0 + (t1 - t0) * a;
                pts.Add(crv.PointAt(t));
            }

            return pts;
        }

        private static double EstimateCompactness(List<Point3d> pts, Point3d center)
        {
            if (pts == null || pts.Count < 3)
                return 0.0;

            double mean = 0.0;
            for (int i = 0; i < pts.Count; i++)
                mean += pts[i].DistanceTo(center);
            mean /= pts.Count;

            if (mean <= 1e-12)
                return 1.0;

            double var = 0.0;
            for (int i = 0; i < pts.Count; i++)
            {
                double d = pts[i].DistanceTo(center) - mean;
                var += d * d;
            }
            var /= pts.Count;

            double std = Math.Sqrt(var);
            return 1.0 / (1.0 + (std / mean));
        }

        private static Point3d EstimateFallbackFromNeighbors(List<StationData> stations, int idx, Point3d EP1, Point3d EP2)
        {
            Point3d? prev = null;
            Point3d? next = null;

            for (int i = idx - 1; i >= 0; i--)
            {
                if (stations[i].Chosen != null)
                {
                    prev = stations[i].Chosen.Center;
                    break;
                }
            }

            for (int i = idx + 1; i < stations.Count; i++)
            {
                if (stations[i].Chosen != null)
                {
                    next = stations[i].Chosen.Center;
                    break;
                }
            }

            if (prev.HasValue && next.HasValue)
                return LerpPoint(prev.Value, next.Value, 0.5);

            if (prev.HasValue)
                return prev.Value;

            if (next.HasValue)
                return next.Value;

            double t = (double)idx / (double)Math.Max(1, stations.Count - 1);
            return LerpPoint(EP1, EP2, t);
        }

        private static List<Point3d> RemoveOutliersLocal(List<Point3d> pts, double threshold)
        {
            if (pts == null || pts.Count < 3)
                return pts;

            List<Point3d> result = new List<Point3d>(pts);

            for (int i = 1; i < result.Count - 1; i++)
            {
                Point3d prev = result[i - 1];
                Point3d curr = result[i];
                Point3d next = result[i + 1];

                Point3d localTrend = LerpPoint(prev, next, 0.5);
                double d = curr.DistanceTo(localTrend);

                if (d > threshold)
                    result[i] = LerpPoint(curr, localTrend, 0.65);
            }

            return result;
        }

        private static List<Point3d> SmoothPointChain(List<Point3d> pts, bool preserveEnds)
        {
            if (pts == null || pts.Count < 3)
                return pts;

            List<Point3d> result = new List<Point3d>(pts);

            int start = preserveEnds ? 1 : 0;
            int end = preserveEnds ? pts.Count - 2 : pts.Count - 1;

            for (int i = start; i <= end; i++)
            {
                if (i == 0 || i == pts.Count - 1)
                    continue;

                Point3d a = pts[i - 1];
                Point3d b = pts[i];
                Point3d c = pts[i + 1];

                result[i] = new Point3d(
                    0.25 * a.X + 0.50 * b.X + 0.25 * c.X,
                    0.25 * a.Y + 0.50 * b.Y + 0.25 * c.Y,
                    0.25 * a.Z + 0.50 * b.Z + 0.25 * c.Z
                );
            }

            return result;
        }

        private static Point3d LerpPoint(Point3d a, Point3d b, double t)
        {
            return new Point3d(
                a.X + (b.X - a.X) * t,
                a.Y + (b.Y - a.Y) * t,
                a.Z + (b.Z - a.Z) * t
            );
        }

        private static Vector3d LerpVector(Vector3d a, Vector3d b, double t)
        {
            return new Vector3d(
                a.X + (b.X - a.X) * t,
                a.Y + (b.Y - a.Y) * t,
                a.Z + (b.Z - a.Z) * t
            );
        }

        private static double SmoothStep(double t)
        {
            if (t < 0.0) t = 0.0;
            if (t > 1.0) t = 1.0;
            return t * t * (3.0 - 2.0 * t);
        }

        private static double Clamp01(double x)
        {
            if (x < 0.0) return 0.0;
            if (x > 1.0) return 1.0;
            return x;
        }

        public override GH_Exposure Exposure => GH_Exposure.primary;
        public override Guid ComponentGuid => new Guid("CC40DECB-14A3-43C1-9275-389A0D064176");

        protected override System.Drawing.Bitmap Icon => Properties.Resources.LogEval;

        private class SectionCandidate
        {
            public Curve Curve;
            public Point3d Center;
            public double Area;
            public double Size;
            public double Compactness;
            public bool IsClosed;
            public double RadialToAxis;
            public double LocalScore;
            public double BestCost;
            public int PrevIndex;
        }

        private class StationData
        {
            public int Index;
            public double T;
            public Point3d AxisPoint;
            public Plane Plane;
            public bool Culled;
            public List<SectionCandidate> Candidates = new List<SectionCandidate>();
            public SectionCandidate Chosen;
        }
    }
}