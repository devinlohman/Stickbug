using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Grasshopper.Kernel.Parameters;
using Rhino;
using Rhino.Geometry;

namespace Stickbug
{
    public class OffsetSpiralWithIslandsComponent : GH_Component
    {
        private bool _useDegreesPR = false;

        public OffsetSpiralWithIslandsComponent()
          : base("Planar Offset", "POffset",
              "Use a planar bounding curve to create an orientable, offset milling toolpath",
              "Stickbug", "Planar")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Curve", "C", "Closed planar bounding curve", GH_ParamAccess.item);
            pManager.AddNumberParameter("Step Over", "STO", "Step-over distance", GH_ParamAccess.item, 10.0);
            pManager.AddNumberParameter("Bit Diameter", "BD", "Tool bit diameter", GH_ParamAccess.item, 10.0);
            pManager.AddAngleParameter("Plane Rotation", "PR", "Rotate output planes", GH_ParamAccess.item, 0.0);
            pManager.AddNumberParameter("Lift Height", "LHT", "Safe lift height", GH_ParamAccess.item, 10.0);
            pManager.AddNumberParameter("Safe Exit", "SE", "Safe inward move at the end of the toolpath", GH_ParamAccess.item, 0.0);
            pManager.AddBooleanParameter("Reverse Cut", "RCD", "Reverse cut direction", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("Flip Safe Up", "FSU", "Flip safe-up direction", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("Flip Planes", "FLP", "Flip output planes", GH_ParamAccess.item, false);
            pManager.AddGenericParameter("Entry Options", "EO", "Optional helical entry options", GH_ParamAccess.item);
            pManager[9].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddPlaneParameter("Planes", "PL", "Ordered toolpath planes", GH_ParamAccess.list);
            pManager.AddCurveParameter("Toolpath", "OC", "Output polyline curve through plane origins", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Jump Indices", "JI", "Indices of jump-up and jump-over points in the final plane list", GH_ParamAccess.list);
        }

        protected override void BeforeSolveInstance()
        {
            _useDegreesPR = false;

            Param_Number prParam = Params.Input[3] as Param_Number;
            if (prParam != null)
                _useDegreesPR = prParam.UseDegrees;
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Curve C = null;
            double STO = 0.0;
            double BD = 0.0;
            double PR = 0.0;
            double LHT = 0.0;
            double SE = 0.0;
            bool RCD = false;
            bool FSU = false;
            bool FLP = false;
            object EO = null;

            if (!DA.GetData(0, ref C)) return;
            if (!DA.GetData(1, ref STO)) return;
            if (!DA.GetData(2, ref BD)) return;
            if (!DA.GetData(3, ref PR)) return;

            if (_useDegreesPR)
                PR = RhinoMath.ToRadians(PR);

            if (!DA.GetData(4, ref LHT)) return;
            if (!DA.GetData(5, ref SE)) return;
            if (!DA.GetData(6, ref RCD)) return;
            if (!DA.GetData(7, ref FSU)) return;
            if (!DA.GetData(8, ref FLP)) return;
            DA.GetData(9, ref EO);

            if (C == null || !C.IsValid || !C.IsClosed || STO <= 0.0 || BD <= 0.0)
                return;

            double tol = RhinoDoc.ActiveDoc != null ? RhinoDoc.ActiveDoc.ModelAbsoluteTolerance : 0.01;
            double eps = Math.Max(10.0 * tol, 1e-6);

            Plane basePlane;
            if (!C.TryGetPlane(out basePlane, tol))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Input curve must be planar");
                return;
            }

            CurveOrientation ori0 = C.ClosedCurveOrientation(basePlane);
            if (ori0 == CurveOrientation.Clockwise)
                basePlane.Flip();

            Vector3d fixedX = basePlane.XAxis;
            Vector3d fixedY = basePlane.YAxis;

            const double chordInner = 0.5;
            const double chordOuter = 0.05;

            double SafeArea(Curve crv)
            {
                AreaMassProperties amp = AreaMassProperties.Compute(crv, tol);
                if (amp == null) return 0.0;
                return Math.Abs(amp.Area);
            }

            Point3d ProbePointOnCurve(Curve crv)
            {
                double t = crv.Domain.ParameterAt(0.37);
                return crv.PointAt(t);
            }

            bool ContainsPoint(Curve boundary, Point3d pt)
            {
                PointContainment rel = boundary.Contains(pt, basePlane, tol);
                return rel == PointContainment.Inside || rel == PointContainment.Coincident;
            }

            int AddPointClean(List<Point3d> path, Point3d pt)
            {
                if (path.Count == 0)
                {
                    path.Add(pt);
                    return 0;
                }

                Point3d lastInPath = path[path.Count - 1];
                if (lastInPath.DistanceTo(pt) <= eps)
                    return path.Count - 1;

                path.Add(pt);

                if (path.Count >= 2)
                {
                    int n = path.Count;
                    if (path[n - 2].DistanceTo(path[n - 1]) <= eps)
                    {
                        path.RemoveAt(n - 1);
                        return n - 2;
                    }
                }

                if (path.Count >= 3)
                {
                    int n = path.Count;
                    Point3d a = path[n - 3];
                    Point3d b = path[n - 2];
                    Point3d c = path[n - 1];

                    double ab = a.DistanceTo(b);
                    double bc = b.DistanceTo(c);

                    if (ab <= eps || bc <= eps)
                    {
                        path.RemoveAt(n - 2);
                        return n - 2;
                    }

                    Vector3d v1 = b - a;
                    Vector3d v2 = c - b;
                    if (v1.Unitize() && v2.Unitize())
                    {
                        double dot = Vector3d.Multiply(v1, v2);
                        if (dot < -0.98 && (ab < 5.0 * eps || bc < 5.0 * eps))
                        {
                            path.RemoveAt(n - 2);
                            return n - 2;
                        }
                    }
                }

                return path.Count - 1;
            }

            List<Curve> OffsetInwardOnce(Curve current, double dist)
            {
                var result = new List<Curve>();
                if (current == null || !current.IsValid || !current.IsClosed) return result;
                if (dist <= tol)
                {
                    result.Add(current.DuplicateCurve());
                    return result;
                }

                double curArea = SafeArea(current);
                if (curArea <= 0.0) return result;

                Curve[] offA = current.Offset(basePlane, dist, tol, CurveOffsetCornerStyle.Round);
                Curve[] offB = current.Offset(basePlane, -dist, tol, CurveOffsetCornerStyle.Round);

                List<Curve> Process(Curve[] arr)
                {
                    var tmp = new List<Curve>();
                    if (arr == null) return tmp;

                    Curve[] joined = Curve.JoinCurves(arr, tol);
                    if (joined == null || joined.Length == 0) joined = arr;

                    foreach (Curve cc in joined)
                    {
                        if (cc == null || !cc.IsValid || !cc.IsClosed) continue;
                        double a = SafeArea(cc);
                        if (a < curArea - 1e-9) tmp.Add(cc);
                    }
                    return tmp;
                }

                List<Curve> candA = Process(offA);
                List<Curve> candB = Process(offB);

                double sumA = 0.0;
                foreach (Curve x in candA) sumA += SafeArea(x);

                double sumB = 0.0;
                foreach (Curve x in candB) sumB += SafeArea(x);

                if (candA.Count == 0) return candB;
                if (candB.Count == 0) return candA;
                return sumB > sumA ? candB : candA;
            }

            List<Curve> AllInwardOffsets(List<Curve> currents, double dist)
            {
                var raw = new List<Curve>();
                if (currents == null) return raw;
                foreach (Curve curve in currents)
                    raw.AddRange(OffsetInwardOnce(curve, dist));
                return raw;
            }

            double DistancePointToSegment(Point3d pt, Point3d a, Point3d b)
            {
                Vector3d ab = b - a;
                double ab2 = ab.SquareLength;
                if (ab2 < 1e-18) return pt.DistanceTo(a);

                double tproj = Vector3d.Multiply(pt - a, ab) / ab2;
                if (tproj < 0.0) tproj = 0.0;
                else if (tproj > 1.0) tproj = 1.0;

                Point3d q = a + tproj * ab;
                return pt.DistanceTo(q);
            }

            void SampleCurveChordError(Curve crv, double maxErr, List<double> tOut)
            {
                if (crv == null || !crv.IsValid) return;

                Interval d = crv.Domain;
                double t0 = d.T0;
                double t1 = d.T1;

                int safety = 0;
                const int maxSegs = 20000;

                void Recurse(double a, double b)
                {
                    if (safety++ > maxSegs)
                    {
                        tOut.Add(b);
                        return;
                    }

                    double m = 0.5 * (a + b);
                    Point3d pa = crv.PointAt(a);
                    Point3d pb = crv.PointAt(b);
                    Point3d pm = crv.PointAt(m);

                    double dist = DistancePointToSegment(pm, pa, pb);

                    if (dist > maxErr && Math.Abs(b - a) > 1e-12)
                    {
                        Recurse(a, m);
                        Recurse(m, b);
                    }
                    else
                    {
                        tOut.Add(b);
                    }
                }

                tOut.Add(t0);
                Recurse(t0, t1);
            }

            List<Point3d> GetChordPointsForced(Curve loop, double chordErr, CurveOrientation wantOri)
            {
                var pts = new List<Point3d>();
                if (loop == null || !loop.IsValid || !loop.IsClosed) return pts;

                var tSamples = new List<double>();
                SampleCurveChordError(loop, chordErr, tSamples);
                if (tSamples.Count < 2) return pts;

                Point3d prevPt = Point3d.Unset;
                foreach (double t in tSamples)
                {
                    Point3d pt = loop.PointAt(t);
                    if (!prevPt.IsValid || pt.DistanceTo(prevPt) > 1e-9)
                        pts.Add(pt);
                    prevPt = pt;
                }

                if (pts.Count > 2 && pts[0].DistanceTo(pts[pts.Count - 1]) < 1e-9)
                    pts.RemoveAt(pts.Count - 1);

                if (pts.Count < 2) return pts;

                CurveOrientation loopOri = loop.ClosedCurveOrientation(basePlane);
                bool needReverse = loopOri != wantOri &&
                                   loopOri != CurveOrientation.Undefined &&
                                   wantOri != CurveOrientation.Undefined;

                if (RCD) needReverse = !needReverse;
                if (needReverse) pts.Reverse();

                return pts;
            }

            int FindBestSeamIndex(List<Point3d> pts, Point3d seam)
            {
                int n = pts.Count;
                if (n == 0) return 0;
                if (n == 1) return 0;

                double bestD = double.PositiveInfinity;
                int bestInsert = 0;

                for (int i = 0; i < n; i++)
                {
                    int j = (i + 1) % n;
                    double d = DistancePointToSegment(seam, pts[i], pts[j]);
                    if (d < bestD)
                    {
                        bestD = d;
                        bestInsert = j;
                    }
                }

                return bestInsert;
            }

            Vector3d SafeZ()
            {
                Vector3d z = Vector3d.CrossProduct(fixedX, fixedY);
                if (!z.Unitize()) z = basePlane.Normal;
                if (!z.Unitize()) z = Vector3d.ZAxis;
                if (FSU) z = -z;
                return z;
            }

            Point3d LiftPoint(Point3d pt)
            {
                return pt + SafeZ() * LHT;
            }

            double ChordForNodeLevel(int level)
            {
                return level == 0 ? chordOuter : chordInner;
            }

            Point3d GetLoopSeamPoint(Curve loop, CurveOrientation wantOri, double chordErr, Point3d startNear)
            {
                List<Point3d> chordPts = GetChordPointsForced(loop, chordErr, wantOri);
                if (chordPts == null || chordPts.Count < 2) return loop.PointAtStart;

                double tSeam;
                Point3d seamPt;
                if (!loop.ClosestPoint(startNear, out tSeam))
                    seamPt = chordPts[0];
                else
                    seamPt = loop.PointAt(tSeam);

                return seamPt;
            }

            void AppendRing(List<Point3d> path, Curve loop, CurveOrientation wantOri, double chordErr, ref Point3d currentPtRef)
            {
                if (loop == null || !loop.IsValid || !loop.IsClosed) return;

                List<Point3d> chordPts = GetChordPointsForced(loop, chordErr, wantOri);
                if (chordPts == null || chordPts.Count < 2) return;

                Point3d startNear = currentPtRef.IsValid ? currentPtRef : loop.GetBoundingBox(true).Center;
                Point3d seamPt = GetLoopSeamPoint(loop, wantOri, chordErr, startNear);

                int startIdx = FindBestSeamIndex(chordPts, seamPt);

                if (seamPt.DistanceTo(chordPts[startIdx]) > eps &&
                    seamPt.DistanceTo(chordPts[(startIdx - 1 + chordPts.Count) % chordPts.Count]) > eps)
                {
                    chordPts.Insert(startIdx, seamPt);
                }

                Point3d ringStart = seamPt;

                AddPointClean(path, ringStart);

                int n = chordPts.Count;
                int seamIdx = FindBestSeamIndex(chordPts, ringStart);
                if (ringStart.DistanceTo(chordPts[seamIdx]) <= eps)
                {
                    startIdx = seamIdx;
                }
                else if (ringStart.DistanceTo(chordPts[(seamIdx - 1 + n) % n]) <= eps)
                {
                    startIdx = (seamIdx - 1 + n) % n;
                }
                else
                {
                    startIdx = seamIdx;
                }

                for (int k = 0; k < chordPts.Count; k++)
                {
                    Point3d pt = chordPts[(startIdx + k) % chordPts.Count];
                    AddPointClean(path, pt);
                }

                AddPointClean(path, ringStart);

                currentPtRef = path[path.Count - 1];
            }

            Point3d NudgePerpToLastDirection(Point3d prevPt, Point3d lastCutPt, double dist)
            {
                if (dist <= 0.0) return lastCutPt;

                Vector3d dir = lastCutPt - prevPt;

                double dz = Vector3d.Multiply(dir, basePlane.Normal);
                dir -= basePlane.Normal * dz;

                if (!dir.Unitize())
                    return lastCutPt;

                Vector3d perp = Vector3d.CrossProduct(basePlane.Normal, dir);
                if (!perp.Unitize())
                    return lastCutPt;

                Point3d a = lastCutPt + perp * dist;
                Point3d b = lastCutPt - perp * dist;

                bool ina = ContainsPoint(C, a);
                bool inb = ContainsPoint(C, b);

                if (ina && !inb) return a;
                if (inb && !ina) return b;

                Point3d ctr = C.GetBoundingBox(true).Center;
                AreaMassProperties amp = AreaMassProperties.Compute(C, tol);
                if (amp != null) ctr = amp.Centroid;

                return a.DistanceTo(ctr) < b.DistanceTo(ctr) ? a : b;
            }

            double helixRadius = 0.0;
            double helixPitch = 0.0;
            bool applyAllDescents = true;
            double helixHeight = 0.0;
            bool helixEnabled = false;

            object eoRaw = EO;

            GH_ObjectWrapper goo = eoRaw as GH_ObjectWrapper;
            if (goo != null)
                eoRaw = goo.Value;

            if (eoRaw != null)
            {
                try
                {
                    Type t = eoRaw.GetType();

                    var fR = t.GetField("HR");
                    var fP = t.GetField("HP");
                    var fA = t.GetField("ALL");
                    var fH = t.GetField("HH");

                    if (fR != null && fR.GetValue(eoRaw) != null)
                        helixRadius = Convert.ToDouble(fR.GetValue(eoRaw));

                    if (fP != null && fP.GetValue(eoRaw) != null)
                        helixPitch = Convert.ToDouble(fP.GetValue(eoRaw));

                    if (fA != null && fA.GetValue(eoRaw) != null)
                        applyAllDescents = Convert.ToBoolean(fA.GetValue(eoRaw));

                    if (fH != null && fH.GetValue(eoRaw) != null)
                        helixHeight = Convert.ToDouble(fH.GetValue(eoRaw));
                }
                catch
                {
                    helixEnabled = false;
                    helixRadius = 0.0;
                    helixPitch = 0.0;
                    applyAllDescents = true;
                    helixHeight = 0.0;
                }
            }

            if (helixRadius < 0.0) helixRadius = 0.0;
            if (helixPitch < 0.0) helixPitch = 0.0;
            if (helixHeight < 0.0) helixHeight = 0.0;

            helixEnabled = helixRadius > eps && helixPitch > eps;

            void AppendHelixSegment(List<Point3d> dst, Point3d from, Point3d to, Vector3d zAxis)
            {
                Vector3d z = zAxis;
                if (!z.Unitize())
                {
                    dst.Add(to);
                    return;
                }

                Vector3d v = to - from;
                double dz = Vector3d.Multiply(v, z);
                double drop = -dz;
                if (drop <= eps)
                {
                    dst.Add(to);
                    return;
                }

                double turns = drop / helixPitch;
                if (turns < 0.05)
                {
                    dst.Add(to);
                    return;
                }

                Vector3d x = fixedX;
                if (!x.Unitize()) x = Vector3d.XAxis;

                Vector3d y = fixedY;
                if (!y.Unitize()) y = Vector3d.YAxis;

                int segs = (int)Math.Ceiling(Math.Max(6.0, turns * 8.0));
                if (segs > 20000) segs = 20000;

                for (int i = 1; i <= segs; i++)
                {
                    double t = (double)i / segs;
                    Point3d pt = from + (to - from) * t;

                    double ang = 2.0 * Math.PI * turns * t;

                    double r = helixRadius;
                    if (i == 1 || i == segs) r = 0.0;

                    pt = pt + x * (r * Math.Cos(ang)) + y * (r * Math.Sin(ang));
                    dst.Add(pt);
                }

                double endTol = Math.Max(10.0 * tol, 1e-6);
                if (dst.Count == 0 || dst[dst.Count - 1].DistanceTo(to) > endTol)
                    dst.Add(to);
            }

            void AppendEntryDescent(List<Point3d> dst, Point3d fromSafe, Point3d toCut, Vector3d zAxis)
            {
                Vector3d z = zAxis;
                if (!z.Unitize())
                {
                    dst.Add(toCut);
                    return;
                }

                double totalDrop = -Vector3d.Multiply(toCut - fromSafe, z);
                if (totalDrop <= eps)
                {
                    dst.Add(toCut);
                    return;
                }

                double h = helixHeight;
                if (h <= eps || h >= totalDrop)
                    h = totalDrop;

                double lead = totalDrop - h;

                Point3d pStart = fromSafe;
                if (lead > eps)
                {
                    pStart = fromSafe - z * lead;
                    dst.Add(pStart);
                }

                Point3d pEndHelix = pStart;
                if (h > eps)
                {
                    pEndHelix = pStart - z * h;
                    AppendHelixSegment(dst, pStart, pEndHelix, z);
                }

                if (pEndHelix.DistanceTo(toCut) > eps)
                    dst.Add(toCut);
            }

            double toolRadius = 0.5 * BD;

            var level0 = new List<Curve> { C.DuplicateCurve() };
            List<Curve> finishCurves = toolRadius > tol ? AllInwardOffsets(level0, toolRadius) : level0;
            if (finishCurves == null || finishCurves.Count == 0) return;

            var levels = new List<List<Curve>>();
            levels.Add(finishCurves);

            const int maxLevels = 10000;
            for (int i = 0; i < maxLevels; i++)
            {
                List<Curve> next = AllInwardOffsets(levels[levels.Count - 1], STO);
                if (next == null || next.Count == 0) break;
                levels.Add(next);
            }

            int deepest = levels.Count - 1;
            if (deepest < 0) return;

            var parentIndex = new List<int[]>();
            parentIndex.Add(new int[levels[0].Count]);

            var hasChild = new List<bool[]>();
            for (int li = 0; li <= deepest; li++)
                hasChild.Add(new bool[levels[li].Count]);

            for (int li = 1; li <= deepest; li++)
            {
                int n = levels[li].Count;
                int[] pIdx = new int[n];
                for (int i = 0; i < n; i++) pIdx[i] = -1;

                List<Curve> parents = levels[li - 1];

                for (int ci = 0; ci < n; ci++)
                {
                    Curve child = levels[li][ci];
                    if (child == null || !child.IsValid || !child.IsClosed) continue;

                    Point3d probe = ProbePointOnCurve(child);

                    int best = -1;
                    double bestArea = double.PositiveInfinity;

                    for (int pi = 0; pi < parents.Count; pi++)
                    {
                        Curve par = parents[pi];
                        if (par == null || !par.IsValid || !par.IsClosed) continue;
                        if (!ContainsPoint(par, probe)) continue;

                        double a = SafeArea(par);
                        if (a < bestArea)
                        {
                            bestArea = a;
                            best = pi;
                        }
                    }

                    pIdx[ci] = best;
                    if (best >= 0) hasChild[li - 1][best] = true;
                }

                parentIndex.Add(pIdx);
            }

            var leaves = new List<NodeKey>();
            for (int li = 0; li <= deepest; li++)
            {
                for (int i = 0; i < levels[li].Count; i++)
                {
                    if (!hasChild[li][i])
                        leaves.Add(new NodeKey(li, i));
                }
            }

            if (leaves.Count == 0) return;

            leaves.Sort((a, b) =>
            {
                Point3d pa = levels[a.L][a.I].GetBoundingBox(true).Center;
                Point3d pb = levels[b.L][b.I].GetBoundingBox(true).Center;
                int cx = pa.X.CompareTo(pb.X);
                if (cx != 0) return cx;
                return pa.Y.CompareTo(pb.Y);
            });

            List<NodeKey> BuildChain(NodeKey leaf)
            {
                var chain = new List<NodeKey>();
                int li = leaf.L;
                int idx = leaf.I;

                while (li >= 0 && idx >= 0)
                {
                    chain.Add(new NodeKey(li, idx));
                    if (li == 0) break;
                    idx = parentIndex[li][idx];
                    li--;
                }

                return chain;
            }

            var chains = new List<List<NodeKey>>();
            foreach (NodeKey leaf in leaves)
                chains.Add(BuildChain(leaf));

            var countLeaves = new Dictionary<NodeKey, int>();
            foreach (List<NodeKey> chain in chains)
            {
                foreach (NodeKey nk in chain)
                {
                    int v;
                    countLeaves.TryGetValue(nk, out v);
                    countLeaves[nk] = v + 1;
                }
            }

            var processedLeaf = new HashSet<NodeKey>();
            var cutRings = new HashSet<NodeKey>();

            bool NodeReady(NodeKey node)
            {
                int needed = countLeaves[node];

                int have = 0;
                for (int i = 0; i < leaves.Count; i++)
                {
                    if (!processedLeaf.Contains(leaves[i])) continue;
                    List<NodeKey> chain = chains[i];
                    for (int k = 0; k < chain.Count; k++)
                    {
                        if (chain[k].Equals(node))
                        {
                            have++;
                            break;
                        }
                    }
                }

                return have >= needed;
            }

            var multiNodes = new List<NodeKey>();
            foreach (KeyValuePair<NodeKey, int> kv in countLeaves)
            {
                if (kv.Value > 1)
                    multiNodes.Add(kv.Key);
            }

            multiNodes.Sort((a, b) =>
            {
                int dl = b.L.CompareTo(a.L);
                if (dl != 0) return dl;
                Point3d pa = levels[a.L][a.I].GetBoundingBox(true).Center;
                Point3d pb = levels[b.L][b.I].GetBoundingBox(true).Center;
                int cx = pa.X.CompareTo(pb.X);
                if (cx != 0) return cx;
                return pa.Y.CompareTo(pb.Y);
            });

            var masterPts = new List<Point3d>();
            var jumpPoints = new List<Point3d>();

            void AddJumpPoint(Point3d pt)
            {
                if (jumpPoints.Count == 0 || jumpPoints[jumpPoints.Count - 1].DistanceTo(pt) > eps)
                    jumpPoints.Add(pt);
            }

            Point3d currentPt = Point3d.Unset;

            void CutReadyMultiRings()
            {
                foreach (NodeKey nk in multiNodes)
                {
                    if (cutRings.Contains(nk)) continue;
                    if (!NodeReady(nk)) continue;

                    Curve loop = levels[nk.L][nk.I];
                    if (loop == null || !loop.IsValid || !loop.IsClosed) continue;

                    CurveOrientation wantOri = loop.ClosedCurveOrientation(basePlane);
                    if (wantOri == CurveOrientation.Undefined) wantOri = CurveOrientation.CounterClockwise;

                    double chordErr = ChordForNodeLevel(nk.L);
                    AppendRing(masterPts, loop, wantOri, chordErr, ref currentPt);
                    cutRings.Add(nk);
                }
            }

            for (int islandIdx = 0; islandIdx < leaves.Count; islandIdx++)
            {
                NodeKey leaf = leaves[islandIdx];
                List<NodeKey> chain = chains[islandIdx];

                CurveOrientation wantOriIsland = CurveOrientation.CounterClockwise;
                for (int k = 0; k < chain.Count; k++)
                {
                    NodeKey nk = chain[k];
                    Curve loop = levels[nk.L][nk.I];
                    if (loop == null || !loop.IsValid || !loop.IsClosed) continue;
                    wantOriIsland = loop.ClosedCurveOrientation(basePlane);
                    if (wantOriIsland == CurveOrientation.Undefined) wantOriIsland = CurveOrientation.CounterClockwise;
                    break;
                }

                bool pendingIslandLift = islandIdx > 0;

                for (int k = 0; k < chain.Count; k++)
                {
                    NodeKey nk = chain[k];

                    int ccount = 0;
                    countLeaves.TryGetValue(nk, out ccount);

                    if (ccount != 1) continue;
                    if (cutRings.Contains(nk)) continue;

                    Curve loop = levels[nk.L][nk.I];
                    if (loop == null || !loop.IsValid || !loop.IsClosed) continue;

                    double chordErr = ChordForNodeLevel(nk.L);

                    if (pendingIslandLift && masterPts.Count > 0 && LHT > 0.0)
                    {
                        Point3d upPt = LiftPoint(masterPts[masterPts.Count - 1]);
                        AddPointClean(masterPts, upPt);
                        AddJumpPoint(upPt);

                        Point3d near = currentPt.IsValid ? currentPt : loop.GetBoundingBox(true).Center;
                        Point3d seamPt = GetLoopSeamPoint(loop, wantOriIsland, chordErr, near);

                        Point3d overPt = LiftPoint(seamPt);
                        AddPointClean(masterPts, overPt);
                        AddJumpPoint(overPt);

                        currentPt = seamPt;
                        pendingIslandLift = false;
                    }

                    AppendRing(masterPts, loop, wantOriIsland, chordErr, ref currentPt);
                    cutRings.Add(nk);
                }

                processedLeaf.Add(leaf);
                CutReadyMultiRings();
            }

            CutReadyMultiRings();

            if (masterPts.Count >= 2 && SE > 0.0)
            {
                Point3d prevPt = masterPts[masterPts.Count - 2];
                Point3d lastCutPt = masterPts[masterPts.Count - 1];
                Point3d nudgePt = NudgePerpToLastDirection(prevPt, lastCutPt, SE);
                AddPointClean(masterPts, nudgePt);
            }

            if (helixEnabled && LHT > 0.0 && masterPts.Count >= 1)
            {
                Vector3d z = SafeZ();
                if (z.Unitize())
                {
                    var rebuilt = new List<Point3d>(masterPts.Count + 1024);

                    Point3d firstCut = masterPts[0];
                    Point3d firstSafe = LiftPoint(firstCut);
                    rebuilt.Add(firstSafe);
                    AppendEntryDescent(rebuilt, firstSafe, firstCut, z);

                    bool didOne = !applyAllDescents;

                    for (int i = 1; i < masterPts.Count; i++)
                    {
                        Point3d p0 = rebuilt[rebuilt.Count - 1];
                        Point3d p1 = masterPts[i];

                        Vector3d v = p1 - p0;
                        double dz = Vector3d.Multiply(v, z);
                        Vector3d lateral = v - z * dz;

                        double drop = -dz;
                        bool isDescent = dz < -eps;
                        bool mostlyVertical = lateral.Length <= Math.Max(5.0 * eps, 0.25 * helixRadius);
                        bool isSafeDrop = Math.Abs(drop - LHT) <= Math.Max(10.0 * tol, 1e-3);

                        if (isDescent && mostlyVertical && isSafeDrop)
                        {
                            if (applyAllDescents)
                            {
                                AppendEntryDescent(rebuilt, p0, p1, z);
                                continue;
                            }
                            else if (!didOne)
                            {
                                AppendEntryDescent(rebuilt, p0, p1, z);
                                didOne = true;
                                continue;
                            }
                        }

                        rebuilt.Add(p1);
                    }

                    masterPts = rebuilt;
                }
            }

            if (masterPts.Count < 2)
                return;

            var jumpIndices = new List<int>();
            int searchStart = 0;
            for (int jp = 0; jp < jumpPoints.Count; jp++)
            {
                Point3d target = jumpPoints[jp];
                for (int i = searchStart; i < masterPts.Count; i++)
                {
                    if (masterPts[i].DistanceTo(target) <= eps)
                    {
                        if (jumpIndices.Count == 0 || jumpIndices[jumpIndices.Count - 1] != i)
                            jumpIndices.Add(i);
                        searchStart = i + 1;
                        break;
                    }
                }
            }

            var outPlanes = new List<Plane>(masterPts.Count);
            var outPts = new List<Point3d>(masterPts.Count);

            foreach (Point3d pt in masterPts)
            {
                Plane pl = new Plane(pt, fixedX, fixedY);
                pl.Rotate(PR, pl.ZAxis, pl.Origin);
                if (FLP) pl.Flip();
                outPlanes.Add(pl);
                outPts.Add(pl.Origin);
            }

            DA.SetDataList(0, outPlanes);
            DA.SetData(1, new PolylineCurve(new Polyline(outPts)));
            DA.SetDataList(2, jumpIndices);
        }

        public override GH_Exposure Exposure => GH_Exposure.primary;

        protected override System.Drawing.Bitmap Icon => Properties.Resources.PlanarOffset;

        public override Guid ComponentGuid => new Guid("3C1EADCF-3880-444A-945A-34859D24FD6C");

        private struct NodeKey : IEquatable<NodeKey>
        {
            public int L;
            public int I;

            public NodeKey(int l, int i)
            {
                L = l;
                I = i;
            }

            public bool Equals(NodeKey other)
            {
                return L == other.L && I == other.I;
            }

            public override bool Equals(object obj)
            {
                return obj is NodeKey other && Equals(other);
            }

            public override int GetHashCode()
            {
                unchecked
                {
                    return (L * 397) ^ I;
                }
            }

            public override string ToString()
            {
                return L + ":" + I;
            }
        }
    }
}