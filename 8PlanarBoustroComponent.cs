using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Grasshopper.Kernel.Parameters;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;

namespace Stickbug
{
    public class ScanlinePathComponent2 : GH_Component
    {
        private bool _useDegreesANG = false;
        private bool _useDegreesPR = false;

        public ScanlinePathComponent2()
          : base("Planar Boustrophedon", "PlanarB",
              "Use a planar bounding curve to create an orientable, scanline milling toolpath",
              "Stickbug", "Planar")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Curve", "C", "Closed planar bounding curve", GH_ParamAccess.item);
            pManager.AddNumberParameter("Step", "ST", "Step-over distance", GH_ParamAccess.item, 10.0);
            pManager.AddAngleParameter("Angle", "ANG", "Scan angle", GH_ParamAccess.item, 0.0);
            pManager.AddNumberParameter("Bit Diameter", "OFF", "Tool bit diameter", GH_ParamAccess.item, 10.0);
            pManager.AddNumberParameter("Extra Inset", "EIS", "Additional inset for scanlines", GH_ParamAccess.item, 5.0);
            pManager.AddNumberParameter("Lift Height", "LHT", "Safe lift height", GH_ParamAccess.item, 10.0);
            pManager.AddNumberParameter("Safe Exit", "SE", "Safe inward move at the end of the toolpath", GH_ParamAccess.item, 0.0);
            pManager.AddAngleParameter("Plane Rotation", "PR", "Rotate output planes", GH_ParamAccess.item, 0.0);
            pManager.AddBooleanParameter("Finish Pass", "AFP", "Add finish pass", GH_ParamAccess.item, true);
            pManager.AddBooleanParameter("Flip Safe Up", "FSU", "Flip safe-up direction", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("Flip Planes", "FLP", "Flip output planes", GH_ParamAccess.item, false);
            pManager.AddGenericParameter("Entry Options", "EO", "Optional helical entry options", GH_ParamAccess.item);
            pManager[11].Optional = true;
        }

        protected override void BeforeSolveInstance()
        {
            _useDegreesANG = false;
            _useDegreesPR = false;

            Param_Number angParam = Params.Input[2] as Param_Number;
            if (angParam != null)
                _useDegreesANG = angParam.UseDegrees;

            Param_Number prParam = Params.Input[7] as Param_Number;
            if (prParam != null)
                _useDegreesPR = prParam.UseDegrees;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddPlaneParameter("Planes", "PL", "Ordered toolpath planes", GH_ParamAccess.list);
            pManager.AddCurveParameter("Toolpath", "OC", "Output polyline curve through plane origins", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Jump Indices", "JI", "Indices of jump-up and jump-over points in the final plane list", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Curve C = null;
            double ST = 0.0;
            double ANG = 0.0;
            double OFF = 0.0;
            double EIS = 0.0;
            double LHT = 0.0;
            double SE = 0.0;
            double PR = 0.0;
            bool AFP = false;
            bool FSU = false;
            bool FLP = false;
            object EO = null;

            if (!DA.GetData(0, ref C)) return;
            if (!DA.GetData(1, ref ST)) return;
            if (!DA.GetData(2, ref ANG)) return;
            if (_useDegreesANG) ANG = RhinoMath.ToRadians(ANG);
            if (!DA.GetData(3, ref OFF)) return;
            if (!DA.GetData(4, ref EIS)) return;
            if (!DA.GetData(5, ref LHT)) return;
            if (!DA.GetData(6, ref SE)) return;
            if (!DA.GetData(7, ref PR)) return;
            if (_useDegreesPR) PR = RhinoMath.ToRadians(PR);
            if (!DA.GetData(8, ref AFP)) return;
            if (!DA.GetData(9, ref FSU)) return;
            if (!DA.GetData(10, ref FLP)) return;
            DA.GetData(11, ref EO);

            if (C == null || !C.IsValid || !C.IsClosed || ST <= 0.0)
                return;

            double tol = RhinoDoc.ActiveDoc != null ? RhinoDoc.ActiveDoc.ModelAbsoluteTolerance : 0.01;
            double eps = Math.Max(10.0 * tol, 1e-6);

            Plane basePlane;
            if (!C.TryGetPlane(out basePlane, tol))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Input curve must be planar");
                return;
            }

            CurveOrientation ori = C.ClosedCurveOrientation(basePlane);
            if (ori == CurveOrientation.Clockwise)
                basePlane.Flip();

            Vector3d zAxis = basePlane.Normal;
            if (!zAxis.Unitize())
                zAxis = Vector3d.ZAxis;

            Vector3d dir = basePlane.XAxis;
            dir.Rotate(ANG, zAxis);
            if (!dir.Unitize())
                dir = basePlane.XAxis;

            Vector3d perp = Vector3d.CrossProduct(zAxis, dir);
            if (!perp.Unitize())
                perp = basePlane.YAxis;

            Vector3d fixedX = dir;
            Vector3d fixedY = perp;

            Vector3d SafeZ()
            {
                Vector3d z = Vector3d.CrossProduct(fixedX, fixedY);
                if (!z.Unitize()) z = basePlane.Normal;
                if (!z.Unitize()) z = Vector3d.ZAxis;
                if (FSU) z = -z;
                return z;
            }

            Vector3d safeZ = SafeZ();

            Point3d LiftPoint(Point3d pt)
            {
                return pt + safeZ * LHT;
            }

            int AddPointClean(List<Point3d> path, Point3d pt)
            {
                if (path.Count == 0)
                {
                    path.Add(pt);
                    return 0;
                }

                if (path[path.Count - 1].DistanceTo(pt) <= eps)
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

                return path.Count - 1;
            }

            double DistancePointToSegment(Point3d pt, Point3d a, Point3d b)
            {
                Vector3d ab = b - a;
                double ab2 = ab.SquareLength;
                if (ab2 < 1e-18) return pt.DistanceTo(a);

                double tSeg = Vector3d.Multiply(pt - a, ab) / ab2;
                if (tSeg < 0.0) tSeg = 0.0;
                else if (tSeg > 1.0) tSeg = 1.0;

                Point3d q = a + tSeg * ab;
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

                void Recurse(double ta, double tb)
                {
                    if (safety++ > maxSegs)
                    {
                        tOut.Add(tb);
                        return;
                    }

                    double tm = 0.5 * (ta + tb);
                    Point3d pa = crv.PointAt(ta);
                    Point3d pb = crv.PointAt(tb);
                    Point3d pm = crv.PointAt(tm);

                    double dist = DistancePointToSegment(pm, pa, pb);

                    if (dist > maxErr && Math.Abs(tb - ta) > 1e-12)
                    {
                        Recurse(ta, tm);
                        Recurse(tm, tb);
                    }
                    else
                    {
                        tOut.Add(tb);
                    }
                }

                tOut.Add(t0);
                Recurse(t0, t1);
            }

            List<Point3d> RotateClosedPointListForStart(List<Point3d> pts, Point3d startNear)
            {
                if (pts == null || pts.Count == 0) return new List<Point3d>();

                int n = pts.Count;
                int best = 0;
                double bestD = double.PositiveInfinity;

                for (int i = 0; i < n; i++)
                {
                    double d = pts[i].DistanceTo(startNear);
                    if (d < bestD)
                    {
                        bestD = d;
                        best = i;
                    }
                }

                var fwd = new List<Point3d>(n);
                for (int k = 0; k < n; k++)
                    fwd.Add(pts[(best + k) % n]);

                var rev = new List<Point3d>(n);
                for (int k = 0; k < n; k++)
                    rev.Add(pts[(best - k + n) % n]);

                if (n >= 2)
                {
                    double df = fwd[1].DistanceTo(startNear);
                    double dr = rev[1].DistanceTo(startNear);
                    return dr < df ? rev : fwd;
                }

                return fwd;
            }

            bool ContainsPoint(Curve boundary, Point3d pt)
            {
                PointContainment rel = boundary.Contains(pt, basePlane, tol);
                return rel == PointContainment.Inside || rel == PointContainment.Coincident;
            }

            bool SegmentInsideEnough(Curve boundary, Point3d a, Point3d b)
            {
                if (boundary == null) return true;
                if (a.DistanceTo(b) <= eps) return true;

                if (!ContainsPoint(boundary, a)) return false;
                if (!ContainsPoint(boundary, b)) return false;

                const int samples = 7;
                for (int i = 1; i <= samples; i++)
                {
                    double tSample = (double)i / (samples + 1);
                    Point3d p = a + (b - a) * tSample;
                    if (!ContainsPoint(boundary, p)) return false;
                }
                return true;
            }

            Point3d InsetPointInward(Curve boundary, Point3d pt, double dist)
            {
                if (boundary == null || dist <= eps) return pt;

                double tCurve;
                if (!boundary.ClosestPoint(pt, out tCurve)) return pt;

                Vector3d tan = boundary.TangentAt(tCurve);
                tan -= basePlane.Normal * Vector3d.Multiply(tan, basePlane.Normal);
                if (!tan.Unitize()) return pt;

                Vector3d n = Vector3d.CrossProduct(basePlane.Normal, tan);
                if (!n.Unitize()) return pt;

                Point3d a = pt + n * dist;
                Point3d b = pt - n * dist;

                bool ina = ContainsPoint(boundary, a);
                bool inb = ContainsPoint(boundary, b);

                if (ina && !inb) return a;
                if (inb && !ina) return b;

                Point3d ctr = boundary.GetBoundingBox(true).Center;
                AreaMassProperties amp = AreaMassProperties.Compute(boundary, tol);
                if (amp != null) ctr = amp.Centroid;

                return a.DistanceTo(ctr) < b.DistanceTo(ctr) ? a : b;
            }

            double helixRadius = 0.0;
            double helixPitch = 0.0;
            bool applyAllDescents = false;
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
                    Type tObj = eoRaw.GetType();

                    var fR = tObj.GetField("HR");
                    var fP = tObj.GetField("HP");
                    var fA = tObj.GetField("ALL");
                    var fH = tObj.GetField("HH");

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
                }
            }

            if (helixRadius < 0.0) helixRadius = 0.0;
            if (helixPitch < 0.0) helixPitch = 0.0;
            if (helixHeight < 0.0) helixHeight = 0.0;

            helixEnabled = helixRadius > eps && helixPitch > eps;

            void AppendHelixSegment(List<Point3d> dst, Point3d fromPt, Point3d toPt, Vector3d zAxisLocal)
            {
                Vector3d z = zAxisLocal;
                if (!z.Unitize())
                {
                    AddPointClean(dst, toPt);
                    return;
                }

                Vector3d v = toPt - fromPt;
                double dz = Vector3d.Multiply(v, z);
                double drop = -dz;
                if (drop <= eps)
                {
                    AddPointClean(dst, toPt);
                    return;
                }

                double turns = drop / helixPitch;
                if (turns < 0.05)
                {
                    AddPointClean(dst, toPt);
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
                    double tHelix = (double)i / segs;
                    Point3d p = fromPt + (toPt - fromPt) * tHelix;

                    double ang = 2.0 * Math.PI * turns * tHelix;

                    double r = helixRadius;
                    if (i == 1 || i == segs) r = 0.0;

                    p = p + x * (r * Math.Cos(ang)) + y * (r * Math.Sin(ang));
                    AddPointClean(dst, p);
                }

                AddPointClean(dst, toPt);
            }

            void AppendEntryDescent(List<Point3d> dst, Point3d fromSafe, Point3d toInsetTarget, Point3d toFinal, Vector3d zAxisLocal)
            {
                Vector3d z = zAxisLocal;
                if (!z.Unitize())
                {
                    AddPointClean(dst, toFinal);
                    return;
                }

                double totalDrop = -Vector3d.Multiply(toInsetTarget - fromSafe, z);
                if (totalDrop <= eps)
                {
                    AddPointClean(dst, toFinal);
                    return;
                }

                double h = helixHeight;
                if (h <= eps || h >= totalDrop) h = totalDrop;

                double lead = totalDrop - h;

                Point3d helixStart = fromSafe;
                if (lead > eps)
                {
                    helixStart = fromSafe - z * lead;
                    AddPointClean(dst, helixStart);
                }

                Point3d helixEnd = helixStart - z * h;

                if (helixEnabled && h > eps) AppendHelixSegment(dst, helixStart, helixEnd, z);
                else AddPointClean(dst, helixEnd);

                AddPointClean(dst, toInsetTarget);
                AddPointClean(dst, toFinal);
            }

            Curve PickInwardOffset(Curve current, double dist)
            {
                if (current == null || !current.IsValid || !current.IsClosed) return null;

                double curArea = 0.0;
                AreaMassProperties curAmp = AreaMassProperties.Compute(current, tol);
                if (curAmp != null) curArea = Math.Abs(curAmp.Area);

                Curve best = null;
                double bestArea = double.NegativeInfinity;

                void Consider(Curve[] arr)
                {
                    if (arr == null) return;
                    foreach (Curve cc in arr)
                    {
                        if (cc == null || !cc.IsValid || !cc.IsClosed) continue;
                        AreaMassProperties amp = AreaMassProperties.Compute(cc, tol);
                        if (amp == null) continue;

                        double a = Math.Abs(amp.Area);
                        if (a >= curArea - 1e-9) continue;

                        if (a > bestArea)
                        {
                            bestArea = a;
                            best = cc;
                        }
                    }
                }

                Consider(current.Offset(basePlane, dist, tol, CurveOffsetCornerStyle.Sharp));
                Consider(current.Offset(basePlane, -dist, tol, CurveOffsetCornerStyle.Sharp));

                return best;
            }

            Curve boundaryMain = C.DuplicateCurve();
            double insetFinishBase = 0.5 * Math.Max(0.0, OFF);
            double insetMain = insetFinishBase + Math.Max(0.0, EIS);

            if (insetMain > tol)
            {
                Curve off = PickInwardOffset(boundaryMain, insetMain);
                if (off != null) boundaryMain = off;
            }

            Curve boundaryFinish = C.DuplicateCurve();
            if (insetFinishBase > tol)
            {
                Curve offF = PickInwardOffset(boundaryFinish, insetFinishBase);
                if (offF != null) boundaryFinish = offF;
            }

            Plane scanPlane = new Plane(basePlane.Origin, dir, perp);
            Transform worldToScan = Transform.PlaneToPlane(scanPlane, Plane.WorldXY);
            Transform scanToWorld = Transform.PlaneToPlane(Plane.WorldXY, scanPlane);

            Point3d ScanToWorldPoint(double x, double y)
            {
                Point3d p = new Point3d(x, y, 0.0);
                p.Transform(scanToWorld);
                return p;
            }

            bool WorldToScanXY(Point3d worldPt, out double x, out double y)
            {
                Point3d scanPt = worldPt;
                scanPt.Transform(worldToScan);
                x = scanPt.X;
                y = scanPt.Y;
                return scanPt.IsValid;
            }

            List<ScanSeg> BuildScanSegments(Curve boundary)
            {
                var builtSegs = new List<ScanSeg>();
                if (boundary == null || !boundary.IsValid || !boundary.IsClosed) return builtSegs;

                Curve b2d = boundary.DuplicateCurve();
                b2d.Transform(worldToScan);

                BoundingBox bb = b2d.GetBoundingBox(true);
                if (!bb.IsValid) return builtSegs;

                double minY = bb.Min.Y;
                double maxY = bb.Max.Y;

                double pad = ST * 2.0;
                minY -= pad;
                maxY += pad;

                double minX = bb.Min.X - pad;
                double maxX = bb.Max.X + pad;

                int maxRows = (int)Math.Ceiling((maxY - minY) / ST) + 3;
                if (maxRows < 1) maxRows = 1;

                int id = 0;

                for (int r = 0; r < maxRows; r++)
                {
                    double y = minY + r * ST;
                    if (y > maxY + tol) break;

                    Point3d s0 = new Point3d(minX, y, 0.0);
                    Point3d s1 = new Point3d(maxX, y, 0.0);

                    LineCurve scan = new LineCurve(new Line(s0, s1));
                    scan.Transform(scanToWorld);

                    CurveIntersections xEvents = Intersection.CurveCurve(scan, boundary, tol, tol);
                    if (xEvents == null || xEvents.Count < 2) continue;

                    var tHits = new List<double>();
                    foreach (IntersectionEvent ev in xEvents)
                        if (ev.IsPoint) tHits.Add(ev.ParameterA);

                    if (tHits.Count < 2) continue;
                    tHits.Sort();

                    for (int k = 0; k + 1 < tHits.Count; k += 2)
                    {
                        double t0 = tHits[k];
                        double t1 = tHits[k + 1];
                        if (t1 - t0 <= tol) continue;

                        Point3d p0w = scan.PointAt(t0);
                        Point3d p1w = scan.PointAt(t1);

                        Point3d p0s = p0w; p0s.Transform(worldToScan);
                        Point3d p1s = p1w; p1s.Transform(worldToScan);

                        double x0 = Math.Min(p0s.X, p1s.X);
                        double x1 = Math.Max(p0s.X, p1s.X);

                        Point3d pLw = ScanToWorldPoint(x0, y);
                        Point3d pRw = ScanToWorldPoint(x1, y);

                        builtSegs.Add(new ScanSeg
                        {
                            Id = id++,
                            Row = r,
                            Y = y,
                            X0 = x0,
                            X1 = x1,
                            P0w = pLw,
                            P1w = pRw
                        });
                    }
                }

                return builtSegs;
            }

            void BuildAdjacency(
                List<ScanSeg> segList,
                out Dictionary<int, List<int>> upMap,
                out Dictionary<EdgeKey, Overlap> overlapMap,
                out Dictionary<int, List<int>> rowMap)
            {
                upMap = new Dictionary<int, List<int>>();
                overlapMap = new Dictionary<EdgeKey, Overlap>();
                rowMap = new Dictionary<int, List<int>>();

                for (int i = 0; i < segList.Count; i++)
                {
                    upMap[i] = new List<int>();

                    List<int> list;
                    if (!rowMap.TryGetValue(segList[i].Row, out list))
                    {
                        list = new List<int>();
                        rowMap[segList[i].Row] = list;
                    }
                    list.Add(i);
                }

                double overlapTol = Math.Max(10.0 * tol, 0.35 * ST);

                foreach (KeyValuePair<int, List<int>> kv in rowMap)
                {
                    int row = kv.Key;
                    List<int> rowA = kv.Value;

                    List<int> rowB;
                    if (!rowMap.TryGetValue(row + 1, out rowB)) continue;

                    for (int ia = 0; ia < rowA.Count; ia++)
                    {
                        int a = rowA[ia];
                        ScanSeg sa = segList[a];

                        for (int ib = 0; ib < rowB.Count; ib++)
                        {
                            int b = rowB[ib];
                            ScanSeg sb = segList[b];

                            double loE = Math.Max(sa.X0, sb.X0) - overlapTol;
                            double hiE = Math.Min(sa.X1, sb.X1) + overlapTol;

                            if (hiE >= loE)
                            {
                                upMap[a].Add(b);

                                double lo = Math.Max(sa.X0, sb.X0);
                                double hi = Math.Min(sa.X1, sb.X1);
                                EdgeKey key = new EdgeKey(a, b);

                                if (!overlapMap.ContainsKey(key))
                                    overlapMap[key] = new Overlap(lo, hi);
                            }
                        }
                    }
                }

                foreach (KeyValuePair<int, List<int>> kv in rowMap)
                    kv.Value.Sort((i, j) => segList[i].X0.CompareTo(segList[j].X0));
            }

            List<ScanSeg> allScanSegs = BuildScanSegments(boundaryMain);
            if (allScanSegs == null || allScanSegs.Count == 0)
                return;

            Dictionary<int, List<int>> upMapOuter;
            Dictionary<int, List<int>> rowMapOuter;
            Dictionary<EdgeKey, Overlap> overlapMapOuter;
            BuildAdjacency(allScanSegs, out upMapOuter, out overlapMapOuter, out rowMapOuter);

            var masterPts = new List<Point3d>(4096);
            var jumpIndices = new List<int>();

            void AddJumpIndex(int idx)
            {
                if (idx < 0) return;
                if (jumpIndices.Count == 0 || jumpIndices[jumpIndices.Count - 1] != idx)
                    jumpIndices.Add(idx);
            }

            bool RowLeftToRight(int row)
            {
                return (row % 2) == 0;
            }

            Point3d SegStart(int segIdx)
            {
                bool l2r = RowLeftToRight(allScanSegs[segIdx].Row);
                return l2r ? allScanSegs[segIdx].P0w : allScanSegs[segIdx].P1w;
            }

            Point3d SegEnd(int segIdx)
            {
                bool l2r = RowLeftToRight(allScanSegs[segIdx].Row);
                return l2r ? allScanSegs[segIdx].P1w : allScanSegs[segIdx].P0w;
            }

            double CurScanX(Point3d curW)
            {
                double x, y;
                if (WorldToScanXY(curW, out x, out y)) return x;
                return 0.0;
            }

            bool didOneNonInitialHelix = false;

            void SafeJumpTo(Point3d fromPt, Point3d toCut, bool allowHelix, bool forceHelix = false)
            {
                if (LHT <= eps)
                {
                    AddPointClean(masterPts, toCut);
                    return;
                }

                Point3d insetTarget = allowHelix ? InsetPointInward(boundaryMain, toCut, helixRadius) : toCut;

                Point3d fromSafe = LiftPoint(fromPt);
                Point3d toSafe = LiftPoint(insetTarget);

                int upIdx = AddPointClean(masterPts, fromSafe);
                AddJumpIndex(upIdx);

                int overIdx = AddPointClean(masterPts, toSafe);
                AddJumpIndex(overIdx);

                bool doHelix = allowHelix && helixEnabled && (forceHelix || applyAllDescents || !didOneNonInitialHelix);

                if (doHelix)
                {
                    AppendEntryDescent(masterPts, toSafe, insetTarget, toCut, safeZ);
                    didOneNonInitialHelix = true;
                }
                else
                {
                    AddPointClean(masterPts, insetTarget);
                    AddPointClean(masterPts, toCut);
                }
            }

            void ConnectOnPlane(ref Point3d activeWorldPt, Point3d nextWorldPt)
            {
                if (boundaryFinish != null && boundaryFinish.IsValid && boundaryFinish.IsClosed)
                {
                    if (!SegmentInsideEnough(boundaryFinish, activeWorldPt, nextWorldPt))
                    {
                        bool allow = applyAllDescents;
                        SafeJumpTo(activeWorldPt, nextWorldPt, allow, allow);
                        activeWorldPt = nextWorldPt;
                        return;
                    }
                }

                AddPointClean(masterPts, nextWorldPt);
                activeWorldPt = nextWorldPt;
            }

            void CutSegAtomic(ref Point3d activeWorldPt, int segIndex)
            {
                Point3d s = SegStart(segIndex);
                Point3d e = SegEnd(segIndex);

                if (activeWorldPt.DistanceTo(s) > eps)
                    ConnectOnPlane(ref activeWorldPt, s);

                AddPointClean(masterPts, e);
                activeWorldPt = e;
            }

            int ChooseBestUpAndDefer(int currentSegIndex, double xNow, HashSet<int> visitedSet, List<int> deferredList)
            {
                List<int> candidates = upMapOuter[currentSegIndex];
                if (candidates == null || candidates.Count == 0) return -1;

                var scored = new List<Tuple<double, int>>(candidates.Count);

                for (int i = 0; i < candidates.Count; i++)
                {
                    int v = candidates[i];
                    if (visitedSet.Contains(v)) continue;

                    Overlap ov;
                    if (!overlapMapOuter.TryGetValue(new EdgeKey(currentSegIndex, v), out ov))
                        ov = new Overlap(xNow, xNow);

                    double score;
                    if (xNow < ov.Lo) score = ov.Lo - xNow;
                    else if (xNow > ov.Hi) score = xNow - ov.Hi;
                    else score = 0.0;

                    scored.Add(Tuple.Create(score, v));
                }

                if (scored.Count == 0) return -1;

                scored.Sort((a, b) =>
                {
                    int c = a.Item1.CompareTo(b.Item1);
                    if (c != 0) return c;
                    return allScanSegs[a.Item2].X0.CompareTo(allScanSegs[b.Item2].X0);
                });

                int best = scored[0].Item2;

                for (int i = 1; i < scored.Count; i++)
                    deferredList.Add(scored[i].Item2);

                return best;
            }

            int PopBestDeferred(List<int> deferredList, HashSet<int> visitedSet, int currentRow)
            {
                int bestIdx = -1;
                int bestAbs = int.MaxValue;
                int bestRow = int.MinValue;

                for (int i = 0; i < deferredList.Count; i++)
                {
                    int s = deferredList[i];
                    if (visitedSet.Contains(s)) continue;

                    int abs = Math.Abs(allScanSegs[s].Row - currentRow);
                    if (abs < bestAbs || (abs == bestAbs && allScanSegs[s].Row > bestRow))
                    {
                        bestAbs = abs;
                        bestRow = allScanSegs[s].Row;
                        bestIdx = i;
                    }
                }

                if (bestIdx < 0) return -1;

                int best = deferredList[bestIdx];
                deferredList.RemoveAt(bestIdx);
                return best;
            }

            int PickNextSeed(HashSet<int> visitedSet)
            {
                int best = -1;
                int bestRow = int.MaxValue;
                double bestX = double.PositiveInfinity;

                foreach (KeyValuePair<int, List<int>> kv in rowMapOuter)
                {
                    int row = kv.Key;
                    foreach (int s in kv.Value)
                    {
                        if (visitedSet.Contains(s)) continue;
                        if (row < bestRow || (row == bestRow && allScanSegs[s].X0 < bestX))
                        {
                            bestRow = row;
                            bestX = allScanSegs[s].X0;
                            best = s;
                        }
                    }
                }
                return best;
            }

            int startSeed = PickNextSeed(new HashSet<int>());
            if (startSeed < 0)
                return;

            Point3d seedStart = SegStart(startSeed);

            if (LHT > eps)
            {
                Point3d firstInset = InsetPointInward(boundaryMain, seedStart, helixRadius);
                Point3d firstSafe = LiftPoint(firstInset);
                AddPointClean(masterPts, firstSafe);

                if (helixEnabled) AppendEntryDescent(masterPts, firstSafe, firstInset, seedStart, safeZ);
                else AddPointClean(masterPts, seedStart);

                didOneNonInitialHelix = true;
            }
            else
            {
                AddPointClean(masterPts, seedStart);
            }

            Point3d activeWorldPtOuter = masterPts[masterPts.Count - 1];
            int currentSegIndexOuter = startSeed;

            var visitedSeg = new HashSet<int>();
            var deferredSegsOuter = new List<int>();

            while (true)
            {
                if (!visitedSeg.Contains(currentSegIndexOuter))
                {
                    CutSegAtomic(ref activeWorldPtOuter, currentSegIndexOuter);
                    visitedSeg.Add(currentSegIndexOuter);
                }

                double xNow = CurScanX(activeWorldPtOuter);
                int next = ChooseBestUpAndDefer(currentSegIndexOuter, xNow, visitedSeg, deferredSegsOuter);

                if (next != -1)
                {
                    Point3d nextStart = SegStart(next);
                    ConnectOnPlane(ref activeWorldPtOuter, nextStart);
                    currentSegIndexOuter = next;
                    continue;
                }

                int jumpSeg = PopBestDeferred(deferredSegsOuter, visitedSeg, allScanSegs[currentSegIndexOuter].Row);

                if (jumpSeg == -1)
                {
                    jumpSeg = PickNextSeed(visitedSeg);
                    if (jumpSeg == -1) break;
                }

                Point3d jumpStart = SegStart(jumpSeg);
                SafeJumpTo(activeWorldPtOuter, jumpStart, true);
                AddPointClean(masterPts, jumpStart);
                activeWorldPtOuter = jumpStart;
                currentSegIndexOuter = jumpSeg;
            }

            if (AFP)
            {
                if (boundaryFinish != null && boundaryFinish.IsValid && boundaryFinish.IsClosed)
                {
                    const double chordErr = 0.02;
                    var tSamples = new List<double>();
                    SampleCurveChordError(boundaryFinish, chordErr, tSamples);

                    var finishPts = new List<Point3d>(tSamples.Count);
                    Point3d prevPt = Point3d.Unset;

                    foreach (double t in tSamples)
                    {
                        Point3d pt = boundaryFinish.PointAt(t);
                        if (!prevPt.IsValid || pt.DistanceTo(prevPt) > 1e-9)
                            finishPts.Add(pt);
                        prevPt = pt;
                    }

                    if (finishPts.Count > 2 && finishPts[0].DistanceTo(finishPts[finishPts.Count - 1]) < 1e-9)
                        finishPts.RemoveAt(finishPts.Count - 1);

                    if (finishPts.Count >= 2)
                    {
                        Point3d startNear = masterPts.Count > 0 ? masterPts[masterPts.Count - 1] : finishPts[0];
                        List<Point3d> rotated = RotateClosedPointListForStart(finishPts, startNear);

                        masterPts.AddRange(rotated);

                        Point3d firstFinish = rotated[0];
                        Point3d lastNow = masterPts[masterPts.Count - 1];
                        if (lastNow.DistanceTo(firstFinish) > 1e-9)
                            masterPts.Add(firstFinish);
                    }
                }
            }

            if (SE > eps && masterPts.Count > 0)
            {
                Curve useBoundary = (boundaryFinish != null && boundaryFinish.IsValid && boundaryFinish.IsClosed)
                    ? boundaryFinish
                    : boundaryMain;

                Point3d lastToolPt = masterPts[masterPts.Count - 1];
                Point3d inset = InsetPointInward(useBoundary, lastToolPt, SE);
                AddPointClean(masterPts, inset);
            }

            if (masterPts.Count < 2)
                return;

            var outPlanes = new List<Plane>(masterPts.Count);
            var outPts = new List<Point3d>(masterPts.Count);

            for (int i = 0; i < masterPts.Count; i++)
            {
                Point3d pt = masterPts[i];
                Plane pl = new Plane(pt, fixedX, fixedY);
                pl.Rotate(PR, pl.ZAxis, pl.Origin);
                if (FLP) pl.Flip();
                outPlanes.Add(pl);
                outPts.Add(pl.Origin);
            }

            Polyline pline = new Polyline(outPts);
            if (pline.IsClosed)
                pline.RemoveAt(pline.Count - 1);

            DA.SetDataList(0, outPlanes);
            DA.SetData(1, new PolylineCurve(pline));
            DA.SetDataList(2, jumpIndices);
        }

        public override GH_Exposure Exposure => GH_Exposure.primary;

        protected override System.Drawing.Bitmap Icon => Properties.Resources.PlanarBoustro;

        public override Guid ComponentGuid => new Guid("4B6C6A96-187C-4891-8959-061D929A6EFE");

        private struct ScanSeg
        {
            public int Id;
            public int Row;
            public double Y;
            public double X0;
            public double X1;
            public Point3d P0w;
            public Point3d P1w;
        }

        private struct EdgeKey : IEquatable<EdgeKey>
        {
            public int A;
            public int B;

            public EdgeKey(int a, int b)
            {
                if (a < b) { A = a; B = b; }
                else { A = b; B = a; }
            }

            public bool Equals(EdgeKey other)
            {
                return A == other.A && B == other.B;
            }

            public override bool Equals(object obj)
            {
                return obj is EdgeKey other && Equals(other);
            }

            public override int GetHashCode()
            {
                unchecked { return (A * 397) ^ B; }
            }
        }

        private struct Overlap
        {
            public double Lo;
            public double Hi;

            public Overlap(double lo, double hi)
            {
                Lo = lo;
                Hi = hi;
            }
        }
    }
}