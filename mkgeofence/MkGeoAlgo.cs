using System;
using System.Collections.Generic;

namespace MkGeoFence
{
    public class MkGeoAlgo {

        internal static bool DEBUG = true;
        internal static Logger geoFenceLogger = null;
        public static void setLogger(Logger logger) {
            geoFenceLogger = logger;
        }

        public static double earthRadius = 3958.8 * 1609.344; // in meters
        static double deg2arc = Math.PI * earthRadius / 180.0;

        static public double meter2deg(double m) {
            return m / deg2arc;
        }

        static public double deg2meter(double d) {
            return deg2arc * d;
        }

        internal static Vertex vector(Vertex from, Vertex to) {
            return new Vertex(to.x - from.x, to.y - from.y);
        }

        internal static double norm(Vertex v) {
            double x = Math.Abs(v.x);
            double y = Math.Abs(v.y);
            if (x < y) {
                double tmp = x;
                x = y;
                y = tmp;
            }
            // Assertion: x >= y, non-negative
            if (x == 0.0) {
                return 0.0;
            }
            double r = y / x;
            return x * Math.Sqrt(1.0 + r*r);
        }

        static double dist(Vertex u, Vertex v) {
            return norm(vector(u, v));
        }

        static double dot(Vertex u, Vertex v) {
            return u.x * v.x + u.y * v.y;
        }

        static double delta(Vertex v1, Vertex v2, Vertex v3) {
            return v1.x * v2.y + v2.x * v3.y + v3.x * v1.y - v3.x * v2.y - v2.x * v1.y - v1.x * v3.y;
        }

        // Signed distance of v to the line from v1 -> v2. Positive if v1, v2, vert, form right-hand circle.
        // Pre-condition: dist(v1, v2) > 0
        static double signedDist(Vertex v1, Vertex v2, Vertex vert) {
            return delta(v1, v2, vert) / dist(v1, v2);
        }

        static bool isInsideBoundingBox(Vertex bbMin, Vertex bbMax, Vertex vert, double margin) {
            if (!(margin < Double.MaxValue)) {
                return true;
            }
            if (vert.x < bbMin.x - margin || vert.x > bbMax.x + margin) {
                return false;
            }
            if (vert.y < bbMin.y - margin || vert.y > bbMax.y + margin) {
                return false;
            }
            return true;
        }

        // Update the border buffer with regard to edge from v1 to v2, and the joint at v1 with incoming edge from v0
        // and outgoing edge to v2
        // If (signed, negative inside and postive outside) distance is less than buffer in absolute sense, return
        // the new distance as the updated buffer
        // We will compute the new distance only if it would be less then maxBorderBuffer.

        // Post-condition: return == buffer || abs(return) < abs(buffer)

        static double updateBorderBuffer(double buffer, Vertex v0, Vertex v1, Vertex v2, Vertex vert, double epsilon) {
            Vertex e1 = vector(v1, v2);
            double v12_len = norm(e1);
            e1 = e1 / v12_len;
            Vertex v1_vert = vector(v1, vert);

            double vert_x = dot(e1, v1_vert);
            if (vert_x > v12_len + epsilon) {
                return buffer;
            }

            double upperBound = Math.Abs(buffer);

            Vertex e2 = new Vertex(-e1.y, e1.x);
            double vert_y = dot(e2, v1_vert);
            if (Math.Abs(vert_y) > upperBound) {
                return buffer;
            }

            double ret = buffer;
            
            if (vert_x >= 0) {
                // 0 <= vert_x <= e1_len
                if (Math.Abs(vert_y) < upperBound) {
                    ret = vert_y;
                }
                return ret;
            }

            // Dealing with the join of v0 -> v1 -> v2
            
            double v1_vert_len = norm(v1_vert);
            if (v1_vert_len > upperBound) {
                return ret;
            }

            Vertex v01 = vector(v0, v1);
            double v01x = dot(v01, e1);
            double v01y = dot(v01, e2);
            Vertex v02 = new Vertex(-v01y, v01x);
            double alpha = Math.Atan2(v02.y, v02.x);
            
            double half_PI = Math.PI / 2.0;
            double two_PI = Math.PI + Math.PI;
            double threeHalfs_PI = Math.PI + half_PI;
            
            if (alpha < 0.0) {
                alpha = alpha + two_PI;
            }
            // alpha goes from 0 to 2π   
            
            double theta = Math.Atan2(vert_y, vert_x);
            if (theta < 0.0) {
                theta = theta + two_PI;
            }

            if (half_PI < alpha && alpha < threeHalfs_PI) {
                if (half_PI < theta && theta < alpha) {
                    ret = v1_vert_len;
                }
            } else {
                alpha = alpha + Math.PI;
                if (alpha > two_PI) {
                    alpha = alpha - two_PI;
                }
                if (alpha < theta && theta < threeHalfs_PI) {
                    ret = -v1_vert_len;
                }
            }
            return ret;
        }

        internal class EdgeBBTester {
            private Vertex point;
            private double point_xminus, point_xplus;
            private double point_yminus, point_yplus;
            
            internal EdgeBBTester(Vertex point, double offset) {
                this.point = point;
                point_xminus = point.x - offset;
                point_xplus  = point.x + offset;
                point_yminus = point.y - offset;
                point_yplus  = point.y + offset;
            }

            internal void tightenOffset(double offset) {
                point_xminus = point.x - offset;
                point_xplus  = point.x + offset;
                point_yminus = point.y - offset;
                point_yplus  = point.y + offset;
            }

            internal bool isInBoundingBox(Vertex xmin, Vertex xmax, Vertex ymin, Vertex ymax) {
                if (point_xplus < xmin.x || point_xminus > xmax.x) {
                    return false;
                }
                if (point_yplus < ymin.y || point_yminus > ymax.y) {
                    return false;
                }
                return true;
            }
        }

        static double maxAbs(Vertex v1, Vertex v2) {
            double ret = 0;
            if (ret < Math.Abs(v1.x)) {
                ret = Math.Abs(v1.x);
            }
            if (ret < Math.Abs(v1.y)) {
                ret = Math.Abs(v1.y);
            }
            if (ret < Math.Abs(v2.x)) {
                ret = Math.Abs(v2.x);
            }
            if (ret < Math.Abs(v2.y)) {
                ret = Math.Abs(v2.y);
            }
            return ret;
        }

        // State constants 
        enum RayState { RayDone, Below, BelowToMiddle, Above, AboveToMiddle };

        static internal (int inout, double radius)
        isInside(Vertex point, LinearRing outerRing, double radiusOfInterest) {

            if (DEBUG) {
                if (radiusOfInterest < 0.0) {
                    throw new Exception("MkGeoAlgo.isInside: radiusOfInterest must be greater than 0");
                }
            }
            // test the bounding box of the ring with radiusOfInterest

            var bb = outerRing.boundingBox();
            if (! isInsideBoundingBox(bb.bottomLeft, bb.topRight, point, radiusOfInterest)) {
                return (1, nextFloat(radiusOfInterest));
            }

            double bbScale = maxAbs(bb.bottomLeft, bb.topRight);
            if (bbScale < 1.0) {
                bbScale = 1.0;
            }
            double epsilon = nextFloat(bbScale) - bbScale;

            int effectiveLength = outerRing.length - 1;
            Func<int, int> roundIndex = n => (n + effectiveLength) % effectiveLength;

            EdgeBBTester edgeBBTester = new EdgeBBTester(point, radiusOfInterest);

            int numIntersection = 0;
            int inout = Int32.MaxValue; // should be -1 (inside), 0 (boundary), 1 (outside)
            double buffer = radiusOfInterest;

            // Determine the initial state. We wlll loop through the verts from "bottom"

            // We will start the loop from bottom

            // bottom is the index of the lowest vert
            int bottom = outerRing.bottomVert;
            
            Vertex v1 = outerRing[bottom];
            RayState state = RayState.Below;
            if (v1.y < point.y) {
                state = RayState.Below;
            
            } else if (v1.y > point.y) {
                state = RayState.RayDone;
                inout = 1;

            // in the following v1.y === point.y
            } else if (bb.bottomLeft.y == bb.topRight.y) {
                // bounding box degenerates to horizontal line segment
                inout = 0;
                state = RayState.RayDone;

            } else {
                // Find the left-most bottom
                int j = bottom;
                for (int i = 1; i < outerRing.length; ++i) {
                    int k = roundIndex(bottom - i);
                    Vertex v_k = outerRing[k];
                    // notice that v1.y is least among all verts
                    // we are sure to find v0
                    if (v_k.y > v1.y) {
                        break;
                    } else {
                        j = k;
                    }
                }
                bottom = j;
                state = RayState.AboveToMiddle;
            }

            // Assertion: state === RayDone || inout == Int32.MaxValue
            //            state !== RayDone || inout != Int32.MaxValue

            Vertex v2 = outerRing[bottom];
            Vertex v0;
            Vertex xmin, xmax, ymin, ymax;
            v1 = outerRing[roundIndex(bottom - 1)];
            for (int i = 1; i < outerRing.length; ++i) {
                v0 = v1;
                v1 = v2;
                v2 = outerRing[roundIndex(bottom + i)];

                if (v1.x < v2.x) {
                    xmin = v1;
                    xmax = v2;
                } else {
                    xmin = v2;
                    xmax = v1;
                }
                if (v1.y < v2.y) {
                    ymin = v1;
                    ymax = v2;
                } else {
                    ymin = v2;
                    ymax = v1;
                }

                if (state == RayState.RayDone) {
                    if (buffer == 0.0) {
                        break;
                    } else {
                        // We have to test edges for the distance
                        if (edgeBBTester.isInBoundingBox(xmin, xmax, ymin, ymax)) {
                            double updated = updateBorderBuffer(buffer, v0, v1, v2, point, epsilon);
                            
                            // Assertion: updated == buffer || |updated| < |buffer| && |updated| < radiusOfInterest
                            
                            if (Math.Abs(updated) < Math.Abs(buffer)) {
                                edgeBBTester.tightenOffset(Math.Abs(updated));
                                buffer = updated;
                            }
                        }
                        continue;
                    }
                } else if (state == RayState.Below || state == RayState.Above) {
                    if (point.y == v2.y) {
                        // entering Middle
                        if (point.x == v2.x) {
                            inout = 0;
                            state = RayState.RayDone;
                        } else {
                            state = (state == RayState.Below) ? RayState.BelowToMiddle : RayState.AboveToMiddle;
                        }
                    } else if (state == RayState.Below ? v2.y > point.y : v2.y < point.y) {
                        // Below -> Above || Above -> Below
                        state = state == RayState.Below ? RayState.Above : RayState.Below;
                        if (point.x <= xmax.x) {
                            if (point.x <= xmin.x) {
                                // there must be hit
                                numIntersection += 1;
                            } else {
                                // xmin.x < point.x <= xmax.x
                                double d = signedDist(ymin, ymax, point);
                                if (d == 0.0) {
                                    // hit the boundary
                                    inout = 0;
                                    state = RayState.RayDone;
                                } else if (d > 0.0) {
                                    // one hit
                                    numIntersection += 1;
                                }
                                // otherwise, Δ < 0: there is no hit
                            }
                        } else {
                            // point.x > xmax.x: there is no hit
                            // if debug
                            //     println("Test 8.5")
                            // end
                        }
                    }
                    // otherwise, state === Below ? v2.y < point.y : v2.y > point.y, and we remain in Below/Above
                        
                } else if (state == RayState.BelowToMiddle || state == RayState.AboveToMiddle) {
                    // We are on the Middle line
                    if (point.y == v2.y) {
                        // Stay on the Middle
                        if (xmin.x <= point.x && point.x <= xmax.x) {
                            // we are on the horizontal edge
                            inout = 0;
                            state = RayState.RayDone;
                        }
                    } else if (state == RayState.BelowToMiddle ? v2.y < point.y : v2.y > point.y) {
                        // Below -> BelowToMiddle -> Below
                        // or, Above -> AboveToMiddle -> Above
                        state = state == RayState.BelowToMiddle ? RayState.Below : RayState.Above;
                    } else {
                        // state === BelowToMiddle ? v2.y > point.y : v2.y < point.y
                        // Below -> BelowToMiddle -> Above
                        // or, Above -> AboveToMiddle -> Below
                        state = state == RayState.BelowToMiddle ? RayState.Above : RayState.Below;
                        if (point.x <= v1.x) {
                            numIntersection += 1;
                        }
                    }
                
                } // end of state transition  
            } // end of the loop through all the verts

            // println("number of intersection = " * string(numIntersection))
            if (inout == Int32.MaxValue) {
                if (numIntersection % 2 == 0) {
                    inout = 1;
                } else {
                    inout = -1;
                }
            }
            if (buffer == radiusOfInterest) {
                if (radiusOfInterest >= Double.MaxValue) {
                    string err = "logical error: the distance to the border cannot be infinity";
                    if (DEBUG) {
                        throw new Exception(err);
                    } else if (geoFenceLogger != null) {
                        geoFenceLogger(err);
                    }
                }
                buffer = inout < 0 ? -nextFloat(radiusOfInterest) : nextFloat(radiusOfInterest);
            }
            return (inout, buffer);
        } // isInside(LinearRing )

        internal static Dictionary<string, (string name, double buffer)[]>
        isInside(MkGeoMultiMap country, Vertex point, double borderMargin) {
            HashSet<string> borders = new HashSet<string>();
            HashSet<string> ins = new HashSet<string>();
            HashSet<string> outs = new HashSet<string>();
            List<(string name, double buffer)> buffers = new List<(string name, double buffer)>();
            Dictionary<string, List<(string name, double buffer)>> ret = new Dictionary<string, List<(string name, double buffer)>>();
            foreach (KeyValuePair<string, MultiPolygon> kv in country) {
                string province = kv.Key;
                MultiPolygon multiPoly = kv.Value;
                double minBuffer = Double.PositiveInfinity;
                foreach (Polygon poly in multiPoly) {
                    LinearRing ring = poly.outerRing;
                    var res = isInside(point, ring, borderMargin);
                    if (res.inout == 0) {
                        borders.Add(province);
                    } else if (res.inout == -1) {
                        ins.Add(province);
                    } else if (res.inout == 1) {
                        outs.Add(province);
                    }
                    if (Math.Abs(res.radius) < Math.Abs(minBuffer)) {
                        minBuffer = res.radius;
                    }
                }
                if (Math.Abs(minBuffer) < Double.PositiveInfinity) {
                    buffers.Add((name: province, buffer: minBuffer));
                }
            }
            HashSet<string> prov = new HashSet<string>(ins);
            prov.UnionWith(borders);
            foreach (string p in prov) {
                if (ins.Contains(p)) {
                    ret["in"] = new List<(string name, double buffer)>();
                    ret["in"].Add((name: p, buffer: 0.0));
                } else if (borders.Contains(p)) {
                    if (ret.ContainsKey("border")) {
                        ret["border"].Add((name: p, buffer: 0.0));
                    } else {
                        ret["border"] = new List<(string name, double buffer)>();
                        ret["border"].Add((name: p, buffer: 0.0));
                    }
                }
            }
            if (buffers.Count > 0) {
                ret["buffer"] = buffers;
            }
            Dictionary<string, (string name, double buffer)[]> retn = new Dictionary<string, (string name, double buffer)[]>();
            foreach (KeyValuePair<string, List<(string name, double buffer)>>  kv in ret) {
                retn.Add(kv.Key, kv.Value.ToArray());
            }
            return retn;
        } // Inside(MkGeoMultiMap)

        static (double, double) sort(double a, double b) {
            if (a <= b) {
                return (a, b);
            } else {
                return (b, a);
            }
        }

        internal static bool doesIntersect(Vertex u1, Vertex u2, Vertex v1, Vertex v2, double epsilon) {
            Vertex vec_u = vector(u1, u2);
            double len_u = norm(vec_u);
            Vertex vec_v = vector(v1, v2);
            double len_v = norm(vec_v);
            Vertex o, e1, e2;
            Vertex p1, p2;
            double len_x;
            if (len_u > len_v) {
                o = u1;
                e1 = vec_u / len_u;
                len_x = len_u;
                e2 = new Vertex(-e1.y, e1.x);
                Vertex o_v1 = vector(o, v1);
                Vertex o_v2 = vector(o, v2);
                p1 = new Vertex(dot(o_v1, e1), dot(o_v1, e2));
                p2 = new Vertex(dot(o_v2, e1), dot(o_v2, e2));
            } else {
                o = v1;
                e1 = vec_v / len_v;
                len_x = len_v;
                e2 = new Vertex(-e1.y, e1.x);
                Vertex o_u1 = vector(o, u1);
                Vertex o_u2 = vector(o, u2);
                p1 = new Vertex(dot(o_u1, e1), dot(o_u1, e2));
                p2 = new Vertex(dot(o_u2, e1), dot(o_u2, e2));
            }
            // Intersect p1 - p2 with e1 axis from 0 to len_x
            if (Math.Abs(p1.y) < epsilon && Math.Abs(p2.y) < epsilon) {
                var minmax = sort(p1.x, p2.x);
                if (minmax.Item2 < -epsilon || minmax.Item1 > len_x + epsilon) {
                    return false;
                } else {
                    return true;
                }
            } else if (Math.Abs(p1.y) < epsilon) {
                if (-epsilon < p1.x && p1.x <= len_x + epsilon) {
                    return true;
                } else {
                    return false;
                }
            } else if (Math.Abs(p2.y) < epsilon) {
                if (-epsilon < p2.x && p2.x <= len_x + epsilon) {
                    return true;
                } else {
                    return false;
                }
            }

            // p1 and p2 are away from 0
            if (p1.y * p2.y > 0) {
                // Same side of x-axis
                return false;
            }

            double t = p2.y / (p2.y - p1.y);
            double x = t * p1.x + (1.0 - t) * p2.x;
            return -epsilon < x &&  x < len_x + epsilon;
        }

        static internal (Vertex bbMin, Vertex bbMax) bbox(params Vertex[] verts) {
            if (verts.Length == 0) {
                throw new ArgumentException();
            } else if (verts.Length == 1) {
                return (new Vertex(verts[0]), new Vertex(verts[0]));
            }
            
            double minx = verts[0].x, maxx = verts[0].x;
            double miny = verts[0].y, maxy = verts[0].y;
            for (int i = 1; i < verts.Length; ++i) {
                if (verts[i].x < minx) {
                    minx = verts[i].x;
                } else if (verts[i].x > maxx) {
                    maxx = verts[i].x;
                }
                if (verts[i].y < miny) {
                    miny = verts[i].y;
                } else if (verts[i].y > maxy) {
                    maxy = verts[i].y;
                }
            }
            return (new Vertex(minx, miny), new Vertex(maxx, maxy));
        }

        static internal double bboxSize(Vertex bbMin, Vertex bbMax) {
            double x = bbMax.x - bbMin.x;
            double y = bbMax.y - bbMin.y;
            return x < y ? y : x;
        }

        static public double nextFloat(double x) {
            if (x < 0.0) {
                throw new ArgumentException("argument to nextfloat must not be negative.");
            }
            long l = BitConverter.DoubleToInt64Bits(x);
            return BitConverter.Int64BitsToDouble(l + 1);
        }
    } // MkGeoAlgo
}
