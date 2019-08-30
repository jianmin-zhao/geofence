using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using static Newtonsoft.Json.JsonConvert;
using Newtonsoft.Json.Linq;

using static MkGeoFence.MkGeoAlgo;

namespace MkGeoFence
{
    public struct Vertex {
        private double _x;
        private double _y;
        
        public Vertex(double x, double y) {
            _x = x;
            _y = y;
        }

        public Vertex(Vertex src) {
            _x = src.x;
            _y = src.y;
        }

        public double x {
            get {
                return _x;
            }
        }

        public double y {
            get {
                return _y;
            }
        }

        public bool coincidesWith(Vertex other) {
            return x == other.x && y == other.y;
        }

        static public Vertex operator *(double s, Vertex v) {
            return new Vertex(s*v.x, s*v.y);
        }

        static public Vertex operator /(Vertex v, double s) {
            return new Vertex(v.x/s, v.y/s);
        }

        static public Vertex operator +(Vertex v1, Vertex v2) {
            return new Vertex(v1.x + v2.x, v1.y + v2.y);
        }

        static public Vertex operator -(Vertex v1, Vertex v2) {
            return new Vertex(v1.x - v2.x, v1.y - v2.y);
        }
    }

    // A LinearRing is a sequence of vertexes, such that
    // 1) the first first vertex coincides with the last one
    // 2) there is no intersection among the edges except for neiboring ones.
    // 3) the clockwise sequence denotes the area it encloses, whereas the counter-clockwise sequence excludes the
    //    the enclosed area from the outer ring (hole).

    public class LinearRing {
        Vertex[] verts;
        int      left;
        int      right;
        int      bottom;
        int      top;

        public LinearRing(Vertex[] vertices) {
            verts = vertices;
            left = 0;
            right = 0;
            bottom = 0;
            top = 0;
            for (int k = 1; k < verts.Length; ++k) {
                if (verts[k].x < verts[left].x) {
                    left = k;
                }
                if (verts[k].x > verts[right].x) {
                    right = k;
                }
                if (verts[k].y < verts[bottom].y) {
                    bottom = k;
                }
                if (verts[k].y > verts[top].y) {
                    top = k;
                }
            }
        }

        public Vertex this[int i] {
            get {
                return verts[i];
            }
        }

        public int length {
            get {
                return verts.Length;
            }
        }

        internal int leftVert {
            get {
                return left;
            }
        }

        internal int rightVert {
            get {
                return right;
            }
        }

        internal int bottomVert {
            get {
                return bottom;
            }
        }

        internal int topVert {
            get {
                return top;
            }
        }

        // Bounding box of the LinearRing.
        // Returns: tuple of two vertex, the first one contains mimimum x and y, and the second one contains
        // maximum x and y

        public (Vertex bottomLeft, Vertex topRight) boundingBox() {
            Vertex bottomLeft = new Vertex(verts[left].x,  verts[bottom].y);
            Vertex topRight   = new Vertex(verts[right].x, verts[top].y);
            return (bottomLeft, topRight);
        }

        struct EdgeInfo {
            internal Vertex bbMin;
            internal Vertex bbMax;
            internal bool   isDegeneate; // degenerate to a point
        }

        public string[] sanityCheck(double epsilon) {

            List<string> errors = new List<string>();

            if (length < 4) {
                errors.Add("An LinearRing must have at least 4 vertexes");
            } else if (!this[0].coincidesWith(this[length-1])) {
                errors.Add("The first and last vertexes must coincide");
            } else if (this[rightVert].x - this[leftVert].x < epsilon || this[topVert].y - this[bottomVert].y < epsilon) {
                errors.Add("This polygon is degenerate to one dimenstion");
            }
            if (errors.Count > 0) {
                return errors.ToArray();
            }
            
            if (length < 5) {
                return new string[0];
            }

            EdgeInfo[] edgeInfos = new EdgeInfo[length - 1];
            for (int i = 0; i < length - 1; ++i) {
                var bb = bbox(this[i], this[i+1]);
                edgeInfos[i].bbMin = bb.bbMin;
                edgeInfos[i].bbMax = bb.bbMax;
                edgeInfos[i].isDegeneate = bboxSize(bb.bbMin, bb.bbMax) < epsilon ? true : false;
                if (edgeInfos[i].isDegeneate) {
                    errors.Add($"Degenerate edge (degenerate to point) at {i}");
                }
            }

            for (int i = 2; i <= length - 2; ++i) {
                // compare v[i] -> v[i+1] with v[j] -> v[j+1] where j goes 0 to i - 2, except for the last one.

                if (edgeInfos[i].isDegeneate) {
                    continue;
                }

                int begin = 0;
                if (i == length - 2) {
                    begin = 1;
                }
                for (int j = begin; j <= i - 2; ++j) {
                    // check intersecton between [v[i], v[i+1]] and [v[j], v[j+1]]

                    if (edgeInfos[j].isDegeneate) {
                        continue;
                    }

                    if (edgeInfos[i].bbMax.x + epsilon < edgeInfos[j].bbMin.x || 
                        edgeInfos[j].bbMax.x + epsilon < edgeInfos[i].bbMin.x ||
                        edgeInfos[i].bbMax.y + epsilon < edgeInfos[j].bbMin.y ||
                        edgeInfos[j].bbMax.y + epsilon < edgeInfos[i].bbMin.y) {
                        continue;
                    }

                    if (doesIntersect(this[i], this[i+1], this[j], this[j+1], epsilon)) {
                        errors.Add($"Edge {i} intersects edge {j}");
                    }
                }
            }
            return errors.ToArray();
        }
    } // LinearRing

    // A polygon is composed by one outer ring and 0 or more inner rings.

    public class Polygon {
        LinearRing[] rings;

        public Polygon(LinearRing[] linearRings) {
            rings = linearRings;
        }

        public LinearRing outerRing {
            get {
                return rings[0];
            }
        }

        public LinearRing[] innerRings {
            get {
                return ((IEnumerable<LinearRing>)rings).Skip(1).ToArray();
            }
        }
    }

    // MultiPolygon is merely a number of polygons. For example, a comprehensive map of Canada may
    // consist of a let of polygons with each polygon being a province/territory.
    
    public class MultiPolygon: IEnumerable<Polygon> {
        Polygon[] polys;

        public MultiPolygon(Polygon[] polygons) {
            polys = polygons;
        }

        public int size {
            get {
                return polys.Length;
            }
        }

        public Polygon this[int i] {
            get {
                return polys[i];
            }
        }

        IEnumerator IEnumerable.GetEnumerator() {
            return GetEnumerator();
        }

        public IEnumerator<Polygon> GetEnumerator() {
            return ((IEnumerable<Polygon>)polys).GetEnumerator();
        }
    }

    public delegate void Logger(String log);

    // Underlying MkGeoMultiMap is a MultiPolygon

    public class MkGeoMultiMap : Dictionary<string, MultiPolygon> {

        // May throw: parseGeoJson may throw if geoJsonFile is not conformant.
        public MkGeoMultiMap(string geoJsonFile) {
            // multiPolygons = null;
            string text = System.IO.File.ReadAllText(geoJsonFile);
            JObject jobj = JObject.Parse(text);
            parseGeoJson(jobj);
        }

        public string[] regionNames {
            get {
                string[] ret = new string[this.Count];
                this.Keys.CopyTo(ret, 0);
                return ret;
            }
        }

        void parseGeoJson(JObject jobj) {

            string jtype = "";
            JArray features;
            try {
                jtype = jobj["type"].Value<string>();
                if (!jtype.Equals("FeatureCollection")) {
                    new Exception("Invalid geo type: "+jtype);
                }
                features = (JArray) jobj["features"];
                foreach (JObject feature in features) {
                    string ftype = feature["type"].Value<string>();
                    if (!ftype.Equals("Feature")) {
                        new Exception("Invalid feature type: "+ftype);
                    }
                    // feature["properties"] = {
                    //   "name": "Quebec",
                    //   "cartodb_id": 1,
                    //   "created_at": "2014-10-16T13:22:00Z",
                    //   "updated_at": "2014-10-16T13:22:00Z"
                    // }
                    string name = ((JObject)feature["properties"])["name"].Value<string>();
                    
                    JObject geometry = (JObject) feature["geometry"];
                    string gtype = geometry["type"].Value<string>();
                    JArray coords = (JArray) geometry["coordinates"];
                    JArray polyCoords = null;
                    if (gtype.Equals("MultiPolygon")) {
                        polyCoords = coords;
                    } else if (gtype.Equals("Polygon")) {
                        polyCoords = new JArray();
                        polyCoords.Add(coords);
                    } else {
                        throw new Exception("Unsurported geometry type");
                    }

                    Polygon[] polys = new Polygon[polyCoords.Count];
                    for (int p = 0; p < polyCoords.Count; ++p) {
                        JArray coords1 = (JArray) polyCoords[p];
                        LinearRing[] rings = new LinearRing[coords1.Count];
                        for (int r = 0; r < coords1.Count; ++r) {
                            JArray coords2 = (JArray) coords1[r];
                            Vertex[] verts = new Vertex[coords2.Count];
                            for (int v = 0; v < coords2.Count; ++v) {
                                JArray vertex = (JArray) coords2[v];
                                verts[v] = new Vertex(vertex[0].Value<double>(), vertex[1].Value<double>());
                            }
                            rings[r] = new LinearRing(verts);
                        }
                        polys[p] = new Polygon(rings);
                    }

                    this[name] = new MultiPolygon(polys);
                }
            } catch (Exception exc) {
                Console.WriteLine("Exception: "+exc.Message);
                throw exc;
            }
        }

        public void summery() {
            if (null == geoFenceLogger) {
                return;
            }
            foreach (KeyValuePair<string, MultiPolygon> kvp in this) {
                geoFenceLogger($"Key = {kvp.Key}");
                MultiPolygon m = kvp.Value;
                geoFenceLogger($"There are {m.size} polygons");
                foreach (Polygon poly in m) {
                    LinearRing outerRing = poly.outerRing;
                    geoFenceLogger($"There are {outerRing.length} verts on the outerRing");
                    LinearRing[] innerRings = poly.innerRings;
                    geoFenceLogger($"There are {innerRings.Length} inner rings");
                }
                geoFenceLogger("");
            }
        }

        public bool enclose(Vertex point, double margin, bool logDetails=false) {
            var res = MkGeoAlgo.isInside(this, point, 2*margin);
            // print(JSON.json(res, 2))
            if (logDetails && null != geoFenceLogger) {
                bool hasIn = false;
                bool hasBorder = false;
                geoFenceLogger("________________");
                geoFenceLogger($"point ({point.x}, {point.y}): ");
                if (res.ContainsKey("in")) {
                    geoFenceLogger($"  is inside {res["in"][0].name}");
                    hasIn = true;
                }
                if (res.ContainsKey("border")) {
                    geoFenceLogger($"  is on the border of {res["border"][0].name}");
                    hasBorder = true;
                }
                if (!(hasIn || hasBorder)) {
                    geoFenceLogger("  is outside the map of this GeoPoly");
                }
                if (res.ContainsKey("buffer")) {
                    geoFenceLogger($"  is within the buffer of these regions: ");
                    foreach (var b in res["buffer"]) {
                        geoFenceLogger($"    ({b.name}, {b.buffer})");
                    }
                }
                geoFenceLogger("^^^^^^^^^^^^^^^");
            }
            if (res.ContainsKey("in") || res.ContainsKey("border")) {
                return true;
            } else if (res.ContainsKey("buffer")) {
                foreach (var b in res["buffer"]) {
                    if (Math.Abs(b.buffer) <= margin) {
                        return true;
                    }
                }
            }
            return false;
        }
    } // MkGeoMultiMap

    // MkGeoMap has only polygon, namely one connected area.
    
    public class MkGeoMap {

        // May throw: parseGeoJson may throw if geoJsonFile is not conformant.
        public MkGeoMap(string geoJsonFile) {

            MkGeoMultiMap multi = new MkGeoMultiMap(geoJsonFile);
            if (multi.Count != 1) {
                throw new Exception("Only support geojson with one feature (region)");
            }
            var keys = multi.Keys;
            _name = keys.ElementAt(0);
            if (multi[_name].size != 1) {
                throw new Exception("Only support geojson feature of Polygon (as opposed to MultiPolygon");
            }
            _polygon = multi[_name][0];
        }

        String _name;
        Polygon _polygon;

        public String name {
            get {
                return _name;
            }
        }

        public Polygon polygon {
            get {
                return _polygon;
            }
        }

        public void summery() {
            if (null == geoFenceLogger) {
                return;
            }
            geoFenceLogger($"This is the map for {this.name}:");
            LinearRing outerRing = _polygon.outerRing;
            geoFenceLogger($"There are {outerRing.length} verts on the outerRing");
            LinearRing[] innerRings = _polygon.innerRings;
            geoFenceLogger($"There are {innerRings.Length} inner rings");
            geoFenceLogger("");
        }

        public double signedDistanceMeterToBorder(double lon, double lat, double radiusOfInterest) {
            if (radiusOfInterest < 0) {
                throw new ArgumentException("radiusOfInterest must not be negative!");
            }

            double radiusDeg = meter2deg(radiusOfInterest);
            // We are coming into the deg space. 360 is the de facto upper limit
            if (radiusDeg > 360.0) {
                radiusDeg = 360.0;
            }
            double signedDistDeg = distanceToBorder(new Vertex(lon, lat), radiusDeg);
            if (Math.Abs(signedDistDeg) > radiusDeg) {
                double radiusOfInterest_plus = radiusOfInterest < Double.MaxValue
                    ? nextFloat(radiusOfInterest) : radiusOfInterest;
                if (signedDistDeg < 0.0) {
                    return -radiusOfInterest_plus;
                } else {
                    return radiusOfInterest_plus;
                }                
            } else {
                return deg2meter(signedDistDeg);
            }
        }

        public double distanceToBorder(Vertex point, double radiusOfInterest) {
            LinearRing outerRing = this.polygon.outerRing;
            var res = MkGeoAlgo.isInside(point, this.polygon.outerRing, radiusOfInterest);

            double ret = res.radius;
            if (res.inout == 0) {
                ret = 0.0;
            } else if (res.inout < 0) {
                // inside
                if (res.radius > 0.0) {
                    // This should not happen, but we log and trim it.
                    string err = $"Inside the ring but buffer {res.radius} > 0! {point.x} , {point.y}, {res.inout}";
                    if (DEBUG) {
                        throw new Exception(err);
                    } else if (geoFenceLogger != null) {
                        geoFenceLogger(err);
                    }
                    ret = 0.0;
                }
            } else {
                // res.inout > 0
                if (res.radius < 0.0) {
                    // This should not happen, but we log and trim it.
                    string err = $"Outside the ring but buffer {res.radius} < 0! {point.x} , {point.y}, {res.inout}";
                    if (DEBUG) {
                        throw new Exception(err);
                    } else if (geoFenceLogger != null) {
                        geoFenceLogger(err);
                    }
                    ret = 0.0;
                }
            }

            return ret;
        }
    } // MkGeoMap
} // namespace MkGepFence
