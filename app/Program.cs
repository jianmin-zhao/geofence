using System;
using MkGeoFence;
using static MkGeoFence.MkGeoAlgo;
using static System.Console;
using System.Diagnostics;

namespace App
{
    class Program
    {
        static void Main(string[] args)
        {

            // string text = System.IO.File.ReadAllText("data/canada.geo.json");
            // System.Console.WriteLine("Contents of WriteText.txt = {0}", text);
            // Object stuff = DeserializeObject(text);
            // System.Console.WriteLine(stuff.GetType().ToString());

            // Tests.test1();
            Tests.test2();
            // Tests.test3();
            // Tests.test4();
            // Tests.test5();
            // Tests.test6();
            // Tests.test7();

            // MkGeoMap mkmap = new MkGeoMap("data/Canada_fixed.3.geojson");
            // Polygon poly = mkmap.polygon;
            // LinearRing ring = poly.outerRing;
            // double eps = MkGeoAlgo.nextFloat(1000.0) - 1000.0;
            // string[] errors = ring.sanityCheck(eps);
            // foreach(string s in errors) {
            //     Console.WriteLine(s);
            // }
        }
    }

    class Tests {

        static Vertex[] testPoints1;

        static Tests() {
            setLogger(System.Console.WriteLine);

            testPoints1 = new Vertex[] {
                new Vertex(-132, 64.65),
                new Vertex(-122.857, 49.141), 
                new Vertex(-113.9507, 52.027),
                new Vertex(-64.5, 45.8),
                new Vertex(-64.0, 43.0)
            };
        }

        internal static void test1() {
            Console.WriteLine("-------- Test 1 ----------");
            MkGeoMultiMap geomap = new MkGeoMultiMap("data/canada.geo.json");
            geomap.summery();
            
            System.Console.WriteLine(String.Join(", ", geomap.regionNames));
            double margin = 0.0;
            foreach (Vertex p in testPoints1) {
                bool ans = geomap.enclose(p, margin, false);
                string ansStr = ans ? "inside" : "outside";
                Console.WriteLine($"Point ({p.x}, {p.y}) is {ansStr} Canada as represented by this MkGeoMap by margin {margin}.");
            }
        }

        internal static void test2() {
            Console.WriteLine("-------- Test 2 ----------");
            
            MkGeoMap mkmap = new MkGeoMap("data/Canada_fixed.3.geojson");
            
            Stopwatch sw = new Stopwatch();

            // int totalTries = 1000000000;
            int totalTries = 1000000;
            LinearRing ring = mkmap.polygon.outerRing;
            var bbox = ring.boundingBox();
            Vertex center = (bbox.bottomLeft + bbox.topRight) / 2.0;
            Vertex bbspan = 1.0 * (bbox.topRight - bbox.bottomLeft);
            Vertex[] points = new Vertex[totalTries];
            Random randx = new Random();
            Random randy = new Random();
            for (int i = 0; i < totalTries; ++i) {
                double x = center.x + (randx.NextDouble() - 0.5) * bbspan.x;
                double y = center.y + (randy.NextDouble() - 0.5) * bbspan.y;
                points[i] = new Vertex(x, y);
            }
            // double radiusOfInterest = Double.MaxValue;
            double radiusOfInterest = Double.MaxValue;
            // double radiusOfInterest = 5.0 * 1609.34; // 5 miles in meters

            Console.WriteLine($"Start running for {totalTries} testing.");
            int inCount = 0;
            int outCount = 0;
            int borderCount = 0;

            sw.Start();
            for (int i = 0; i < totalTries; ++i) {
                double dist = mkmap.signedDistanceMeterToBorder(points[i].x, points[i].y, radiusOfInterest);
                if (dist < 0.0) {
                    ++inCount;
                } else if (dist > 0.0) {
                    ++outCount;
                } else {
                    ++borderCount;
                }
            }
            sw.Stop();

            Console.WriteLine($"Total time = {sw.ElapsedMilliseconds}. In = {inCount}, Out = {outCount}, "
                +$"Border = {borderCount}, radiusOfInterest = {radiusOfInterest}");
            Console.WriteLine($"{(double)(sw.ElapsedMilliseconds)/totalTries} milliseconds per point");
        }

        internal static void test3() {
            Console.WriteLine("-------- Test 3 ----------");
            
            MkGeoMap mkmap = new MkGeoMap("data/Canada_fixed.3.geojson");

            double[] ys = new double[17];
            for (int i = 0; i < 17; ++i) {
                ys[i] = 45.54 + 0.01 * i;
            }
            double margin = MkGeoAlgo.meter2deg(700.0);
            double offsetLimit = MkGeoAlgo.meter2deg(10000.0);
            foreach (double y in ys) {
                Vertex pp = new Vertex(-60.0, y);
                double ans = mkmap.distanceToBorder(pp, offsetLimit);
                string ansStr = Math.Abs(ans) < margin ? "inside" : "outside";
                Console.WriteLine($"Point ({pp.x}, {pp.y}) is {ansStr} Canada as represented by this MkGeoMap "
                +"by margin {margin}. The offset is {ans:F4}");
            }
        }

        internal static void test4() {
            Console.WriteLine("-------- Test 4 ----------");
            
            MkGeoMap mkmap = new MkGeoMap("data/Canada_fixed.3.geojson");

            double[] ys = new double[17];
            for (int i = 0; i < 17; ++i) {
                ys[i] = 45.54 + 0.01 * i;
            }
            foreach (double y in ys) {
                Vertex pp = new Vertex(-60.0, y);
                double ans = mkmap.signedDistanceMeterToBorder(pp.x, pp.y, Double.PositiveInfinity);
                Console.WriteLine($"Point ({pp.x}, {pp.y}) is {ans:F2} from the boarder");
            }
        }

        internal static void test5() {
            Console.WriteLine("-------- Test 5 ----------");
            
            MkGeoMap mkmap = new MkGeoMap("data/Canada_fixed.3.geojson"); 

            double[,] points = new double[,] {                
                {-140.45745849609378, 60.30858669066228},
                {-140.51994323730472, 60.22003701633967},
                {-141.0026550292969, 60.30722620202002},
                {-141.00059509277347, 69.6458642180311},
                {-140.80078125000003, 73.35305494105975},
                {-76.81640625000001, 83.89571893465583},
                {-58.16162109375, 82.56623305534582},
                {-57.94189453125001, 82.58327054597385},
                {-66.73095703125001, 80.89719300308133},
                {-73.76220703125001, 78.45982190791324},
            };
            Vertex[] vPoints = new Vertex[points.GetLength(0)];
            for (int i = 0; i < points.GetLength(0); ++i) {
                vPoints[i] = new Vertex(points[i,0], points[i,1]);
            }

            double distOfInterest = Double.MaxValue;
            foreach (Vertex p in vPoints) {
                try {
                    double ans = mkmap.distanceToBorder(p, distOfInterest);
                    Console.WriteLine($"Point ({p.x}, {p.y}) is {ans:F8} away from the boarder");
                } catch (Exception exc) {
                    Console.WriteLine("Exception "+exc.Message);
                }
            }
        }

        internal static void test6() {
            Console.WriteLine("-------- Test 6 ----------");

            MkGeoMap mkmap = new MkGeoMap("data/Canada_fixed.3.geojson");

            Polygon poly = mkmap.polygon;
            LinearRing ring = poly.outerRing;
            int vertLen = ring.length;
            Random rand = new Random();
            Vertex[] testPoints = new Vertex[vertLen];
            for (int i = 0; i < vertLen; ++i) {
                double rand1 = rand.NextDouble() - 0.5;
                double rand2 = rand.NextDouble() - 0.5;
                testPoints[i] = new Vertex(ring[i].x + rand1, ring[i].y + rand2);
            }

            foreach (Vertex p in testPoints) {
                double ans = mkmap.signedDistanceMeterToBorder(p.x, p.y, 40000.0);
                Console.WriteLine($"Point ({p.x}, {p.y}) is {ans:F2} away from the boarder");
            }
        }

        internal static void test7() {
            Console.WriteLine("-------- Test 7 ----------");

            MkGeoMap mkmap = new MkGeoMap("data/Canada_fixed.3.geojson");

            Polygon poly = mkmap.polygon;
            LinearRing ring = poly.outerRing;
            int vertLen = ring.length;
            Random rand = new Random();
            Vertex[] testPoints = new Vertex[vertLen];
            for (int i = 0; i < vertLen; ++i) {
                double rand1 = rand.NextDouble() - 0.5;
                double rand2 = rand.NextDouble() - 0.5;
                testPoints[i] = new Vertex(ring[i].x + rand1, ring[i].y + rand2);
            }

            foreach (Vertex p in testPoints) {
                double ans = mkmap.distanceToBorder(p, Double.PositiveInfinity);
                Console.WriteLine($"Point ({p.x}, {p.y}) is {ans:F2} away from the boarder");
            }
        }
    }
}
