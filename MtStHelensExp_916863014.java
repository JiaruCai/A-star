import java.awt.Point;
import java.util.*;

/// A sample AI that takes a very suboptimal path.
/**
 * This is a sample AI that moves as far horizontally as necessary to reach the target,
 * then as far vertically as necessary to reach the target.  It is intended primarily as
 * a demonstration of the various pieces of the program.
 *
 */
public class MtStHelensExp_916863014 implements AIModule {
    /// Creates the path to the goal.
    public List<Point> createPath(final TerrainMap map)
    {
        // Holds the resulting path
        final ArrayList<Point> path = new ArrayList<Point>();

        // Keep track of where we are and add the start point.
        final Point CurrentPoint = map.getStartPoint();
        final Point Stoppoint = map.getEndPoint();
        ArrayList<Point> result =  Astar(CurrentPoint, Stoppoint, map);
        for (Point points : result) {
            path.add(points);
        }
        return path;
    }

    //fix this later

    private double nullToInf(Double d) {
        if (d == null) {
            return Double.POSITIVE_INFINITY;
        }
        return d;
    }

    public ArrayList<Point> Astar(Point CurrentPoint, Point Stoppoint, TerrainMap map) {

        HashMap<Point, Point> prevs = new HashMap<>();

        HashMap<Point, Double> gScores = new HashMap<>();
        gScores.put(CurrentPoint, 0.0);

        HashMap<Point, Double> fScores = new HashMap<>();
        fScores.put(CurrentPoint, getHeuristic(map, CurrentPoint, Stoppoint));



        PriorityQueue<Point> myPQ = new PriorityQueue<Point>(
                (a, b) -> (nullToInf(fScores.get(a)) < nullToInf(fScores.get(b)) ? -1 : 1));
        myPQ.add(CurrentPoint);

        while (!myPQ.isEmpty()) {
            Point cur = myPQ.poll();

            if (cur.equals(Stoppoint)) {
                ArrayDeque<Point> ret = new ArrayDeque<>();
                ret.add(cur);

                while (prevs.containsKey(cur)) {
                    cur = prevs.get(cur);
                    ret.addFirst(cur);
                }

                return new ArrayList<Point>(ret);
            }

            for (Point neighbour : map.getNeighbors(cur)) {
                double tentative_gscore = gScores.get(cur) + map.getCost(cur, neighbour);
//                System.out.println("Neighbour has cost " + map.getCost(cur, neighbour));

                if (!gScores.containsKey(neighbour) || tentative_gscore < gScores.get(neighbour)) {
                    prevs.put(neighbour, cur);
                    gScores.put(neighbour, tentative_gscore);
                    fScores.put(neighbour, tentative_gscore + getHeuristic(map, neighbour, Stoppoint));

//                    if (myPQ.contains(neighbour)) {
                    myPQ.remove(neighbour);
//                    }
                    myPQ.add(neighbour);
                }
            }
        }

        System.out.println("Couldn't find path from " + CurrentPoint.toString() + " to " + Stoppoint.toString());
        return null;
    }

    private double getHeuristic(final TerrainMap map, final Point pt1, final Point pt2) {
        // return Math.pow(2.0, (getTile(p2) - getTile(p1)))
        return Math.pow(2.0, (map.getTile(pt2) - map.getTile(pt1)))*0.5 + Math.max(Math.abs(pt1.x - pt2.x), Math.abs(pt1.y - pt2.y))*0.5;
    }

}