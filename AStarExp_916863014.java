import java.awt.Point;
import java.util.*;

/// A sample AI that takes a very suboptimal path.
/**
 * This is a sample AI that moves as far horizontally as necessary to reach the target,
 * then as far vertically as necessary to reach the target.  It is intended primarily as
 * a demonstration of the various pieces of the program.
 *
 */
public class AStarExp_916863014 implements AIModule {
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
        /*
//        HashSet<Point> closed = new HashSet<>();
        PriorityQueue<Pair> myPQ = new PriorityQueue<>();
        ArrayList<Point> currentRoute = new ArrayList<>();
        currentRoute.add(CurrentPoint);
        Pair currentPair = new Pair(CurrentPoint, null, 0,0, currentRoute);
        myPQ.add(currentPair);
        while (!myPQ.isEmpty()) {
            Pair v = myPQ.poll();
            if (v.current == Stoppoint) {
                return v.route;
            }
//            if (!closed.contains(v.current)) {
                //get neighbors
                Point[] neighbors = map.getNeighbors(v.current);
                for (Point neighbor : neighbors) {
//                    if (!neighbor.equals(v.pre)) {
                        //how to put neighbors into myPQ in order
                        //estimated total cost mid point new neighbor
                        double fScore = v.gScore + map.getCost(v.current, neighbor) + getHeuristic(map, neighbor, Stoppoint);
                        //actual start to new neighbor
                        double gScore = v.gScore + map.getCost(v.current,neighbor);
                        ArrayList<Point> newRoute = new ArrayList<>(v.route);
                        newRoute.add(neighbor);
                        Pair neighborPair = new Pair(neighbor, CurrentPoint, fScore, gScore, newRoute);
                        myPQ.add(neighborPair);
//                    }
                }
//                closed.add(v.current);
//            }
        }
        System.out.println("Couldn't find path from " + CurrentPoint.toString() + " to " + Stoppoint.toString());
        return null;
        */

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
        double min_exp_cost = 0;
        double h1 = map.getTile(pt1);
        double h2 = map.getTile(pt2);

        if (h2 - h1 > 0) {
            min_exp_cost = (h2 - h1);
        } else {
            min_exp_cost = 0.5 * (h2 - h1);
        }

        return min_exp_cost + Math.max(Math.abs(pt1.x - pt2.x), Math.abs(pt1.y - pt2.y));
    }

/*
    private class Pair implements Comparable<Pair>{

        private Point current;
        private Point pre;
        private double fScore;
        private double gScore;
        private ArrayList<Point> route;

        Pair(Point current, Point pre, double fScore, double gScore, ArrayList<Point> route) {
            this.current = current;
            this.pre = pre;
            this.fScore = fScore;
            this.gScore = gScore;
            this.route = route;
        }

        @Override
        public int compareTo(Pair o) {
            if (this.fScore < o.fScore) {
                return -1;
            } else if (this.fScore == o.fScore) {
                return 0;
            } else {
                return 1;
            }
        }
    }
    //first cost function
    */
}