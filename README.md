# my_2D_grid_city
This is a 2D grid city code for helping in delivering packages 

Import java.util.*;
Import java.io.PrintWriter;
Import java.io.FileWriter;
Import java.io.IOException;

Public class Autonomous_Delivery
    Static class Pos {
        Final int r, c;
        Pos(int r, int c) { this.r = r; this.c = c; }
        Public boolean equals(Object o) {
            If (!(o instanceof Pos)) return false;
            Pos p = (Pos)o; return p.r == r && p.c == c;
        }
        Public int hashCode() { return r * 1000003 + c; }
        Public String toString() { return “(“ + r + “,” + c + “)”; }
    }

    Static int manhattan(Pos a, Pos b) {
        Return Math.abs(a.r – b.r) + Math.abs(a.c – b.c);
    }


    Static class DynamicObstacle {
        Final List<Pos> path; // repeated
        Final int startTime;
        DynamicObstacle(List<Pos> path, int startTime) { this.path = path; this.startTime = startTime; }
        Boolean occupies(Pos p, int time) {
            If (time < startTime) return false;
            Int idx = (time – startTime) % path.size();
            Return path.get(idx).equals(p);
        }
    }

    Static class GridWorld {
        Final int width, height;
        Final double[][] terrain; // movement cost for entering cell
        Final boolean[][] blocked; // static obstacles
        Final List<DynamicObstacle> dynamic;

        GridWorld(int h, int w) {
            This.width = w; this.height = h;
            Terrain = new double[h][w];
            Blocked = new boolean[h][w];
            Dynamic = new ArrayList<>();
            For (int i=0;i<h;i++) for (int j=0;j<w;j++) terrain[i][j] = 1.0;
        }

        Boolean inBounds(Pos p) { return p.r >=0 && p.r < height && p.c >= 0 && p.c < width; }
        Boolean passable(Pos p) { return inBounds(p) && !blocked[p.r][p.c]; }
        Boolean passable(Pos p, int time) {
            If (!inBounds(p) || blocked[p.r][p.c]) return false;
            For (DynamicObstacle dob : dynamic) if (dob.occupies(p, time)) return false;
            Return true;
        }
        Double cost(Pos p) { return terrain[p.r][p.c]; }

        Iterable<Pos> neighbors(Pos p) {
            ArrayList<Pos> n = new ArrayList<>(4);
            Int[][] moves = {{1,0},{-1,0},{0,1},{0,-1}};
            For (int[] m : moves) {
                Pos np = new Pos(p.r + m[0], p.c + m[1]);
                If (inBounds(np) && !blocked[np.r][np.c]) n.add(np);
            }
            Return n;
        }
    }
    Static class SearchResult {
        Final List<Pos> path;
        Final double cost;
        Final int nodesExpanded;
        Final double timeSec;
        SearchResult(List<Pos> path, double cost, int nodesExpanded, double timeSec) {
            This.path = path; this.cost = cost; this.nodesExpanded = nodesExpanded; this.timeSec = timeSec;
        }
    }
    Static SearchResult bfs(GridWorld g, Pos start, Pos goal, int maxNodes) {
        Long t0 = System.nanoTime();
        ArrayDeque<Pos> frontier = new ArrayDeque<>();
        Frontier.add(start);
        Map<Pos,Pos> parent = new HashMap<>();
        Parent.put(start, null);
        Int expanded = 0;
        While (!frontier.isEmpty()) {
            Pos cur = frontier.poll();
            Expanded++;
            If (expanded > maxNodes) break;
            If (cur.equals(goal)) {
                List<Pos> path = reconstruct(parent, start, goal);
                Double cost = pathCost(g, path);
                Return new SearchResult(path, cost, expanded, (System.nanoTime()-t0)/1e9);
            }
            For (Pos nb : g.neighbors(cur)) {
                If (!parent.containsKey(nb)) {
                    Parent.put(nb, cur);
                    Frontier.add(nb);
                }
            }
        }
        Return new SearchResult(null, Double.POSITIVE_INFINITY, expanded, (System.nanoTime()-t0)/1e9);
    }

    Static SearchResult uniformCost(GridWorld g, Pos start, Pos goal, int maxNodes) {
        Long t0 = System.nanoTime();
        Class Node implements Comparable<Node> {
            Pos p; double cost;
            Node(Pos p, double cost) { this.p = p; this.cost = cost; }
            Public int compareTo(Node o) { return Double.compare(this.cost, o.cost); }
        }
        PriorityQueue<Node> pq = new PriorityQueue<>();
        Pq.add(new Node(start, 0.0));
        Map<Pos,Pos> parent = new HashMap<>();
        Map<Pos,Double> costSoFar = new HashMap<>();
        costSoFar.put(start, 0.0);
        int expanded = 0;
        while (!pq.isEmpty()) {
            Node n = pq.poll();
            Pos cur = n.p;
            Double curCost = n.cost;
            Expanded++;
            If (expanded > maxNodes) break;
            If (cur.equals(goal)) {
                List<Pos> path = reconstruct(parent, start, goal);
                Return new SearchResult(path, curCost, expanded, (System.nanoTime()-t0)/1e9);
            }
            For (Pos nb : g.neighbors(cur)) {
                Double newCost = costSoFar.get(cur) + g.cost(nb);
                If (!costSoFar.containsKey(nb) || newCost < costSoFar.get(nb)) {
                    costSoFar.put(nb, newCost);
                    parent.put(nb, cur);
                    pq.add(new Node(nb, newCost));
                }
            }
        }
        Return new SearchResult(null, Double.POSITIVE_INFINITY, expanded, (System.nanoTime()-t0)/1e9);
    }

    Static SearchResult aStar(GridWorld g, Pos start, Pos goal, int maxNodes) {
        Long t0 = System.nanoTime();
        Double minCost = Double.POSITIVE_INFINITY;
        For (int i=0;i<g.height;i++) for (int j=0;j<g.width;j++) minCost = Math.min(minCost, g.terrain[i][j]);
        If (minCost <= 0) minCost = 1e-6;
        Class Node implements Comparable<Node> {
            Pos p; double gCost; double f;
            Node(Pos p, double gCost, double f) { this.p = p; this.gCost = gCost; this.f = f; }
            Public int compareTo(Node o) { int cmp = Double.compare(this.f, o.f); if (cmp!=0) return cmp; return Double.compare(this.gCost, o.gCost); }
        }
        PriorityQueue<Node> pq = new PriorityQueue<>();
        Pq.add(new Node(start, 0.0, minCost * manhattan(start, goal)));
        Map<Pos,Pos> parent = new HashMap<>();
        Map<Pos,Double> gScore = new HashMap<>();
        gScore.put(start, 0.0);
        int expanded = 0;
        while (!pq.isEmpty()) {
            Node n = pq.poll();
            Pos cur = n.p;
            Double curG = n.gCost;
            Expanded++;
            If (expanded > maxNodes) break;
            If (cur.equals(goal)) {
                List<Pos> path = reconstruct(parent, start, goal);
                Return new SearchResult(path, curG, expanded, (System.nanoTime()-t0)/1e9);
            }
            For (Pos nb : g.neighbors(cur)) {
                Double tentativeG = gScore.get(cur) + g.cost(nb);
                If (!gScore.containsKey(nb) || tentativeG < gScore.get(nb)) {
                    gScore.put(nb, tentativeG);
                    parent.put(nb, cur);
                    double f = tentativeG + minCost * manhattan(nb, goal);
                    pq.add(new Node(nb, tentativeG, f));
                }
            }
        }
        Return new SearchResult(null, Double.POSITIVE_INFINITY, expanded, (System.nanoTime()-t0)/1e9);
    }

    Static List<Pos> reconstruct(Map<Pos,Pos> parent, Pos start, Pos goal) {
        LinkedList<Pos> rev = new LinkedList<>();
        Pos cur = goal;
        While (cur != null && !cur.equals(start)) {
            Rev.addFirst(cur);
            Cur = parent.get(cur);
        }
        If (cur == null) return null;
        Rev.addFirst(start);
        Return new ArrayList<>(rev);
    }

    Static double pathCost(GridWorld g, List<Pos> path) {
        If (path == null) return Double.POSITIVE_INFINITY;
        Double s = 0.0;
        For (Pos p : path) s += g.cost(p);
        Return s;
    }
            For (int k=1;k<sub.path.size()-1;k++) candidate.add(sub.path.get(k));
            For (int k=j+1;k<current.size();k++) candidate.add(current.get(k));
            Double candCost = pathCost(g, candidate);
            Double elapsed = (System.nanoTime() – t0) / 1e9;
            Double temp = Math.max(0.01, 1.0 – elapsed / timeLimitSec); // simple schedule
            Double acceptProb = 1.0;
            If (candCost > currentCost) {
                acceptProb = Math.exp((currentCost – candCost) / (temp + 1e-9));
            }
            If (rnd.nextDouble() < acceptProb) {
                Current = candidate;
                currentCost = candCost;
                if (currentCost < bestCost) {
                    best = new ArrayList<>(current);
                    bestCost = currentCost;
                }
            }
        }
        Return new SearchResult(best, bestCost, iterations, (System.nanoTime()-t0)/1e9);
    }

    
    Static class ExecOutcome {
        Final List<Pos> executed;
        Final double cost;
        Final int nodesExpandedDuringExecution;
        Final int replans;
        Final double timeSec;
        Final boolean success;
        ExecOutcome(List<Pos> executed, double cost, int nodesExpandedDuringExecution, int replans, double timeSec, boolean success) {
            This.executed = executed; this.cost = cost; this.nodesExpandedDuringExecution = nodesExpandedDuringExecution;
            This.replans = replans; this.timeSec = timeSec; this.success = success;
        }
    }

    Static ExecOutcome simulateExecution(GridWorld g, List<Pos> plan, String plannerMode) {
        Long t0 = System.nanoTime();
        If (plan == null || plan.isEmpty()) return new ExecOutcome(Collections.emptyList(), 0.0, 0, 0, (System.nanoTime()-t0)/1e9, false);
        List<Pos> executed = new ArrayList<>();
        Executed.add(plan.get(0));
        Int timeStep = 0;
        Int idx = 1;
        Pos current = plan.get(0);
        Int replans = 0;
        Int nodesExpandedAcc = 0;
        While (idx < plan.size()) {
            Pos nxt = plan.get(idx);
            
                List<Pos> newPlan = null;
                SearchResult sr = null;
                If (plannerMode.equals(“bfs”)) {
                    Sr = bfs(g, current, plan.get(plan.size()-1), 200000);
                    nodesExpandedAcc += sr.nodesExpanded;
                    newPlan = sr.path;
                } else if (plannerMode.equals(“ucs”)) {
                    Sr = uniformCost(g, current, plan.get(plan.size()-1), 200000);
                    nodesExpandedAcc += sr.nodesExpanded;
                    newPlan = sr.path;
                } else if (plannerMode.equals(“astar”)) {
                    Sr = aStar(g, current, plan.get(plan.size()-1), 200000);
                    nodesExpandedAcc += sr.nodesExpanded;
                    newPlan = sr.path;
                } else if (plannerMode.equals(“sa_replan”)) {
                    // first try A* limited, then try quick SA repair
                    Sr = aStar(g, current, plan.get(plan.size()-1), 5000);
                    nodesExpandedAcc += sr.nodesExpanded;
                    if (sr.path == null) {
                        newPlan = null;
                    } else {
                        SearchResult sa = simulatedAnnealingRepair(g, current, plan.get(plan.size()-1), sr.path, 0.2);
                        nodesExpandedAcc += sa.nodesExpanded;
                        newPlan = sa.path;
                    }
                } else {
                    newPlan = null;
                }
                If (newPlan == null) {
                    Double cost = pathCost(g, executed);
                    Return new ExecOutcome(executed, cost, nodesExpandedAcc, replans, (System.nanoTime()-t0)/1e9, false);
                } else {
                    Plan = newPlan;
                    Idx = 1;
                    Continue;
                }
            }
            // step valid
            Executed.add(nxt);
            timeStep += 1;
            current = nxt;
            idx += 1;
        }
        Double cost = pathCost(g, executed);
        Return new ExecOutcome(executed, cost, nodesExpandedAcc, replans, (System.nanoTime()-t0)/1e9, true);
    }

    
        {
            Int w=20,h=20;
            Map<String,Object> m = new HashMap<>();
            m.put(“name”,”open_var_20”); m.put(“w”,w); m.put(“h”,h);
            Map<Pos,Double> terrain = new HashMap<>();
            For (int i=0;i<50;i++) terrain.put(new Pos(rnd.nextInt(h), rnd.nextInt(w)), (rnd.nextBoolean()?1.0: (rnd.nextBoolean()?1.5:2.0)));
            Set<Pos> staticOb = new HashSet<>();
            For (int i=0;i<60;i++) staticOb.add(new Pos(rnd.nextInt(h), rnd.nextInt(w)));
            m.put(“terrain”,terrain); m.put(“static”,staticOb); m.put(“dynamic”,Collections.emptyList());
            maps.add(m);
        }

    
        {
            Int w=20,h=20;
            Map<String,Object> m = new HashMap<>();
            m.put(“name”,”cluttered_20”); m.put(“w”,w); m.put(“h”,h);
            Map<Pos,Double> terrain = new HashMap<>();
            Set<Pos> staticOb = new HashSet<>();
            For (int r=0;r<h;r++) for (int c=0;c<w;c++) if (rnd.nextDouble() < 0.25) staticOb.add(new Pos(r,c));
            m.put(“terrain”,terrain); m.put(“static”,staticOb); m.put(“dynamic”,Collections.emptyList());
            maps.add(m);
        }

        
        {
            Int w=20,h=20;
            Map<String,Object> m = new HashMap<>();
            m.put(“name”,”dynamic_avenue”); m.put(“w”,w); m.put(“h”,h);
            Set<Pos> staticOb = new HashSet<>();
            Map<Pos,Double> terrain = new HashMap<>();
            List<DynamicObstacle> dyn = new ArrayList<>();
            List<Pos> p1 = new ArrayList<>(); for (int c=0;c<w;c++) p1.add(new Pos(10,c));
            List<Pos> p2 = new ArrayList<>(); for (int c=w-1;c>=0;c--) p2.add(new Pos(10,c));
            Dyn.add(new DynamicObstacle(p1, 0));
            Dyn.add(new DynamicObstacle(p2, 5));
            m.put(“terrain”,terrain); m.put(“static”,staticOb); m.put(“dynamic”,dyn);
            maps.add(m);
        }
        }

        Return maps;
    }


    Static GridWorld buildGridFromSpec(Map<String,Object> spec) {
        Int w = (int)spec.get(“w”);
        Int h = (int)spec.get(“h”);
        GridWorld g = new GridWorld(h, w);
        SuppressWarnings(“unchecked”)
        Map<Pos,Double> terrain = (Map<Pos,Double>)spec.get(“terrain”);
        If (terrain != null) {
            For (Map.Entry<Pos,Double> e : terrain.entrySet()) {
                Pos p = e.getKey(); if (g.inBounds(p)) g.terrain[p.r][p.c] = e.getValue();
            }
        }
        @SuppressWarnings(“unchecked”)
        Set<Pos> staticOb = (Set<Pos>)spec.get(“static”);
        If (staticOb != null) {
            For (Pos p: staticOb) if (g.inBounds(p)) g.blocked[p.r][p.c] = true;
        }
        @SuppressWarnings(“unchecked”)
        List<DynamicObstacle> dyn = (List<DynamicObstacle>)spec.get(“dynamic”);
        If (dyn != null) {
            g.dynamic.addAll(dyn);
        }
        Return g;
    }

   
    Static Pos sampleFree(GridWorld g) {
        For (int attempt=0; attempt<10000; attempt++) {
            Int r = ThreadLocalRandom.current().nextInt(g.height);
            Int c = ThreadLocalRandom.current().nextInt(g.width);
            Pos p = new Pos(r,c);
            If (g.passable(p)) return p;
        }
        Throw new RuntimeException(“Could not find free cell”);
    }

    Static class ExperimentRow {
        String mapName, planner;
        Pos start, goal;
        Int initialPathLen;
        Double planCost;
        Double planTime;
        Int planNodes;
        Double execCost;
        Double execTime;
        Int execNodes;
        Int replans;
        Boolean success;
        ExperimentRow(String mapName, String planner, Pos start, Pos goal) {
            This.mapName = mapName; this.planner = planner; this.start = start; this.goal = goal;
        }
    }

    Public static void main(String[] args) {
        List<Map<String,Object>> specs = makeTestMaps();
        List<ExperimentRow> rows = new ArrayList<>();
        For (Map<String,Object> spec : specs) {
            GridWorld g = buildGridFromSp

