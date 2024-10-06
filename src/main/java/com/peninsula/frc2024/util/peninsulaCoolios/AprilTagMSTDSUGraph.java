package com.peninsula.frc2024.util.peninsulaCoolios;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.*;

public class AprilTagMSTDSUGraph {
  public final ArrayList<ArrayList<Edge>> adjacencyList; // Directed Graph
  private final ArrayList<Edge> allEdges;
  private final int size;
  private final double distrustThreshold;

  public ArrayList<ArrayList<Edge>> adjacencyListMST; // Non directed tree
  private final int[] unionBelongsTo;
  private final int[] unionParent;
  private final Pose3d[] allTagPoses;
  private final double[][] dist;
  private final HashMap<Integer, Pose3d> poseIds = new HashMap<>();

  private ArrayList<Integer> roots = new ArrayList<>();
  private final int[] disjointSetMinDistIndex;
  private final int[] distToAll;

  public AprilTagMSTDSUGraph(int size, int distrustThreshold) {

    this.size = size;
    this.distrustThreshold = (distrustThreshold == -1) ? Double.MAX_VALUE : distrustThreshold;

    adjacencyList = new ArrayList<>(size);
    allEdges = new ArrayList<>();
    allTagPoses = new Pose3d[size];
    unionBelongsTo = new int[size];
    unionParent = new int[size];
    dist = new double[size][size];

    disjointSetMinDistIndex = new int[size];
    distToAll = new int[size];

    for (int i = 0; i < size; i++) adjacencyList.add(new ArrayList<>());
  }

  /** Kruskal's Algorithm for minimum spanning tree (MST) using Disjoint Set Union (DSU) */
  public void MST() {
    adjacencyListMST = new ArrayList<>(size);
    for (int i = 0; i < size; i++) {
      adjacencyListMST.add(new ArrayList<>());
      unionBelongsTo[i] = i;
      unionParent[i] = i;
    }

    Collections.sort(allEdges);

    for (int i = 0; i < allEdges.size(); i++) {
      Edge candidate = allEdges.get(i);

      if (findParent(candidate.tagId_from) != findParent(candidate.tagId_to)) {

        unionParent[findParent(candidate.tagId_to)] = findParent(candidate.tagId_from);

        adjacencyListMST.get(candidate.tagId_from).add(candidate);
        adjacencyListMST.get(candidate.inverse().tagId_from).add(candidate.inverse());
      }
    }
    for (int i = 0; i < size; i++) {
      unionBelongsTo[i] = findParent(i);
    }
  }

  /**
   * Recursively finds the parents of a node in the unionParent array
   *
   * @param n node to find parent of
   * @return the top level parent of node n
   */
  public int findParent(int n) {
    if (unionParent[n] == n) return n;
    return findParent(unionParent[n]);
  }

  /**
   * Starting a BFS flood fill at each root, at each edge step the Tranform3d is added to the
   * from-node to get the pose of the to-node
   */
  public void fillPoses() {
    Queue<Integer> q = new LinkedList<>();

    Arrays.fill(allTagPoses, null);

    for (int root : roots) {
      if (!poseIds.containsKey(root)) {
        throw new RuntimeException("Root pose " + root + " not found in hashMap");
      } else {
        allTagPoses[root] = poseIds.get(root);
        q.add(root);
      }
    }

    while (!q.isEmpty()) {
      int node = q.poll();
      for (Edge e : adjacencyListMST.get(node)) {
        if (allTagPoses[e.tagId_to] == null) {
          q.add(e.tagId_to);
          allTagPoses[e.tagId_to] = allTagPoses[e.tagId_from].transformBy(e.transform);
        }
      }
    }
  }

  /**
   * Using the data from the floyd warshall computation (distance to all other nodes) and the union
   * each node belongs to, each disjoint graph's best root is solved for interatively
   */
  public void findRootsMinDist() {
    fillDistFloydWarshall();

    roots.clear();

    Arrays.fill(disjointSetMinDistIndex, Integer.MAX_VALUE);

    for (int i = 0; i < size; i++) {
      for (int j = 0; j < size; j++) {
        if (dist[i][j] != Integer.MAX_VALUE) distToAll[i] += dist[i][j];
      }
    }

    // Iteration through the DSU and find root for each union
    for (int i = 0; i < size; i++) {
      int union = unionBelongsTo[i];
      if (disjointSetMinDistIndex[union] == Integer.MAX_VALUE) {
        // First of union
        disjointSetMinDistIndex[union] = i;
      } else {
        if (distToAll[i] < distToAll[disjointSetMinDistIndex[union]]) {
          disjointSetMinDistIndex[union] = i;
        }
      }
    }

    for (int i = 0; i < size; i++) {
      if (disjointSetMinDistIndex[unionBelongsTo[i]] != Integer.MAX_VALUE
          && !roots.contains(disjointSetMinDistIndex[unionBelongsTo[i]])) {
        if (poseIds.containsKey(disjointSetMinDistIndex[unionBelongsTo[i]]))
          roots.add(disjointSetMinDistIndex[unionBelongsTo[i]]);
      }
    }
  }

  /** Floyd-Warshall algorithm to fill min dist array to query for best roots */
  private void fillDistFloydWarshall() {
    for (int i = 0; i < size; i++) {
      for (int j = 0; j < size; j++) {
        dist[i][j] = Integer.MAX_VALUE;
        if (i == j) dist[i][j] = 0;
      }
    }
    for (int i = 0; i < size; i++) {
      for (Edge d : adjacencyListMST.get(i)) {
        dist[i][d.tagId_to] = d.weight;
      }
    }
    for (int k = 0; k < size; k++) {
      for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
          if (dist[i][j] > dist[i][k] + dist[k][j]) {
            dist[i][j] = dist[i][k] + dist[k][j];
          }
        }
      }
    }
  }

  public ArrayList<Integer> getRoots() {
    return roots;
  }

  public Pose3d[] getAllTagPoses() {
    return allTagPoses;
  }

  public void setRoots(ArrayList<Integer> roots) {
    this.roots = roots;
  }

  public void addPoseForRooting(AprilTag tag) {
    poseIds.put(tag.ID, tag.pose);
  }

  /**
   * Adds an observation to the overall graph
   *
   * @param tagId_from the first tag observed
   * @param tagId_to the second tag observed
   * @param transform3d the transform to get from first tag to second tag
   * @param distrust Measure of how inaccurate this observation could be
   */
  public void addEdge(int tagId_from, int tagId_to, Transform3d transform3d, double distrust) {
    if (distrust > distrustThreshold) return;

    Edge newEdge = new Edge(distrust, transform3d, tagId_to, tagId_from);

    adjacencyList.get(tagId_from).add(newEdge);
    allEdges.add(newEdge);
  }

  public class Edge implements Comparable {
    private final double weight;
    private final int tagId_to;
    private final int tagId_from;
    private final Transform3d transform;

    public Edge(double weight, Transform3d transform, int tagId_to, int tagId_from) {
      this.weight = weight;
      this.transform = transform;
      this.tagId_to = tagId_to;
      this.tagId_from = tagId_from;
    }

    public double getWeight() {
      return weight;
    }

    public int getTagId_to() {
      return tagId_to;
    }

    public int getTagId_from() {
      return tagId_from;
    }

    public Edge inverse() {
      return new Edge(weight, transform.inverse(), tagId_from, tagId_to);
    }

    @Override
    public String toString() {
      return "Edge{"
          + "weight="
          + weight
          + ", tagId_to="
          + tagId_to
          + ", tagId_from="
          + tagId_from
          + ", transform="
          + transform
          + '}';
    }

    @Override
    public int compareTo(Object o) {
      return (int) (100 * this.weight - 100 * ((Edge) o).weight);
    }
  }
}
