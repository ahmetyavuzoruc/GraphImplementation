package com.graphImplementation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Stack;

public class Graph {

	private int V, E;
	private int[][] dir;
	private int in[];
	private LinkedList<Integer> adj[];
	Edge edge[];
	public List<List<Edge>> adjList = null;

	public Graph(int v) {
		V = v;
		adj = new LinkedList[v];
		in = new int[V];
		for (int i = 0; i < v; ++i) {
			adj[i] = new LinkedList();
			in[i] = 0;
		}
	}

	public Graph(int v, int e) {
		this.V = v;
		this.E = e;
		dir = new int[v][];
		for (int i = 0; i < v; i++) {
			dir[i] = new int[v];
		}

		edge = new Edge[E];
		for (int i = 0; i < e; ++i) {
			edge[i] = new Edge();
		}

	}
	
	
	public Graph(List<Edge> edges, int N)
	{
		adjList = new ArrayList<>(N);

		for (int i = 0; i < N; i++) {
			adjList.add(i, new ArrayList<>());
		}

		for (Edge edge: edges) {
			adjList.get(edge.src).add(edge);
		}
	}

	public void addEdge(int v, int w) {
		adj[v].add(w);
		in[w]++;
	}

	public static int findVertexDegree(Graph G, int v) {
		int degree = 0;
		for (int i = 0; i < G.V; i++) {
			if (G.dir[v][i] == 1)
				degree++;
		}
		return degree;
	}

	public void BFS(int s) {

		boolean visited[] = new boolean[V];
		BFSUtil(s, visited);
	}

	void DFS(int v) {

		boolean visited[] = new boolean[V];
		DFSUtil(v, visited);
	}

	void BFSUtil(int s, boolean visited[]) {

		visited[s] = true;
		LinkedList<Integer> queue = new LinkedList<Integer>();

		queue.add(s);

		while (queue.size() != 0) {

			s = queue.poll();
			System.out.print(s + " ");

			Iterator<Integer> i = adj[s].listIterator();
			while (i.hasNext()) {
				int n = i.next();
				if (!visited[n]) {
					visited[n] = true;
					queue.add(n);
				}
			}
		}
	}

	void DFSUtil(int v, boolean visited[]) {

		visited[v] = true;
		System.out.print(v + " ");

		Iterator<Integer> i = adj[v].listIterator();
		while (i.hasNext()) {
			int n = i.next();
			if (!visited[n])
				DFSUtil(n, visited);
		}
	}

	public Graph getTranspose() {
		Graph g = new Graph(V);
		for (int v = 0; v < V; v++) {
			Iterator<Integer> i = adj[v].listIterator();
			while (i.hasNext())
				g.adj[i.next()].add(v);
			(g.in[v])++;
		}
		return g;
	}

	public void fillOrder(int v, boolean visited[], Stack stack) {

		visited[v] = true;

		Iterator<Integer> i = adj[v].iterator();
		while (i.hasNext()) {
			int n = i.next();
			if (!visited[n])
				fillOrder(n, visited, stack);
		}

		stack.push(new Integer(v));
	}

	public void printSCCs() {
		Stack stack = new Stack();

		boolean visited[] = new boolean[V];
		for (int i = 0; i < V; i++)
			visited[i] = false;

		for (int i = 0; i < V; i++)
			if (visited[i] == false)
				fillOrder(i, visited, stack);

		Graph gr = getTranspose();

		for (int i = 0; i < V; i++)
			visited[i] = false;

		while (stack.empty() == false) {
			int v = (int) stack.pop();

			if (visited[v] == false) {
				gr.DFSUtil(v, visited);
				System.out.println();
			}
		}
	}

	Boolean isSC() {

		boolean visited[] = new boolean[V];
		for (int i = 0; i < V; i++)
			visited[i] = false;

		DFSUtil(0, visited);

		for (int i = 0; i < V; i++)
			if (visited[i] == false)
				return false;

		Graph gr = getTranspose();

		for (int i = 0; i < V; i++)
			visited[i] = false;

		gr.DFSUtil(0, visited);

		for (int i = 0; i < V; i++)
			if (visited[i] == false)
				return false;

		return true;
	}

	Boolean isEulerianCycle() {

		if (isSC() == false)
			return false;

		for (int i = 0; i < V; i++)
			if (adj[i].size() != in[i])
				return false;

		return true;
	}

	public int find(Subset subsets[], int i) {

		if (subsets[i].parent != i)
			subsets[i].parent = find(subsets, subsets[i].parent);

		return subsets[i].parent;
	}

	public void Union(Subset subsets[], int x, int y) {
		int xroot = find(subsets, x);
		int yroot = find(subsets, y);

		if (subsets[xroot].rank < subsets[yroot].rank)
			subsets[xroot].parent = yroot;
		else if (subsets[xroot].rank > subsets[yroot].rank)
			subsets[yroot].parent = xroot;

		else {
			subsets[yroot].parent = xroot;
			subsets[xroot].rank++;
		}
	}

	public void KruskalMST() {
		Edge result[] = new Edge[V];
		int e = 0;
		int i = 0;
		for (i = 0; i < V; ++i)
			result[i] = new Edge();

		Arrays.sort(edge);

		Subset subsets[] = new Subset[V];
		for (i = 0; i < V; ++i)
			subsets[i] = new Subset();

		for (int v = 0; v < V; ++v) {
			subsets[v].parent = v;
			subsets[v].rank = 0;
		}

		i = 0;
		while (e < V - 1) {

			Edge next_edge = new Edge();
			next_edge = edge[i++];

			int x = find(subsets, next_edge.src);
			int y = find(subsets, next_edge.dest);
			if (x != y) {
				result[e++] = next_edge;
				Union(subsets, x, y);
			}
		}

		System.out.println("Kruskal’s Minimum Spanning Tree : ");
		for (i = 0; i < e; ++i)
			System.out.println(result[i].src + " -- " + result[i].dest + " == " + result[i].weight);
	}
	
	
	private static void getRoute(int prev[], int i, List<Integer> route)
	{
		if (i >= 0) {
			getRoute(prev, prev[i], route);
			route.add(i);
		}
	}

	public static void shortestPath(Graph graph, int source, int N)
	{
		
		PriorityQueue<Node> minHeap;
		minHeap = new PriorityQueue<>(Comparator.comparingInt(node -> node.weight));
		minHeap.add(new Node(source, 0));

		List<Integer> dist = new ArrayList<>(Collections.nCopies(N, Integer.MAX_VALUE));

		dist.set(source, 0);

		boolean[] done = new boolean[N];
		done[source] = true;

		int prev[] = new int[N];
		prev[source] = -1;

		List<Integer> route = new ArrayList<>();

		while (!minHeap.isEmpty())
		{
			
			Node node = minHeap.poll();

			int u = node.vertex;

			for (Edge edge: graph.adjList.get(u))
			{
				int v = edge.dest;
				int weight = edge.weight;

				if (!done[v] && (dist.get(u) + weight) < dist.get(v))
				{
					dist.set(v, dist.get(u) + weight);
					prev[v] = u;
					minHeap.add(new Node(v, dist.get(v)));
				}
			}

			done[u] = true;
		}

		for (int i = 1; i < N; ++i)
		{
			if (i != source && dist.get(i) != Integer.MAX_VALUE) {
				getRoute(prev, i, route);
				System.out.printf("Path (%d -> %d): Minimum Cost = %d and Route is %s\n",
								source, i, dist.get(i), route);
				route.clear();
			}
		}

	}
	public static void main(String args[]) {

		Graph g = new Graph(5);

		g.addEdge(0, 1);
		g.addEdge(0, 2);
		g.addEdge(1, 2);
		g.addEdge(2, 0);
		g.addEdge(2, 3);
		g.addEdge(3, 3);

		Graph G = new Graph(6, 7);

		G.dir[0][1] = 1;
		G.dir[0][2] = 1;
		G.dir[0][3] = 1;
		G.dir[0][4] = 1;
		G.dir[0][5] = 1;

		G.dir[1][0] = 1;
		G.dir[1][2] = 1;
		G.dir[1][3] = 1;
		G.dir[1][4] = 1;
		G.dir[1][5] = 1;

		G.dir[2][0] = 1;
		G.dir[2][1] = 1;
		G.dir[2][3] = 1;
		G.dir[2][4] = 1;
		G.dir[2][5] = 1;

		G.dir[3][0] = 1;
		G.dir[3][1] = 1;
		G.dir[3][2] = 1;
		G.dir[3][4] = 1;
		G.dir[3][5] = 1;

		G.dir[4][0] = 1;
		G.dir[4][1] = 1;
		G.dir[4][2] = 1;
		G.dir[4][3] = 1;
		G.dir[4][5] = 1;

		G.dir[5][0] = 1;
		G.dir[5][1] = 1;
		G.dir[5][2] = 1;
		G.dir[5][3] = 1;
		G.dir[5][4] = 1;

		Graph graph = new Graph(4, 5);

		graph.edge[0].src = 0;
		graph.edge[0].dest = 1;
		graph.edge[0].weight = 10;

		graph.edge[1].src = 0;
		graph.edge[1].dest = 2;
		graph.edge[1].weight = 6;

		graph.edge[2].src = 0;
		graph.edge[2].dest = 3;
		graph.edge[2].weight = 5;

		graph.edge[3].src = 1;
		graph.edge[3].dest = 3;
		graph.edge[3].weight = 15;

		graph.edge[4].src = 2;
		graph.edge[4].dest = 3;
		graph.edge[4].weight = 4;

		
		List<Edge> edges = Arrays.asList
				(
				new Edge(0, 1, 10), 
				new Edge(0, 4, 3),
				new Edge(1, 2, 2), 
				new Edge(1, 4, 4),
				new Edge(2, 3, 9), 
				new Edge(3, 2, 7),
				new Edge(4, 1, 1), 
				new Edge(4, 2, 8),
				new Edge(4, 3, 2)
		);

		Graph gra = new Graph(edges, 5);
		
		
		System.out.println("Degree Of Vertex : " + findVertexDegree(G, 0));
		System.out.println();
		System.out.println("---------------");

		System.out.print("Breadth First Traversal : ");
		g.BFS(2);
		System.out.println();
		System.out.println();
		System.out.println("---------------");

		System.out.print("Depth First Traversal : ");
		g.DFS(2);
		System.out.println();
		System.out.println();
		System.out.println("---------------");

		graph.KruskalMST();
		System.out.println();
		System.out.println("---------------");
		
		System.out.println("Shortest Path : ");
		shortestPath(gra, 0, 5);
		System.out.println();
		System.out.println("---------------");
			
		System.out.println("Strongly connected components : ");
		g.printSCCs();
		System.out.println();
		System.out.println("---------------");

		if (g.isEulerianCycle())
			System.out.println("Given directed graph is eulerian ");
		else
			System.out.println("Given directed graph is NOT eulerian ");

	}

}
