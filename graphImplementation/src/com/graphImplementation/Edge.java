package com.graphImplementation;

public class Edge  implements Comparable<Edge> {

	public int src, dest, weight; 

	public Edge() {
		
	}
	 
    public Edge(int src, int dest, int weight) {
		super();
		this.src = src;
		this.dest = dest;
		this.weight = weight;
	}


	public int compareTo(Edge compareEdge) 
    { 
        return this.weight-compareEdge.weight; 
    } 

}
