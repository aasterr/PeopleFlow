import networkx as nx
import pickle
import os
import math
 
def dist(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
 
def generate_corridor_graph():
    G = nx.Graph()
 
    # Coordinate dei waypoint (devono corrispondere al corridor.xml)
    waypoints = {
        "WP_BOTTOM": (0.0, -8.5),
        "WP_CROSS":  (0.0,  0.0),
    }
 
    # Aggiunta nodi
    for name, pos in waypoints.items():
        G.add_node(name, pos=pos)
 
    # Connessione diretta lungo il braccio verticale
    G.add_edge(
        "WP_BOTTOM",
        "WP_CROSS",
        weight=dist(waypoints["WP_BOTTOM"], waypoints["WP_CROSS"])
    )
 
    # Salvataggio nella stessa cartella dello script
    output_dir = os.path.dirname(os.path.abspath(__file__))
    output_file = os.path.join(output_dir, "graph.pkl")
 
    with open(output_file, 'wb') as f:
        pickle.dump(G, f)
 
    print(f"Grafo 'corridor' generato con successo in: {output_file}")
    print(f"  Nodi: {list(G.nodes(data=True))}")
    print(f"  Archi: {list(G.edges(data=True))}")
 
if __name__ == "__main__":
    generate_corridor_graph()
