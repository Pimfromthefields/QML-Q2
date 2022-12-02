#Authors: Group 36 of QML

from gurobipy import *
import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
import networkx as nx

model = Model ('TSP Problem')


# ---- Parameters ----


file = pd.read_csv('data_small.csv', delimiter = ';', header = None)  # Load small dataset
#file = pd.read_csv('data_large.csv', delimiter = ';', header = None) # Load big dataset

node = file[0].tolist()
x_loc = file[1].tolist()
y_loc = file[2].tolist()
Q = file[3].tolist() #demand (Quantity)
rT = file[4].tolist()
dT = file[5].tolist()
sT = file[6].tolist()

M = 100000 #nog te bepalen

# Creating distance parameter from xloc and yloc
d = np.zeros((len(node), len(node)))
N = range(len(node))
for i in N:
    for j in N:
        d[i, j] = np.sqrt(((x_loc[j] - x_loc[i]) ** 2) + ((y_loc[j] - y_loc[i]) ** 2))

# ---- Sets ----
N = range(len(node)) # this set is already defined above, but presented here again for completeness.

# ---- Decision Variables ----

x = {}  
for i in N:
    for j in N:
        x[i,j] = model.addVar (vtype = GRB.BINARY, name = 'X[' + str(i) + ',' + str(j) + ']')

T = {}
for i in N:
    T[i]=model.addVar(lb = 0, vtype=GRB.CONTINUOUS, name="T["+str(i)+"]")

model.update ()


# ---- Objective Function ----
model.setObjective ((quicksum (d[i,j] * x[i,j] for i in N for j in N)))
model.modelSense = GRB.MINIMIZE
model.update ()


# ---- Constraints ----
con1 = {} #ensures that every node is visited exactly once.
for j in N:
    con1[j] = model.addConstr(quicksum(x[i,j] for i in N) == 1)

con2 = {} #ensures that every node is leaved exactly once.
for i in N:
    con2[j] = model.addConstr(quicksum(x[i,j] for j in N) == 1)

#con3 = {} #Ensuring no subroutes


con4 = {}
for i in N:
    for j in N:
        if i != j:
            con4[i,j] = model.addConstr(T[j] >= T[i] + sT[i] + d[i,j] - M * (1 - x[i,j]))
        
con5 = {}
for i in N:
    con5[i] = model.addConstr(rT[i] <= T[i])
    
con6 = {}
for i in N:
    con6[i] = model.addConstr(T[i] <= dT[i])


# ---- Solve ----

model.setParam( 'OutputFlag', True) # silencing gurobi output or not
model.setParam ('MIPGap', 0);       # find the optimal solution
model.write("output.lp")            # print the model in .lp format file

model.optimize ()



# --- Print results ---
print ('\n--------------------------------------------------------------------\n')
    
if model.status == GRB.Status.OPTIMAL: # If optimal solution is found
    print ('Total costs: %10.2f euro' % model.objVal)
    print ('')
    
    total_distance = 0

    for i in N:
        for j in N:
            print(x[i,j].x)
            #total_distance = total_distance + d[i,j] * x[i,j].x
        
    #print('The total distance travelled is: ' + str(total_distance))
    #print('The total cost is: ' + str(round(model.objVal,2)))
    
    
    
else:
    print ('\nNo feasible solution found')

print ('\nREADY\n')

# --- Visualization ---
G = nx.DiGraph()

arcs = [(i,j) for i in N for j in N]
active_arcs = [(i,j) for i in N for j in N if x[i,j].x>0.01]

G.add_edges_from(arcs)

pos = {node[i]: (x_loc[i],y_loc[i]) for i in N}
label_list = node + [""]
labels = {node[i]: label_list[i] for i in N}

color_map = []
for i in N:
    color_map.append('blue')
        
plt.figure(3,figsize=(15,15)) 
nx.draw_networkx_nodes(G, pos, node_color=color_map, node_size=2000)

nx.draw_networkx_edges(G, pos, edgelist=active_arcs, edge_color='k')

nx.draw_networkx_labels(G, pos, labels=labels, font_weight='bold', font_family="sans-serif", font_size=20)

plt.title("Solution part B", fontweight='bold')