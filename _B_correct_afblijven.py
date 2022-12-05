#Authors: Group 36 of QML

from gurobipy import *
import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
import networkx as nx

model = Model ('TSP Problem')



# ---- Parameters ----
file = pd.read_csv('data_small.txt', header = None, delim_whitespace=True)  # Load small dataset
#file = pd.read_csv('data_large.txt', header = None, delim_whitespace=True)  # Load large dataset

node = file[0].tolist()
x_loc = file[1].tolist()
y_loc = file[2].tolist()
Q = file[3].tolist() #demand (Quantity)
rT = file[4].tolist()
dT = file[5].tolist()
sT = file[6].tolist()

eps = 0.0001
M = 100000 + eps  #nog te bepalen

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
        x[i,j] = model.addVar(vtype = GRB.BINARY, name = 'X[' + str(i) + ',' + str(j) + ']')

T = {}
for i in N:
    T[i]=model.addVar(lb = 0, vtype=GRB.CONTINUOUS, name="T["+str(i)+"]")

model.update ()


# ---- Objective Function ----
model.setObjective (quicksum(d[i,j] * x[i,j] for i in N for j in N))
model.modelSense = GRB.MINIMIZE
model.update ()


# ---- Constraints ----
con1 = {} #ensures that every node is visited exactly once.
for j in N:
    con1[j] = model.addConstr(quicksum(x[i,j] for i in N) == 1)

con2 = {} #ensures that every node is leaved exactly once.
for i in N:
    con2[i] = model.addConstr(quicksum(x[i,j] for j in N) == 1)
    
## Arrival time constraints
con3 = {}
con4 = {}
con5 = {}
for i in range(0,len(N)): # this works if one of the two for loops has 0, not when both have 0 
    con3[i] = model.addConstr(rT[i] <= T[i]) # arrival time later than ready time
    con4[i] = model.addConstr(T[i] <= dT[i]) # arrival time before due time
    for j in range(1,len(N)):
        con5[i,j] = model.addConstr(T[j] >= T[i] + sT[i] + d[i,j] - M*(1-x[i,j]))


# ---- Solve ----
model.setParam( 'OutputFlag', True) # silencing gurobi output or not
model.setParam ('MIPGap', 0);       # find the optimal solution
model.write("output.lp")            # print the model in .lp format file

model.optimize ()



# --- Print results ---
print ('\n--------------------------------------------------------------------\n')
    
if model.status == GRB.Status.OPTIMAL: # If optimal solution is found
    print ('Optimal solution: %10.2f time units' % model.objVal)
    print ('')
    
else:
    print ('\nNo feasible solution found')

print ('\nREADY\n')
print ('Distance matrix:')

s = '%8s' % ''
for j in range(len(N)):
    s = s + '%8s' % N[j]
print (s)    

for i in range(len(N)):
    s = '%8s' % N[i]
    for j in range(len(N)):
            s = s + '%8.1f' % d[i,j]
    s = s + '%8.1f' % sum (d[i,j] for j in N)   
    print(s)

u = '%8s' % ''
for j in range(len(N)):
    u = u + '%8.1f' % sum (d[i,j] for i in N)      
print(u)



print('')
print ('Route matrix:')
s = '%8s' % ''
for j in range(len(N)):
    s = s + '%8s' % N[j]
print (s)    

for i in range(len(N)):
    s = '%8s' % N[i]
    for j in range(len(N)):
            s = s + '%8.1f' % x[i,j].x
    s = s + '%8.1f' % sum (x[i,j].x for j in N)   
    print(s)

u = '%8s' % ''
for j in range(len(N)):
    u = u + '%8.1f' % sum (x[i,j].x for i in N)      
print(u)



print ('')
print ('Arrival times: \n')
stored = []
print('%8s' % 'Node' + '%8s' % 'Time' + '%8s' % 'Demand')
for i in range(len(N)):
    tim = '%8s' % N[i] + '%8.1f' % T[i].x + '%8s' % Q[i]
    stored.append(T[i].x)
    print(tim)



nodes = node
zipped_ns = zip(nodes,stored)
new = list(zipped_ns)
res = sorted(new, key = operator.itemgetter(1))

print('')
print ('Arrival times sorted & including load: \n')

stored = []
total_load = sum(Q)
print('%8s' % 'Node' + '%8s' % 'Time' + '%8s' % 'Demand' + '%8s' % 'Load')
for i in range(len(res)):
    total_load += -Q[res[i][0]]
    tim = '%8s' % res[i][0] + '%8.1f' % res[i][1] + '%8s' % Q[res[i][0]] + '%8.1f' % total_load
    print(tim)

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
    color_map.append('green')
        
plt.figure(3,figsize=(15,15)) 
nx.draw_networkx_nodes(G, pos, node_color=color_map, node_size=2000)

nx.draw_networkx_edges(G, pos, edgelist=active_arcs, edge_color='k')

nx.draw_networkx_labels(G, pos, labels=labels, font_weight='bold', font_family="sans-serif", font_size=20)

plt.title("Solution part B", fontweight='bold')