from gurobipy import *
import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
import networkx as nx

model = Model("TSP_model")

# ---- Parameters ----

# Load data
file = pd.read_csv("data_b.txt", delim_whitespace=True, header = None)

node = file[0].tolist()
# Parameters
xloc = file[1].tolist() 
yloc = file[2].tolist() 
D = file[3].tolist() 
RT = file[4].tolist() 
DT = file[5].tolist() 
ST = file[6].tolist() 
PID = file[7].tolist() 
DID = file[8].tolist() 
q = 200
M = 10000

#Cost parameters for question E
#DepotCosts = [150,100]
#DepotCosts = [0,0]

# pick up locations
P = [i for i in PID if i > 0]
Z = [i for i in DID if i > 0]
nodeset = len(node)

depots = []
clients = []
for i in range(len(D)):
    if D[i] == 0:
        depots.append(i)
    else:
        clients.append(i)
        
for i in depots:
    node = node +[len(file[0])+i]
    xloc = xloc + [file[1][i]]
    yloc = yloc + [file[2][i]]
    D = D + [file[3][i]]
    RT = RT + [file[4][i]]
    DT = DT + [file[5][i]]
    ST = ST + [file[6][i]]
    
depotsset = []
for i in range(len(D)):
    if D[i] == 0:
        depotsset.append(i)

# ---- Indices and Sets ---- 
nodes = len(node)
vehiclenr = 1

vehicles = np.ones(vehiclenr)
    
        
N = range(nodes)
K = range(vehiclenr)
 

# Creating distance parameter from xloc and yloc
d = np.zeros((nodes, nodes))
for i in N:
    for j in N:
        d[i, j] = np.sqrt(((xloc[j] - xloc[i]) ** 2) + ((yloc[j] - yloc[i]) ** 2))

# ---- Decision variables ----
x = {}
T = {}
Q = {}

# xijk binary decision variable indicating if a vehicle will visit node j directly after node i
for i in N:
    for j in N:
        for k in K:
            x[i,j,k]=model.addVar(vtype=GRB.BINARY, name="X["+str(i)+","+str(j)+","+str(k)+"]")

# Tik arrival time of vehicle k at location i
for i in N:
    for k in K:
        T[i,k]=model.addVar(lb = 0, vtype=GRB.CONTINUOUS, name="T["+str(i)+","+str(k)+"]")

# Qik capacity of vehicle k at location i
for i in N:
    for k in K:
        Q[i,k] = model.addVar(lb = 0, vtype=GRB.CONTINUOUS, name="T["+str(i)+","+str(k)+"]")

# --- Variable ---

# Waiting time for vehicle k at node i
WT = {}
for i in N:
    for k in K:
        WT[i,k] = RT[i] - T[i,k]

# Integrate new variables
model.update ()

# ---- Objective Function ----

model.setObjective (quicksum (d[i,j]*x[i,j,k] for i in N for j in N for k in K))
model.modelSense = GRB.MINIMIZE
model.update ()

# ---- Constraints ----

#constraint 1 all pickups nodes must be visited, and must be visited exactly once. 
con1 = {}
for i in P:
    con1[i] = model.addConstr(quicksum(x[i, j, k] for k in K for j in N) == 1)
  
#constraint 2 ensures that only pickup nodes will be visited
con2 = {}
for i in Z:
    con2[i] = model.addConstr(quicksum(x[i, j, k] for k in K for j in N) == 0)

#constraint 3 flow constraint, makes sure that each vehicle k visiting a client will also leave this client.
con3 = {}
for i in clients:
    for k in K:
        con3[i, k] = model.addConstr(quicksum(x[j, i, k] for j in N) - quicksum(x[i, j, k] for j in N) == 0 )
 
#constraint 4: first time constraint, indicating that the arrival time at the node following the current node must be greater/equal to the arrival time at the current node plus service time and the distance.
con4 ={}
for i in N:
    for j in N:
        for k in K:
            con4[i, j, k] = model.addConstr(T[j, k] >= T[i, k] + ST[i] + d[i, j] - M * (1 - x[i, j, k]))
       
#constraint 5: the second time constraint which ensures that the arrival time of vehicle k at node i must be greather than or equal to the ready time of node i, and smaller than or equal to the due time of node i. 
con51 = {}
for i in N:
    for k in K:
        con51[i, k] = model.addConstr(T[i, k] >= RT[i])           
con52 = {}
for i in N:
    for k in K:
        con52[i, k] = model.addConstr(T[i, k] <= DT[i])
   
#constraint 6: capacity constraints indicating that the capacity at the next node j must be greater than or equal to the capacity at the current node i, plus the demand at the next node j. 
con61 ={}
for i in N:
    for j in clients:
        for k in K:
            con61[i, j, k] = model.addConstr(Q[i, k] + D[j] - Q[j, k] <= M * (1 - x[i, j, k]))
con62 ={}
for i in N:
    for j in clients:
        for k in K:
            con62[i, j, k] = model.addConstr(Q[i, k] + D[j] - Q[j, k] >= - M * (1 - x[i, j, k]))

#constraint 7: the capacity of the vehicles cannot be exceeded 
con7 = {}
for k in K:
    for i in clients:
        con7[k,i] = model.addConstr(Q[i,k] <= q)
        
#constraint 8: the capacity used at the depots is equal to zero. 
con8 = {}
for k in K:
    for i in depotsset:
        con8[k,i] = model.addConstr(Q[i,k] == 0)
        
#constraint 9: the number of times a depot is left, is equal to the amount of vehicles
con9 = {}
for k in K:
    con9[k] = model.addConstr(quicksum(x[i,j,k] for i in depotsset for j in N) == vehicles[k])

     
# ---- Solve ----

model.setParam( 'OutputFlag', True) # silencing gurobi output or not
model.setParam ('MIPGap', 0);       # find the optimal solution
model.write("output.lp")            # print the model in .lp format file

model.optimize ()
'''
# --- Print results ---
total_distance = 0

for i in N:
    for j in N:
        for k in K:
            total_distance = total_distance + d[i,j] * x[i,j,k].x
        
print('The total distance travelled is: ' + str(total_distance))
print('The total cost is: ' + str(round(model.objVal,2)))

for k in K:
    z = 'Vehicle %d' %(k+1)
    for i in N:
        z = z + '%8s' %node[i]
    print(z)
    for j in N:
        z = '%8s' %node[j]
        for i in N:
            z = z + '%8.3f' % x[i,j,k].x
        print(z)
        
for k in K:
    z = 'Vehicle %d' %(k+1)
    for i in N:
        z = z + '%8s' %y[i,k].x
    print(z)
'''

# --- Visualization ---
G = nx.DiGraph()

arcs = [(i,j) for i in N for j in N]
active_arcs = [(i,j,k) for i in N for j in N for k in K if x[i,j,k].x>0.01]

G.add_edges_from(arcs)

pos = {node[i]: (xloc[i],yloc[i]) for i in N}
label_list = list(range(nodes)) + [""]
labels = {node[i]: label_list[i] for i in N}

color_map = []
for i in G:
    if i in P:
        color_map.append('blue')
    elif i == 0 or i == 1:
        color_map.append('red') 
    else: 
        color_map.append('green')
        
plt.figure(3,figsize=(15,15)) 
nx.draw_networkx_nodes(G, pos, node_color=color_map, node_size=2000)

nx.draw_networkx_edges(G, pos, edgelist=active_arcs, edge_color='k')

nx.draw_networkx_labels(G, pos, labels=labels, font_weight='bold', font_family="sans-serif", font_size=20)

plt.title("Solution part B", fontweight='bold')
plt.axis("off")
plt.savefig('solution_part_D.png')

# --- Print results ---
total_distance = 0

for i in N:
    for j in N:
        for k in K:
            total_distance = total_distance + d[i,j] * x[i,j,k].x
        
print('The total distance travelled is: ' + str(round(total_distance,3)))
print('The total cost is: ' + str(round(model.objVal,2)))


vehicle = []
location = [] 
time = []
load = []

for k in K:
    nodes_vehicle = [i for i in N for j in N if x[i,j,k].x == 1]
    nodes_vehicle = nodes_vehicle + [i for i in depots for j in N if x[j,i,k].x==1]
    
    for i in nodes_vehicle:
        vehicle.append(k)
        location.append(i)
        time.append(round(T[i,k].x,0))
        load.append(Q[i,k].x)

results = pd.DataFrame(np.array([vehicle, location, time, load]).T,columns=['Vehicle','Location','Time','Load'])
results = results.sort_values(by=['Vehicle', 'Time'])
print(results.to_latex(index=False)) 

"""
print('Total time is'  %(sum(T[i,k].x for i in N for k in K)) )
print('')
"""

for k in K:
    print ('\n--------------------------------------------------------------------\n')
    print('Routing for vehicle ' + str(k))
    vehicle_results = results[(results['Vehicle'] == k)]
    print(vehicle_results)