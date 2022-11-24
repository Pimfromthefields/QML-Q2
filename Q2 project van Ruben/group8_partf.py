from gurobipy import *
from gurobipy import GRB
import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
import networkx as nx

model = Model("TSP_model")

softlimit = 5
hardlimit = 5*60

def softtime(model, where):
    if where == GRB.Callback.MIP:
        runtime = model.cbGet(GRB.Callback.RUNTIME)
        objbst = model.cbGet(GRB.Callback.MIP_OBJBST)
        objbnd = model.cbGet(GRB.Callback.MIP_OBJBND)
        gap = abs((objbst - objbnd) / objbst)

        if runtime > softlimit and gap < 0.1:
            model.terminate()

# ---- Parameters ----

# Load data
file = pd.read_csv("data_f.txt", delim_whitespace=True, header = None)

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
q = 100
M = 100000
#DepotCosts = [150,100]
DepotCosts = [0,0]

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
    
DepotCosts = DepotCosts + [0]*len(clients) + DepotCosts
    
depotsset = []
for i in range(len(D)):
    if D[i] == 0:
        depotsset.append(i)

# Indices and Sets
nodes = len(node)
vehiclenr = 15

vehicles = np.ones(vehiclenr)
    
        
N = range(nodes)
K = range(vehiclenr)
 

# Creating distance parameter from xloc and yloc
d = np.zeros((nodes, nodes))
for i in N:
    for j in N:
        d[i, j] = np.sqrt(((xloc[j] - xloc[i]) ** 2) + ((yloc[j] - yloc[i]) ** 2))

# Decision variables
x = {}
y = {}
T = {}
Q = {}

# xijk binary decision variable
for i in N:
    for j in N:
        for k in K:
            x[i,j,k]=model.addVar(vtype=GRB.BINARY, name="X["+str(i)+","+str(j)+","+str(k)+"]")

# yik binary decision variable
for i in N:
    for k in K:
        y[i,k]=model.addVar(vtype=GRB.BINARY, name="Y["+str(i)+","+str(k)+"]")

# tik arrival time of vehicle k at location t
for i in N:
    for k in K:
        T[i,k]=model.addVar(lb = 0, vtype=GRB.CONTINUOUS, name="T["+str(i)+","+str(k)+"]")

for i in N:
    for k in K:
        Q[i,k] = model.addVar(lb = 0, vtype=GRB.CONTINUOUS, name="T["+str(i)+","+str(k)+"]")

# Variable

# waiting time
WT = {}
for i in N:
    for k in K:
        WT[i,k] = RT[i] - T[i,k]

# Integrate new variables
model.update ()

# ---- Objective Function ----

#model.setObjective (quicksum (d[i,j]*x[i,j,k] for i in N for j in N for k in K) + quicksum(quicksum(x[i,j,k] for k in K for j in N) * DepotCosts[i] for i in depots))
model.setObjectiveN (quicksum (d[i,j]*x[i,j,k] for i in N for j in N for k in K),0,1, weight=0.2)
model.setObjectiveN (quicksum(x[i,j,k] * DepotCosts[i] for k in K for i in depotsset for j in N),1,1, weight=1)
#model.setObjective (quicksum (d[i,j]*x[i,j,k] for i in N for j in N for k in K) + quicksum(x[i,j,k] * DepotCosts[i] for k in K for j in N for i in depots))
#

model.modelSense = GRB.MINIMIZE
model.update ()

# ---- Constraints ----

# Constraint 1: All pickup nodes must be visited only once
con1 = {}
for i in P:
    con1[i] = model.addConstr(quicksum(x[i, j, k] for k in K for j in N) == 1)

# Constraint 2: Ensures that only pickup nodes will be visited
con2 = {}
for i in Z:
    con2[i] = model.addConstr(quicksum(x[i, j, k] for k in K for j in N) == 0)

# Constraint 3: Flow constraint, each vehicle visiting a client will also leave
con3 = {}
for i in clients:
    for k in K:
        con3[i, k] = model.addConstr(quicksum(x[j, i, k] for j in N) - quicksum(x[i, j, k] for j in N) == 0 )
 
# Constraint 4: Time constraint of the arrival time
con4 ={}
for i in N:
    for j in N:
        for k in K:
            con4[i, j, k] = model.addConstr(T[j, k] >= T[i, k] + ST[i] + d[i, j] - M * (1 - x[i, j, k]))
       
# Constraint 41&42: Vehicles must arrive within the provided time limit
con41 = {}
for i in N:
    for k in K:
        con41[i, k] = model.addConstr(T[i, k] >= RT[i])           
con42 = {}
for i in N:
    for k in K:
        con42[i, k] = model.addConstr(T[i, k] <= DT[i])

# Constraint 51&52: Capacity constraint of node j
con51 ={}
for i in N:
    for j in clients:
        for k in K:
            con51[i, j, k] = model.addConstr(Q[i, k] + D[j] - Q[j, k] <= M * (1 - x[i, j, k]))
con52 ={}
for i in N:
    for j in clients:
        for k in K:
            con52[i, j, k] = model.addConstr(Q[i, k] + D[j] - Q[j, k] >= - M * (1 - x[i, j, k]))
 
# Constraint 53: Capacity of the vehicle can not be exceeded            
con53 = {}
for k in K:
    for i in clients:
        con53[k,i] = model.addConstr(Q[i,k] <= q)

# Constraint 54: The capacity at the depots is equal to zero         
con54 = {}
for k in K:
    for i in depotsset:
        con54[k,i] = model.addConstr(Q[i,k] == 0)
        
#constraint 6: The total number of times a depot is left is equal to the number of vehicles
con6 = {}
for k in K:
    con6[k] = model.addConstr(quicksum(x[i,j,k] for i in depotsset for j in N) == 1)

# Constraint 71 & 72: ensures that the start depot is the same as the end depot
con71 = {} 
for i in depots:
    for k in K:
        con71[i,k] = model.addConstr(quicksum(x[i,j,k] for j in N) - quicksum(x[j,(i+nodeset),k] for j in N)==0)
con72 = {}
for i in depots:
    for k in K:
        con72[i,k] = model.addConstr(quicksum(x[(i+nodeset),j,k] for j in N) - quicksum(x[j,i,k] for j in N)==0)

# ---- Solve ----

model.setParam( 'OutputFlag', True) # silencing gurobi output or not
model.setParam ('MIPGap', 0);       # find the optimal solution
model.write("output.lp")            # print the model in .lp format file

#model.setParam('TimeLimit', hardlimit)
model.optimize()

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
nx.draw_networkx_nodes(G, pos, node_color=color_map, node_size=500)

nx.draw_networkx_edges(G, pos, edgelist=active_arcs, edge_color='k')

nx.draw_networkx_labels(G, pos, labels=labels, font_weight='bold', font_family="sans-serif", font_size=10)

plt.title("Solution part B", fontweight='bold')
plt.axis("off")
plt.savefig('solution_part_D.png')

# --- Print results ---
total_distance = 0
total_cost=0

for i in N:
    for j in N:
        for k in K:
            total_distance = total_distance + d[i,j] * x[i,j,k].x

for i in depotsset:
    for j in N:
        for k in K:
            total_cost = total_cost + DepotCosts[i] * x[i,j,k].x
            
print('The total distance travelled is: ' + str(round(total_distance,3)))
print('The total depot cost is: ' + str(round(total_cost,3)))
print('The total cost is: ' + str(round(model.objVal,2)))


vehicle = []
location = [] 
time = []
load = []

for k in K:
    nodes_vehicle = [i for i in N for j in N if x[i,j,k].x == 1]
    nodes_vehicle = nodes_vehicle + [i for i in depotsset for j in N if x[j,i,k].x==1]
    
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