#Authors: Group 36 of QML

from gurobipy import *
import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
import networkx as nx

model = Model ('Split delivery + heteregeneous fleet VRP Problem - Exercise I & J')



# ---- Parameters ----
#file = pd.read_csv('data_small.txt', header = None, delim_whitespace=True)  # Load small dataset
file = pd.read_csv('data_large.txt', header = None, delim_whitespace=True)  # Load large dataset

node = file[0].tolist()   #node indices
x_loc = file[1].tolist()  #x coordinates of the nodes
y_loc = file[2].tolist()  #y coordinates of the nodes
Q = file[3].tolist()      #demand (Quantity)
rT = file[4].tolist()     #ready Time
dT = file[5].tolist()     #due Time
sT = file[6].tolist()     #service Time

num_vehicle = 25          #Length of set vehicles

c = []                    #Capacity
fc = []                   #Fixed costs

for i in range(num_vehicle):
    if i < 10:
        c.append(20)
        fc.append(100)
    else:
        c.append(100)
        fc.append(4000)

eps = 0.0001    #Small positive tolerance
M = 5000 + eps  #In the same order of magnitude of the summed demand of the large dataset (1260)

# Creating distance parameter from xloc and yloc
d = np.zeros((len(node), len(node)))
N = range(len(node))
for i in N:
    for j in N:
        d[i, j] = np.sqrt(((x_loc[j] - x_loc[i]) ** 2) + ((y_loc[j] - y_loc[i]) ** 2))



# ---- Sets ----
N = range(len(node))      #Set of nodes i (or j). 
K = range(num_vehicle)    #Set of vehicles k.



# ---- Decision Variables ----
#Binary variable indicating whether the vehicle visits node j after node i
x = {}  
for i in N:
    for j in N:
        for k in K:
            x[i,j,k] = model.addVar(vtype = GRB.BINARY, name = 'x[' + str(i) + ',' + str(j) + ',' + str(k) + ']')

#Arrival time of vehicle k at node i (continuous). 
T = {}
for i in N:
    for k in K:
        T[i,k]=model.addVar(lb = 0, vtype=GRB.CONTINUOUS, name="T[' + str(i) + ',' + str(k) + ']")

#Continuous variable indicating the proportion of j’th node’s demand delivered by vehicle k 
y = {}  
for j in N:
    for k in K:
        y[j,k] = model.addVar(lb = 0, ub = 1, vtype = GRB.CONTINUOUS, name = 'y[' + str(j) + ',' + str(k) + ']')

#Binary variable indicating if vehicle k is used (1) or not (0)
w = {}
for k in K:
    w[k] = model.addVar(vtype = GRB.BINARY, name="w["+str(k)+"]")


model.update ()


# ---- Objective Function ----
model.setObjective (quicksum(d[i,j] * x[i,j,k] for i in N for j in N for k in K) + quicksum(fc[k]*w[k] for k in K))
model.modelSense = GRB.MINIMIZE
model.update ()

# ---- Constraints --------

# Time constraints
con1 = {} #Arrival time should be later than ready time
con2 = {} #Arrival time should not be later than due time
con3 = {} #Ensures that arrival time of vehicle k at j is equal to the sum of the arrival time at i, the sT and rT. 
for i in N:
    for k in K:
        con1[i,k] = model.addConstr(rT[i] <= T[i,k]) 
        con2[i,k] = model.addConstr(T[i,k] <= dT[i])  
        for j in range(1,len(N)):
            con3[i,j,k] = model.addConstr(T[j,k] >= T[i,k] + sT[i] + d[i,j] - M*(1-x[i,j,k]))

con4 = {} #Ensures that a vehicle cannot travel from point i to the same point i
for i in N:
    for k in K:
        con4[i,k] = model.addConstr(x[i,i,k] == 0)
        
con5 = {} #Ensures that the demand is fully satisfied for every node.
con6 = {} #Ensures that every node is visited at least once, but possibly multiple times.
con7 = {} #Ensures that every node is exited at least once, but possibly multiple times.
con8 = {} #Ensures that a vehicle k exits all nodes j which it visits.
con9 = {} #Ensures that every vehicle fulfils demand while adhering to the maximum capacity
for j in range(1,len(N)):
    con5[j] =  model.addConstr(quicksum(y[j,k] for k in K) == 1)
    con6[j] = model.addConstr(quicksum(x[i,j,k] for k in K for i in N)>=1)
    con7[j] = model.addConstr(quicksum(x[j,i,k] for k in K for i in N)>=1)
    for k in K:
        con8[j,k] = model.addConstr(quicksum(x[i,j,k] for i in N) == quicksum(x[j,i,k] for i in N))
        con9[j,k] = model.addConstr(y[j,k]<=M/Q[j]*quicksum(x[i,j,k] for i in N))

con10 = {} #Defining variable w[k]
con11 = {} #New: Ensures that the packages that a vehicle carries does not exceed the capacity.
con12 = {} #New: Ensures that every route starts at node 0.
con13 = {} #New: Ensures that every route ends at node 0.
for k in K:
    con10[k] = model.addConstr(w[k] == quicksum(x[0,j,k] for j in range(1,len(N))))
    con11[j] = model.addConstr(quicksum(Q[j]*y[j,k] for j in range(1,len(N))) <= c[k])
    con12[k] = model.addConstr(quicksum(x[0,j,k] for j in N) <=1)
    con13[k] = model.addConstr(quicksum(x[j,0,k] for j in N) <=1)

# ---- Solve ----

model.setParam( 'OutputFlag', True) # silencing gurobi output or not
model.setParam ('MIPGap', 0);       # find the optimal solution
model.setParam('TimeLimit',1800)
model.write("output.lp")            # print the model in .lp format file

model.optimize ()



# --- Print results ---
print ('\n--------------------------------------------------------------------\n')
    
if model.status == GRB.Status.OPTIMAL: # If optimal solution is found
    print ('Optimal solution: %10.2f cost units' % model.objVal)
    print ('')
    
    print('Distance traveled:')
    print(sum(d[i,j]*x[i,j,k].x for i in N for j in N for k in K))
    
  
else:
    print ('\nNo feasible solution found')



print ('\nREADY\n')



print('')
print ('Route matrix:')
s = '%8s' % ''
for j in range(len(N)):
    s = s + '%8s' % N[j]
print (s)    

for i in range(len(N)):
    s = '%8s' % N[i]
    for j in range(len(N)):
        s = s + '%8.1f' % sum(x[i,j,k].x for k in K)
    s = s + '%8.1f' % sum (x[i,j,k].x for j in N for k in K)   
    print(s)

u = '%8s' % ''
for j in range(len(N)):
    u = u + '%8.1f' % sum (x[i,j,k].x for i in N for k in K)      
print(u)



print('')
print ('Proportions per vehicle k (left axis)')
s = '%8s' % ''
for j in range(len(N)):
    s = s + '%8s' % N[j]
print (s)    

for k in range(len(K)):
    s = '%8s' % k
    for j in range(len(N)):
        s = s + '%8.1f' % y[j,k].x 
    s = s + '%8.1f' % sum (y[j,k].x for j in N)
    print(s)

u = '%8s' % ''
for j in range(len(N)):
    u = u + '%8.1f' % sum (y[j,k].x for k in K)      
print(u)



stored = []
vehicles_list = []
for i in N:
    vehicles_list.append('-1')
for i in range(0,1):
    stored.append(T[i,k].x)
    vehicles_list[i]='all'

for i in range(1, len(N)):   
    for k in range(len(K)):
        if y[i,k].x >= 0.998:
            vehicles_list[i]=k
        elif 0.001 < y[i,k].x < 0.998:
            vehicles_list[i] = '++'
    stored.append(T[i,k].x)

nodes = node

print ('')
print ('Arrival times sorted by node: \n')

zipped_ns2 = zip(nodes,stored,vehicles_list)
new2 = list(zipped_ns2)
res2 = sorted(new2, key = operator.itemgetter(0))

print('%8s' % 'Node' + '%8s' % 'Time' + '%8s' % 'Demand' + '%10s' % 'Vehicles')
for i in range(len(res2)):
    tim = '%8s' % res2[i][0] + '%8.1f' % res2[i][1] + '%8s' % Q[res2[i][0]] + '%8s' % res2[i][2]
    print(tim)


print('')
print ('Arrival times sorted by time: \n')

zipped_ns = zip(nodes,stored,vehicles_list)
new = list(zipped_ns)
res = sorted(new, key = operator.itemgetter(1))

print('%8s' % 'Node' + '%8s' % 'Time' + '%8s' % 'Demand' + '%10s' % 'Vehicles')
for i in range(len(res)):
    tim = '%8s' % res[i][0] + '%8.1f' % res[i][1] + '%8s' % Q[res[i][0]] + '%8s' % res[i][2]
    print(tim)

print('')
print ('Results: \n')
print ('Best solution: %10.2f cost units' % model.objVal)

print ('')   
print('Distance traveled:')
print(sum(d[i,j]*x[i,j,k].x for i in N for j in N for k in K))

print ('')   
print('Total number of fixed costs:')
print(sum(fc[k]*w[k].x for k in K))

print ('')   
print('Number of small vehicles:')
print(sum(w[k].x for k in range(0,10)))

print ('')   
print('Number of large vehicles:')
print(sum(w[k].x for k in range(10,25)))

# --- Visualization ---
G = nx.DiGraph()

arcs = [(i,j) for i in N for j in N]
active_arcs = [(i,j,k) for i in N for j in N for k in K if x[i,j,k].x>0.01]

G.add_edges_from(arcs)

pos = {node[i]: (x_loc[i],y_loc[i]) for i in N}
label_list = node + [""]
labels = {node[i]: label_list[i] for i in N}

color_map = []
colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf', '#7FFF00', '#FFD700', '#F08080', '#FFA07A', '#03A89E', '#FFE4B5', '#FFFF00', '#FF6347', '#00FF7F', '#87CEFF', '#A52A2A']


for j in vehicles_list:
    if j == 'all':
        color_map.append('grey')
    elif j == '++':
        color_map.append('#8c564b')
    elif j < 10: #for large dataset set j < 10. Then cargo bikes are green and the vans are blue.
        color_map.append('g')
    else:
        color_map.append('b')
        
plt.figure(3,figsize=(15,15)) 
nx.draw_networkx_nodes(G, pos, node_color=color_map, node_size=700)

nx.draw_networkx_edges(G, pos, edgelist=active_arcs, edge_color='k')

nx.draw_networkx_labels(G, pos, labels=labels, font_weight='bold', font_family="sans-serif", font_size=15)

plt.title("Solution part J", fontweight='bold')