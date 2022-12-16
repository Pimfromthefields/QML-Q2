#Authors: Group 36 of QML

from gurobipy import *
import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
import networkx as nx

model = Model ('Split delivery + heteregeneous fleet VRP Problem - Exercise I & J')



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

#num_vehicle = 25

#c = []
#fc = []

#for i in range(num_vehicle):
#    if i < 10:
#        c.append(20)
#        fc.append(100)
#    else:
#        c.append(100)
#        fc.append(4000)

num_vehicle = 8
c = [100,100,20,20,20,20,20,20]
fc = [4000,4000,100,100,100,100,100,100]

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
K = range(num_vehicle)



# ---- Decision Variables ----
x = {}
T = {}  
for i in N:
     T[i]=model.addVar(lb = 0, vtype=GRB.CONTINUOUS, name="T["+str(i)+"]")
     for j in N:
         for k in K:
             x[i,j,k] = model.addVar(vtype = GRB.BINARY, name = 'x[' + str(i) + ',' + str(j) + ',' + str(k) + ']')

w = {}
for k in K:
    w[k] = model.addVar(vtype = GRB.BINARY, name="w["+str(k)+"]")

y = {}  
for j in N:
    for k in K:
        y[j,k] = model.addVar(lb = 0, ub = 1, vtype = GRB.CONTINUOUS, name = 'y[' + str(j) + ',' + str(k) + ']')


model.update ()


# ---- Objective Function ----
model.setObjective (quicksum(d[i,j] * x[i,j,k] for i in N for j in N for k in K) + quicksum(fc[k]*w[k] for k in K))
model.modelSense = GRB.MINIMIZE
model.update ()

con1 = {}
con2 = {}
con3 = {}
con4 = {}
con5 = {}
for i in N:
    con1[i] = model.addConstr(rT[i] <= T[i]) # arrival time later than ready time
    con2[i] = model.addConstr(T[i] <= dT[i]) # arrival time before due time
    for k in K:    
        con3[i] = model.addConstr(x[i,i,k] == 0)
        for j in range(1,len(N)):
            con4[i,j,k] = model.addConstr(T[j] >= T[i] + sT[i] + d[i,j] - M*(1-x[i,j,k]))
            con5[i,j,k] = model.addConstr(w[k] >= x[i,j,k])

con6 = {}
con7 = {}
con8 = {}
con9 = {}
con10 = {}
for j in range(1,len(N)):
    con6[j] =  model.addConstr(quicksum(y[j,k] for k in K) == 1)
    con7[j] = model.addConstr(quicksum(x[i,j,k] for k in K for i in N)>=1)
    con8[j] = model.addConstr(quicksum(x[j,i,k] for k in K for i in N)>=1)
    for k in K:
        con9[j] = model.addConstr(quicksum(x[i,j,k] for i in N) == quicksum(x[j,i,k] for i in N))
        con10[j,k] = model.addConstr(y[j,k]<=M/Q[j]*quicksum(x[i,j,k] for i in N))

con11 = {}
con12 = {}
con13 = {}
#con14 = {}
for k in K:
    con11[j] = model.addConstr(quicksum(Q[j]*y[j,k] for j in range(1,len(N))) <= c[k])
    con12[k] = model.addConstr(quicksum(x[0,j,k] for j in N) <=1)
    con13[k] = model.addConstr(quicksum(x[j,0,k] for j in N) <=1)
#    con14[k] = model.addConstr(w[k] == 1-x[0,0,k])

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
    print(s)

u = '%8s' % ''
for j in range(len(N)):
    u = u + '%8.1f' % sum (y[j,k].x for k in K)      
print(u)



print ('')
print ('Arrival times: \n')
stored = []
#vehicles_list = ['++','++','++','++','++','++','++']
vehicles_list = []
for i in N:
    vehicles_list.append('++')
print('%8s' % 'Node' + '%8s' % 'Time' + '%8s' % 'Demand' + '%10s' % 'Vehicles')
for i in range(0,1):
    tim = '%8s' % N[i] + '%8.1f' % T[i].x + '%8s' % Q[i] + '%8s' % 'all'
    stored.append(T[i].x)
    vehicles_list[i]='all'
    print(tim)

for i in range(1, len(N)):
    tim = '%8s' % N[i] + '%8.1f' % T[i].x + '%8s' % Q[i] + '      '
    for k in range(len(K)):
        if y[i,k].x == 1:
            tim += '%2s' % k
            vehicles_list[i]=k
        elif 0 < y[i,k].x < 1:
            tim += '%s' % '+'
            vehicles_list[i] = '++'
    stored.append(T[i].x)
    print(tim)

nodes = node
zipped_ns = zip(nodes,stored,vehicles_list)
new = list(zipped_ns)
res = sorted(new, key = operator.itemgetter(1))



print('')
print ('Arrival times sorted by time: \n')

print('%8s' % 'Node' + '%8s' % 'Time' + '%8s' % 'Demand' + '%10s' % 'Vehicles')
for i in range(len(res)):
    tim = '%8s' % res[i][0] + '%8.1f' % res[i][1] + '%8s' % Q[res[i][0]] + '%8s' % res[i][2]
    print(tim)

print(vehicles_list)

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

plt.title("Solution part E", fontweight='bold')