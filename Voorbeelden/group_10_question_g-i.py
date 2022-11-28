from gurobipy import *
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.backends.backend_pdf import PdfPages
from datetime import datetime

data_file = "data_i"

# --- Model ---        

model = Model("Assignment Q2 2021 - Group 10")


# --- Parameters ---

xcoord = np.loadtxt("%s.txt" %data_file)[:, 1]          # x coordinates of the location
ycoord = np.loadtxt("%s.txt" %data_file)[:, 2]          # y coordinates of the location
demand = np.loadtxt("%s.txt" %data_file)[:, 3]          # parcel volume for the task 
readytime = np.loadtxt("%s.txt" %data_file)[:, 4]       # earliest time for a task
duetime = np.loadtxt("%s.txt" %data_file)[:, 5]         # latest time for a task
servicetime = np.loadtxt("%s.txt" %data_file)[:, 6]     # time needed for a task at the location
pickup_id = np.loadtxt("%s.txt" %data_file)[:, 7]       # origin location for a delivery
deliver_id = np.loadtxt("%s.txt" %data_file)[:, 8]      # destination location for a pickup
capacity = [200]*20                                     # capacity of vehicle k
depot_costs = [150,100,100,125]                         # costs to make use of a depot
M = 10**8

d = []
for i in range(len(xcoord)):
    distance_ij = []
    for j in range(len(ycoord)):
        distance_ij.append(np.sqrt((xcoord[i]-xcoord[j])**2 + (ycoord[i]-ycoord[j])**2))
    d.append(list(distance_ij))    
d = np.array(d)


# --- Sets ---

N = range(len(np.loadtxt("%s.txt" %data_file)[:, 0]))   # set of all locations
K = range(len(capacity))                                # set of vehicles

P = []                                                  # set of pick-up
D = []                                                  # set of delivery
PD = []                                                 # set of pick-up and deliveries
S = []                                                  # set of depots (stock)

for i in N:
    if demand[i] > 0:
        P.append(i)
        D.append(int(deliver_id[i]))
        PD.append(i)
        PD.append(int(deliver_id[i]))
    elif demand[i] == 0:
        S.append(i)


# --- Variables ---

# binary variable indicating if vehicle k travels between location i and j
x = {}
for i in N:
    for j in N:
        for k in K:
            x[i,j,k] = model.addVar (lb = 0, vtype = GRB.BINARY)

# Arrival time of truck k at location i
TA = {}
for k in K:
    for i in N:
        TA[k,i] = model.addVar (lb = readytime[i], ub = duetime[i], vtype = GRB.CONTINUOUS)
        
# Costs of usage of hub
c = {}
for k in K:
    c[k] = model.addVar (lb = 0, vtype = GRB.CONTINUOUS)
    
# Load of vehicle
q = {}
for k in K:
    for i in N:
        q[k,i] = model.addVar(lb = 0, ub=capacity[k], vtype = GRB.CONTINUOUS)

# Integrate new variables
model.update ()


# ---- Objective Function ---

model.setParam('NumObj', 2)
model.setObjectiveN( quicksum(x[i,j,k]*d[i][j] for i in N for j in N for k in K),0,1 )
model.setObjectiveN( quicksum(c[k] for k in K),1,1 )
model.modelSense = GRB.MINIMIZE
model.update ()


# --- Constraints ---

# Constraint : Visit pick-up and delivery locations
con2 = {}
for j in P:
    con2[j] = model.addConstr( quicksum(x[i,j,k] for i in N for k in K) == 1 )
for j in D:
    con2[j] = model.addConstr( quicksum(x[i,j,k] for i in N for k in K) == 1 )

# Constraint : Arrival time of next location
con3 = {}
for k in K:
    for i in N:
        for j in PD:
            con3[k,i,j] = model.addConstr( TA[k,j] >= TA[k,i] + d[i][j] + servicetime[i] - M*(1-x[i,j,k]) )

# Constraint : Capacity of vehicle at the next location
con4a = {}
for k in K:
    for i in N:
        for j in PD:
            con4a[k,i,j] = model.addConstr( q[k,j] >= q[k,i] + demand[j] - M*(1-x[i,j,k]) )

# Constraint : Capacity of vehicle at the next location
con4b = {}
for k in K:
    for i in N:
        for j in PD:
            con4b[k,i,j] = model.addConstr( q[k,j] <= q[k,i] + demand[j] + M*(1-x[i,j,k]) )

# Constraint : Vehicle can not travel to its current position
con5 = {}
for i in N:
    for k in K:
        con5[i,k] = model.addConstr( x[i,i,k] == 0 )

# Constraint : Flow conservation
con6 = {}
for i in N:
    for k in K:
        con6[i,k] = model.addConstr( quicksum(x[i,j,k] for j in N ) == quicksum(x[j,i,k] for j in N ) )
  
# Constraint : Every vehicle starts at a depot
con7 = {}
for k in K:
    con7[k] = model.addConstr( quicksum(x[i,j,k] for i in S for j in N) == 1 )
    
# Constraint : Cost per vehicle leaving from each depot
con8 = {}
for k in K:
    con8[k] = model.addConstr( quicksum(x[i,j,k]*depot_costs[i] for i in S for j in N) == c[k] )

# Constraint : Vehicle must visit specific pick-up location before corresponding delivery location
con9a = {}
for k in K:
    for j in D:
        con9a[k,j] = model.addConstr(quicksum( x[pickup_id[j],z,k] for z in N) >= quicksum( x[i,j,k] for i in N) )
        
con9b = {}
for k in K:
    for i in P:
        con9b[k,i] = model.addConstr(quicksum( x[z,deliver_id[i],k] for z in N) >= quicksum( x[i,j,k] for j in N) ) 


# --- Solve --

model.setParam( 'OutputFlag', True) # silencing gurobi output or not
model.setParam ('MIPGap', 0);       # find the optimal solution
# model.Params.TimeLimit = 3600       # limit the computational timne to 3600 seconds
model.write("output.lp")            # print the model in .lp format file
model.optimize ()


# --- Print results ---
print ('\n--------------------------------------------------------------------\n')

if model.status == GRB.Status.OPTIMAL: # If optimal solution is found
    
    print ('Total distance :', round(sum(x[i,j,k].x*d[i][j] for i in N for j in N for k in K),2),'km')
    print ('Total costs :', round(sum(c[k].x for k in K),2),'euros')
    
    complete_plot_xcoord = []
    complete_plot_ycoord = []
    
    for k in K:
        
        print("\n")
        print("Vehicle :", k+1)
        
        values = []
        
        for i in N:
            for j in N:
                if(x[i,j,k].x == 1):
                    values.append([i,j,round(d[i][j],1),round((TA[k,i].x + servicetime[i]),1),round(TA[k,j].x,1),demand[j],round(q[k,j].x),xcoord[i],xcoord[j],ycoord[i],ycoord[j]])
        if len(values) < 2:
            print("Not used")
        else:        
            data = pd.DataFrame(np.array(values), columns=['i', 'j', 'dist', 'TD','TA','demand','load','xcoord_i','xcoord_j','ycoord_i','ycoord_j'])
            data = data.sort_values(by=['TD'])
            
            total_distance_value = 0
            total_distance = []
            
            total_load_value = 0
            total_load = []
            
            for i in range(len(data.loc[:,'i'])):
                total_distance_value = total_distance_value + data.iloc[i]['dist']
                total_distance.append(total_distance_value)
                total_load_value = total_load_value + data.iloc[i]['demand']
                total_load.append(total_load_value)
                
            
            # Complete route
            route_xcoord = list(data.loc[:,'xcoord_i'])
            route_xcoord.append(data.iloc[-1]['xcoord_j'])
            complete_plot_xcoord.append(route_xcoord)
            
            route_ycoord = list(data.loc[:,'ycoord_i'])
            route_ycoord.append(data.iloc[-1]['ycoord_j'])
            complete_plot_ycoord.append(route_ycoord)
            
            # Depot of route
            depot_xcoord = data.loc[0,'xcoord_i']
            depot_ycoord = data.loc[0,'ycoord_i'] 
            
            #Replace arrival time of last node
            data['TA'] = data['TA'].replace([0],(data.iloc[-1]['TD']+data.iloc[-1]['dist']))
            
            # Locations of route
            locations_xcoord = route_xcoord[1:-1]
            locations_ycoord = route_ycoord[1:-1]
            annotations_location = list(data.iloc[1:]['i'])
            
            data['tot_dist'] = total_distance
            data['tot_load'] = total_load
            
            color = (np.random.random(), np.random.random(), np.random.random())
            
            now = datetime.now()
            dt_string = now.strftime("%d_%m_%Y-%H_%M_%S")
            
            data = data[['i', 'j', 'tot_dist', 'TD', 'TA','demand','load','tot_load']]
                
            print(data)
            
            depot = plt.scatter(depot_xcoord, depot_ycoord, marker='o', color="#808080")
            location = plt.scatter(locations_xcoord, locations_ycoord, marker='x', color="#808080")
            
            plt.annotate(data.loc[0,'i'], (depot_xcoord, depot_ycoord), color="#000000")
            
            for i, label in enumerate(annotations_location):
                plt.annotate(label, (locations_xcoord[i], locations_ycoord[i]), color="#000000")
            
            plt.xlabel('x coordinates (km)')
            plt.ylabel('y coordinates (km)')
            
            plt.legend((depot, location),
                    ('Depot', 'Location'),
                    scatterpoints=1,
                    ncol=3,
                    fontsize=8)
            
            plt.title('Route Vehicle %s'%round(k+1,0))
            plt.plot(route_xcoord,route_ycoord,linestyle='solid',color=color)
            plt.show()
            plt.clf()

    # Complete plot         
    xcoord_depot = []
    ycoord_depot = []
    annotations_depot = []
    xcoord_location = []
    ycoord_location = []
    annotations_location = []
    
    for i in range(len(xcoord)):
        if pickup_id[i] == deliver_id[i] == 0:
            xcoord_depot.append(xcoord[i])
            ycoord_depot.append(ycoord[i])
            annotations_depot.append(i)
        else:
            xcoord_location.append(xcoord[i])
            ycoord_location.append(ycoord[i])
            annotations_location.append(i)
    
    depot = plt.scatter(xcoord_depot, ycoord_depot, marker='o', color="#808080")
    location = plt.scatter(xcoord_location, ycoord_location, marker='x', color="#808080")
    
    for i, label in enumerate(annotations_depot):
        plt.annotate(label, (xcoord_depot[i], ycoord_depot[i]), color="#000000")
    
    plt.xlabel('x coordinates (km)')
    plt.ylabel('y coordinates (km)')
    
    plt.legend((depot, location),
           ('Depot', 'Location'),
           scatterpoints=1,
           loc='lower left',
           ncol=3,
           fontsize=8)
    
    plt.title('Route of all vehicles')
    
    
    
    for i in range(len(complete_plot_xcoord)):
        
        color = (np.random.random(), np.random.random(), np.random.random())
        
        plt.plot(complete_plot_xcoord[i],complete_plot_ycoord[i],linestyle='solid',color=color)

    plt.show()

else:
    print ('\nNo feasible solution found')
    
print ('\nREADY\n')

