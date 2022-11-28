#Authors: Group 36 of QML

from gurobipy import *
import pandas as pd
import numpy as np
from matplotlib import pyplot as plt

#model = Model ('TSP Problem')


# ---- Parameters ----


file = pd.read_csv('data_small.csv', delimiter = ';', header = None)  # Load small dataset
#file = pd.read_csv('data_large.csv', delimiter = ';', header = None) # Load small dataset

node = file[0].tolist()
x_loc = file[1].tolist()
y_loc = file[2].tolist()
D = file[3].tolist()
rT = file[4].tolist()
dT = file[5].tolist()
sT = file[6].tolist()



# ---- Sets ----

#I = 
#J = 

# ---- Decision Variables ----

'''x = {} 
for i in I:
    for j in J:
#        x[i,j] = model.addVar (lb = 0, vtype = GRB.CONTINUOUS, obj = cargoprofit[i], name = 'X[' + str(i) + ',' + str(j) + ']')
        x[i,j] = model.addVar (lb = 0, vtype = GRB.CONTINUOUS, name = 'X[' + str(i) + ',' + str(j) + ']')

model.update ()'''


# ---- Objective Function ----

'''model.setObjective ( )
model.modelSense = GRB.MINIMIZE
model.update ()'''


# ---- Constraints ----

# ---- Solve ----

#model.setParam( 'OutputFlag', True) # silencing gurobi output or not
#model.setParam ('MIPGap', 0);       # find the optimal solution
#model.write("output.lp")            # print the model in .lp format file

#model.optimize ()



# --- Print results ---
print ('\n--------------------------------------------------------------------\n')
    
#if model.status == GRB.Status.OPTIMAL: # If optimal solution is found
#    print ('Total costs: %10.2f euro' % model.objVal)
#    print ('')
    
#else:
#    print ('\nNo feasible solution found')

#print ('\nREADY\n')