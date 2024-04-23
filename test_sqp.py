# Let us consider the problem of minimizing the Rosenbrock function. 
# This function (and its respective derivatives) is implemented 
# in rosen (resp. rosen_der, rosen_hess) in the scipy.optimize.
from scipy.optimize import minimize, rosen, rosen_der

# A simple application of the Nelder-Mead method is:
x0 = [1.3, 0.7, 0.8, 1.9, 1.2]
res = minimize(rosen, x0, method='Nelder-Mead', tol=1e-6)
res.x

# Now using the BFGS algorithm, using the first derivative and a few options:
res = minimize(rosen, x0, method='BFGS', jac=rosen_der,
               options={'gtol': 1e-6, 'disp': True})
res.x  # The x value that minimizes the Rosenbrock function
print(res.message)

# Next, consider a minimization problem with several constraints 
# (namely Example 16.4 from [5]). The objective function is:
# Objective function
fun = lambda x: (x[0] - 1)**2 + (x[1] - 2.5)**2 
# Constraints
cons = ({'type': 'ineq', 'fun': lambda x:  x[0] - 2 * x[1] + 2}, 
        {'type': 'ineq', 'fun': lambda x: -x[0] - 2 * x[1] + 6},
        {'type': 'ineq', 'fun': lambda x: -x[0] + 2 * x[1] + 2})

# Bounds for variables
bnds = ((0, None), (0, None))

# Call the solver
res = minimize(fun, (2, 0), method='SLSQP', bounds=bnds,
               constraints=cons)

print(res.x)  # The x value that minimizes the objective function