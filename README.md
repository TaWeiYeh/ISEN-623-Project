# ISEN-623-Project
In trajectory optimization, I want to compare the computation speed between Differential Dynamic Programming and Sequential Quadratic Programming. 

In trajectory optimization, there are two prevalent optimizers which is Differential Dynamic Programming (DDP) and Sequential Quadratic Programming.  I want to compare the computation speed between the two optimizer. DDP is implemented by Zhaoming Xie in their paper, Differential Dynamic Programming with Nonlinear Constraints (https://github.com/ZhaomingXie/CDDP/tree/master). SQP would be implemented using SciPy package. 

# Requirement
All package version is accepted. 
```
numpy
scipy
osqp
```

# Execution
## SQP
```
python3 sqp_scipy.py
```

## CDDP
```
python3 cddp.py
```

## MATLAB
MATLAB is implemented using CasAdi, a popular and free optimal control optimizer. The background solver is IPOPT (Interior Point OPTimizer). This MATLAB version is created to integrate SNOPT, a professional SQP solver, in CasAdi. However, the integration is not straightforward. Thus, the MATLAB result is not included in the trajectory optimization efficiency comparison. 