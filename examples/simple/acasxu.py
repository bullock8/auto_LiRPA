"""
A toy example for bounding neural network outputs under input perturbations.
"""
import torch
from collections import defaultdict
from auto_LiRPA import BoundedModule, BoundedTensor
from auto_LiRPA.perturbations import PerturbationLpNorm

net = 0
model = torch.load(f"ACASXU_run2a_{net + 1}_1_batch_2000.pth")
print(type(model))
# Input x.
x = torch.tensor([[1., 1.,1,1,1]])
# Lowe and upper bounds of x.
lower = torch.tensor([[0,0,0,0,0]])
upper = torch.tensor([[2.,2,2,2,2]])

# Wrap model with auto_LiRPA for bound computation.
# The second parameter is for constructing the trace of the computational graph,
# and its content is not important.
lirpa_model = BoundedModule(model, torch.empty_like(x))
pred = lirpa_model(x)
print(pred)
# Compute bounds using LiRPA using the given lower and upper bounds.
norm = float("inf")
ptb = PerturbationLpNorm(norm = norm, x_L=lower, x_U=upper)
bounded_x = BoundedTensor(x, ptb)

# Compute bounds.
lb, ub = lirpa_model.compute_bounds(x=(bounded_x,), method='IBP')
# print(f'IBP bounds: lower={lb.item()}, upper={ub.item()}')
print(f'IBP: lower: {lb}, upper: {ub}')
lb, ub = lirpa_model.compute_bounds(x=(bounded_x,), method='CROWN')
print(f'CROWN : lower: {lb}, upper: {ub}')

# Getting the linear bound coefficients (A matrix).
# required_A = defaultdict(set)
# required_A[lirpa_model.output_name[0]].add(lirpa_model.input_name[0])
# lb, ub, A = lirpa_model.compute_bounds(x=(bounded_x,), method='CROWN', return_A=True, needed_A_dict=required_A)
# print('CROWN linear (symbolic) bounds: lA x + lbias <= f(x) <= uA x + ubias, where')
# print(A[lirpa_model.output_name[0]][lirpa_model.input_name[0]])

# Opimized bounds, which is tighter.
lb, ub = lirpa_model.compute_bounds(x=(bounded_x,), method='alpha-CROWN')
# print(f'alpha-CROWN bounds: lower={lb.item()}, upper={ub.item()}')
print(f'alpha-CROWN : lower: {lb}, upper: {ub}')
# print('alpha-CROWN linear (symbolic) bounds: lA x + lbias <= f(x) <= uA x + ubias, where')
# print(A[lirpa_model.output_name[0]][lirpa_model.input_name[0]])
