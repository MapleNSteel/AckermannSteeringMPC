import muaompc
import numpy
import system1

system1.N=10

mpcy = muaompc.ltidt.setup_mpc_problem("system1.py", verbose=True)

print(mpcy)

mpcy.ctl.conf.in_iter = 5  # configuration done only once
mpcy.ctl.conf.warmstart = True  # use warmstart

mpcy.ctl.x_ref = numpy.ones(mpcy.ctl.x_ref.shape) * 2

# repeat the following lines for every new state x 
x = numpy.ones(mpcy.ctl.x_ref.shape) * 3
mpcy.ctl.solve_problem(x)
u0 = mpcy.ctl.u_opt[:mpcy.size.inputs]  # 1st input vector in sequence
