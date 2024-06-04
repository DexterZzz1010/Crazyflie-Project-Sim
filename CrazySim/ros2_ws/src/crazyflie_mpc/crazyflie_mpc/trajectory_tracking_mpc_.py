from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import numpy as np
import casadi as ca
from casadi import SX, vertcat, horzcat, diag, inv_minor, cross, sqrt,  cos, sin, norm_2, tanh, GenMX_zeros
from scipy.linalg import block_diag
from .quadrotor_simplified_model import QuadrotorSimplified
from threading import Thread
from time import sleep, time
from pathlib import Path
import importlib
import sys

class TrajectoryTrackingMpc:
    def __init__(self, name: str, quadrotor: QuadrotorSimplified, horizon: float, num_steps: int, code_export_directory : Path=Path('acados_generated_files')):
        self.model_name = name
        self.quad = quadrotor
        self.horizon = horizon
        self.num_steps = num_steps
        self.ocp_solver = None
        self.solver_locked = False
        self.hover_control = np.array([0., 0., 0., self.quad.gravity*self.quad.mass])
        # self.acados_generated_files_path = Path(__file__).parent.resolve() / 'acados_generated_files'
        self.acados_generated_files_path = code_export_directory
        try:
            if self.acados_generated_files_path.is_dir():
                sys.path.append(str(self.acados_generated_files_path))
            acados_ocp_solver_pyx = importlib.import_module('c_generated_code.acados_ocp_solver_pyx')
            self.ocp_solver = acados_ocp_solver_pyx.AcadosOcpSolverCython(self.model_name, 'SQP', self.num_steps)
            print('Acados cython module imported successfully.')
        except ImportError:
            print('Acados cython code not generated. Generating cython code now...')
            self.generate_mpc()
    
    def __copy__(self):
        return type(self)(self.model_name, self.quad, self.horizon, self.num_steps, self.acados_generated_files_path)


    '''
    非线性约束函数，确保无人机不进入定义的禁飞区 
    '''
    def no_fly_zone_constraint(x, p):
        # 无人机的位置状态
        pos_x, pos_y, pos_z = x[0], x[1], x[2]
        
        # 计算到两个球形禁飞区的距离
        dist1 = ca.sqrt((pos_x - p[0])**2 + (pos_y - p[1])**2 + (pos_z - p[2])**2)
        dist2 = ca.sqrt((pos_x - p[3])**2 + (pos_y - p[4])**2 + (pos_z - p[5])**2)
        
        # 距离应大于等于球的半径
        return ca.vertcat(dist1 - p[6], dist2 - p[7])

    '''
    负责初始化和设置mpc问题的模型和求解器配置
    '''
    def generate_mpc(self):
        f_expl, x, u = self.quad.dynamics()
        # Define the Acados model 
        # 定义模型
        model = AcadosModel()
        model.f_expl_expr = f_expl # 动力学模型
        model.x = x # 输入 可以在此加入障碍物距离
        model.u = u # 输出
        model.name = self.model_name
        model.p = ca.SX.sym('p', 8)  # 球形区域的参数: 两个中心位置(各3维) + 两个半径

        # Define the optimal control problem
        ocp = AcadosOcp() # 控制优化问题
        ocp.model = model

        ocp.code_export_directory = self.acados_generated_files_path / ('c_generated_code')
        nx = model.x.size()[0] # number of states
        nu = model.u.size()[0] # number of controls
        ny = nx + nu  # size of intermediate cost reference vector in least squares objective
        ny_e = nx # size of terminal reference vector

        N = self.num_steps
        Tf = self.horizon
        ocp.dims.N = N    # 0.1 small step for example
        ocp.solver_options.tf = Tf   # ho 30/20   -   8sec to more

        # 权重矩阵
        Q = np.diag([20., 20., 20., 2., 2., 2., 1., 1., 1.])   # 状态变量
        R = diag(horzcat(1., 1., 1., 1.))  # 控制输入变量
        W = block_diag(Q,R) #组合对角矩阵

        # 代价函数
        ocp.cost.cost_type = 'LINEAR_LS' # 最小二乘
        # 转换矩阵， 将状态和控制输入映射到输出向量
        ocp.cost.Vx = np.vstack([np.identity(nx), np.zeros((nu,nx))])
        ocp.cost.Vu = np.vstack([np.zeros((nx,nu)), np.identity(nu)])
        ocp.cost.W = W # 权重矩阵
        ocp.cost.yref = np.zeros(ny) # 目标参考值

        # 终端代价
        ocp.cost.cost_type_e = 'LINEAR_LS'
        ocp.cost.W_e = Q
        ocp.cost.Vx_e = np.vstack([np.identity(nx)])
        ocp.cost.yref_e = np.zeros(ny_e)
  
        # bounds on control
        '''
        - `lbu` 和 `ubu`: 分别设置控制输入的下界和上界。例如，角度和推力的限制。
        - `idxbu`: 指定哪些控制输入受到界限约束。
        - `lbx` 和 `ubx`: 分别设置状态变量的下界和上界。例如，位置和角度的限制。
        - `idxbx`: 指定哪些状态变量受到界限约束。
        '''
        

        max_angle = np.radians(15) # [rad]
        max_thrust = 0.477627618 # [N]
        ocp.constraints.lbu = np.array([-max_angle, -max_angle, -np.radians(10), 0.])
        ocp.constraints.ubu = np.array([max_angle, max_angle, np.radians(10), max_thrust])
        ocp.constraints.idxbu = np.array([0,1,2,3])

        max_height = 4.0
        ocp.constraints.lbx = np.array([-10.,-10.,0.,-2.,-2.,-2.,-max_angle, -max_angle, -np.radians(180)])
        ocp.constraints.ubx = np.array([10.,10.,max_height,2.,2.,2.,max_angle, max_angle, np.radians(180)])
        ocp.constraints.idxbx = np.array([0,1,2,3,4,5,6,7,8])

        # initial state
        ocp.constraints.x0 = np.zeros(9)

        json_file = str(self.acados_generated_files_path / ('acados_ocp.json'))
        # solver options
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.nlp_solver_type = 'SQP'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.tol = 1e-3 # tolorance - 17 the higher number the tighter the solution neeeds to be, lower if mpc crashes
        ocp.solver_options.qp_tol = 1e-3
        ocp.solver_options.nlp_solver_max_iter = 20
        ocp.solver_options.qp_solver_iter_max = 30  # incurease the iterations if you crash
        # 4 gives a small table explanation good for report 
        ocp.solver_options.print_level = 0
        
        AcadosOcpSolver.generate(ocp, json_file=json_file)
        AcadosOcpSolver.build(ocp.code_export_directory, with_cython=True)

        if self.acados_generated_files_path.is_dir():
            sys.path.append(str(self.acados_generated_files_path))
        acados_ocp_solver_pyx = importlib.import_module('c_generated_code.acados_ocp_solver_pyx')
        self.ocp_solver = acados_ocp_solver_pyx.AcadosOcpSolverCython(self.model_name, 'SQP', self.num_steps)


    def solve_mpc(self, x0, yref, yref_e, solution_callback=None):
        if self.solver_locked:
            # print('mpc solver locked, skipping...')
            return
        self.solver_locked = True

        N = self.num_steps
        nx = len(x0)
        nu = 4
        
        if yref.shape[1] != self.num_steps:
            raise Exception('incorrect size of yref')
    
        for i in range(N):
            self.ocp_solver.set(i, 'yref', np.array([*yref[:,i], *self.hover_control]))
            # self.ocp_solver.set(i, 'x', yref[:,i])
            # self.ocp_solver.set(i, 'u', self.hover_control)
        
        self.ocp_solver.set(N, 'yref', yref_e)

        x_mpc = np.zeros((N+1, nx))
        u_mpc = np.zeros((N, nu))
        self.ocp_solver.set(0, 'lbx', x0)
        self.ocp_solver.set(0, 'ubx', x0)
        
        status = self.ocp_solver.solve()

        # extract state and control solution from solver
        for i in range(N):
            x_mpc[i,:] = self.ocp_solver.get(i, "x")
            u_mpc[i,:] = self.ocp_solver.get(i, "u")
        x_mpc[N,:] = self.ocp_solver.get(N, "x")

        self.solver_locked = False

        if solution_callback is not None:
            solution_callback(status, x_mpc, u_mpc)
        else:    
            return status, x_mpc, u_mpc