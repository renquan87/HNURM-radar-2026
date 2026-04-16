"""
bbox_kalman.py — 像素级边界框卡尔曼滤波模块
==========================================================
本模块提供无状态的边界框卡尔曼滤波计算接口，用于在二维图像坐标系内
对检测框进行时序平滑、短时预测与观测校正。模块仅处理像素域运动学，不涉及
物理世界坐标映射与业务发布策略。

状态向量: [cx, cy, w, h, vx, vy, vw, vh]
    - 位置与尺度: 边界框中心点与宽高
    - 速度项: 对应四个量的一阶变化率

观测向量: [cx, cy, w, h]
    - 来自检测器的当前帧边界框中心点与宽高观测

核心逻辑与主要功能：
    - 构建并维护标准线性卡尔曼系统矩阵（F/H/I），支持按真实 `dt` 动态更新状态转移。
    - 基于目标尺度自适应计算过程噪声与观测噪声，改善远距离小目标稳定性。
    - 提供 `initiate/predict/update` 三阶段接口，供上层关联器执行完整滤波循环。
    - 在观测突变场景下执行跳变重置与速度钳制，抑制预测发散与异常“飞框”。
    - 通过位置/速度/尺度差分权重配置，在平滑性与跟随性之间保持可调平衡。
"""
import numpy as np

class BBoxKalmanFilter(object):
    """
    标准的 8 状态边界框卡尔曼滤波器工具类。
    与EKF变量命名统一。
    """
    def __init__(self, dt: float = 1.0/30.0):
        """
        初始化系统矩阵。
        参数 dt: 系统的帧间时间间隔。默认假设相机推理帧率为 30Hz。
        """
        self.n = 8  # 状态向量个数n (cx, cy, w, h, vx, vy, vw, vh)
        self.m = 4  # 测量观测值个数m (cx, cy, w, h)
        
        # 预测状态变换矩阵,依据恒定速度运动学方程得出的变换矩阵,维数n x n
        # F_k(n,n) * X^_k-1(n,1) --> x^_k(n,1) 新时刻状态向量
        self.F_k = np.eye(self.n)
        for i in range(self.m):
            self.F_k[i, self.m + i] = 1.0

        # 预测阻尼：平移速度弱阻尼、尺度速度强阻尼，用于抑制异常外推。
        self.vel_damping_xy = 0.94
        self.vel_damping_wh = 0.92

        self.F_k[4, 4] = self.vel_damping_xy
        self.F_k[5, 5] = self.vel_damping_xy
        self.F_k[6, 6] = self.vel_damping_wh
        self.F_k[7, 7] = self.vel_damping_wh  
                
                
        # [debug]切断 vw (宽度变化率) 和 vh (高度变化率) 在预测步中对实际宽高的影响。
        # 这样在纯预测 (LOST/GUESSING) 期间，中心点(cx, cy)会按速度(vx, vy)正常滑行，
        # 但框的大小(w, h)将被锁住，保持消失前最后一帧的真实大小，面积绝不会变为0
        # ==========================================
        self.F_k[2, 6] = 0.0  # 锁死宽度的缩放预测
        self.F_k[3, 7] = 0.0  # 锁死高度的缩放预测

        # 传感器测量值向量与预测值向量之间的线性转换矩阵 (观测矩阵)
        # m x n矩阵, H_k(mxn) * X(nx1) = ZZ_k(mx1)
        self.H_k = np.eye(self.m, self.n)

        # 单位矩阵I, 这里当数字1使用. P_k = (I - K_k*H_k)*P_k
        self.I = np.eye(self.n)


        # 1. 过程噪声基准权重 (Q): 反映系统动态模型的信任程度，数值远小于R以确保对预测的及时响应。
        self._q_weight_pos = 1.0 / 10000    
        self._q_weight_vel = 1.0 / 15       # 速度过程噪声，大幅增大以加速速度状态收敛
        # # 上一版参数
        # self._q_weight_vel = 1.0 / 40
        # # 原始参数
        # self._q_weight_vel = 1.0 / 80
        self._q_weight_scale = 1.0 / 20000


        # 2. 观测噪声基准权重 (R): 保持对观测值的适度信任，构建低通屏障
        self._r_weight_pos = 1.0 / 800     # 大幅降低 R，使 K 从 0.15 提升至 0.5 附近
        # # 上一版参数
        # self._r_weight_pos = 1.0 / 200
        self._r_weight_scale = 1.0 / 200    # 尺度的观测噪声权重
        # # 原始参数
        # self._r_weight_pos = 1.0 / 100      # 中心点位置的观测噪声权重 (量级远大于Q)
        # self._r_weight_scale = 1.0 / 200    # 尺度的观测噪声权重

        # 3. AKF 自适应参数
        self._akf_innov_thr = 0.08
        self._akf_alpha_max = 8.0
        self._akf_r_gain = 0.7

        # 4. AKF predict 侧：速度自适应过程噪声
        self._akf_vel_boost_thr = 18.0
        self._akf_vel_boost_max = 5.0

        # 5. 速度直注入
        self._vel_inject_gain = 0.22
        self._vel_inject_deadzone = 0.03
        self._vel_inject_max = 140.0
        self._vel_inject_norm_max = 0.7   # 新增：残差过大时禁止注入，防误关联飞框

        # 6. 预测速度钳位
        self._predict_speed_max = 220.0   # 新增：纯预测阶段速度上限(px/s)

        # 调试开关：输出 predict/update 关键指标
        self._debug = True
        
        




    def _get_adaptive_sf(self, w, h):
        """
        计算尺度自适应因子，用于降低远距离小目标框抖动的影响。
        """
        area = w * h
        # 设定基准面积为 3600 px^2 (60x60)
        if area < 3600:
            return 1.0 + (3600 - area) / 1200.0
        return 1.0

    def initiate(self, z: np.ndarray):
        """
        从单个 YOLO 观测值初始化一个新的轨迹状态。
        参数:
            z: 传感器读数 (measurement), [cx, cy, w, h]
        返回:
            x: 初始化的 n维状态向量
            P_result: 初始化的 nxn最优估计协方差矩阵
        """
        # x: 上一时刻(k-1)或当前时刻k的状态向量: n个元素向量
        x = np.r_[z, np.zeros_like(z)]

        # 引入尺度因子 sf，针对远距离小目标适度放大初始不确定性。
        sf = self._get_adaptive_sf(z[2], z[3])

        # 初始协方差矩阵设定：给速度项分配极大的不确定性
        std = [
            2 * self._r_weight_pos * z[2] * sf,     # cx 初始噪声
            2 * self._r_weight_pos * z[3] * sf,     # cy 初始噪声
            2 * self._r_weight_scale * z[2] * sf,   # w 初始噪声
            2 * self._r_weight_scale * z[3] * sf,   # h 初始噪声
            10 * self._q_weight_vel * z[2] * sf,    # vx 初始分布
            10 * self._q_weight_vel * z[3] * sf,    # vy 初始分布
            10 * self._q_weight_vel * z[2] * sf,    # vw 初始分布
            10 * self._q_weight_vel * z[3] * sf     # vh 初始分布
        ]
        # P_result: 最优P_k, 当前时刻最优估计协方差矩阵 (对角阵)
        P_result = np.diag(np.square(std))
        return x, P_result

    def predict(self, x: np.ndarray, P_result: np.ndarray, dt=0.033):
        """
        根据系统运动学方程，更新预测状态。
        参数:
            x: 前一时刻(k-1)状态向量
            P_result: 前一时刻最优估计协方差矩阵
            dt: 真实时间步长
        返回:
            x_pred: 新时刻(k)的先验预测状态向量
            P_current: 新时刻的先验预测协方差矩阵
        """
        # 动态更新状态转移矩阵中的 dt 系数，使位移与真实时间步长一致。
        self.F_k[0, 4] = dt
        self.F_k[1, 5] = dt

        # 根据当前预测框宽高计算尺度因子。
        sf = self._get_adaptive_sf(x[2], x[3])

        # Q_k: 各状态变量的预测噪声协方差矩阵 (动态计算)
        std_pos = [
            self._q_weight_pos * x[2] * sf,
            self._q_weight_pos * x[3] * sf,
            self._q_weight_scale * x[2] * sf,
            self._q_weight_scale * x[3] * sf
        ]
        std_vel = [
           self._q_weight_vel * x[2] * sf,
            self._q_weight_vel * x[3] * sf,
            self._q_weight_vel * x[2] * sf,
            self._q_weight_vel * x[3] * sf
        ]
        Q_k = np.diag(np.square(np.r_[std_pos, std_vel]))

        # ── AKF predict: 速度自适应过程噪声 ──
        # 高速运动时放大 Q_vel，使 P_vel 更大，下一帧 update 时 K_vel 增大，速度学习更快
        speed = float(np.sqrt(x[4]**2 + x[5]**2))
        vel_boost = max(1.0, speed / self._akf_vel_boost_thr)
        vel_boost = min(vel_boost, self._akf_vel_boost_max)
        if vel_boost > 1.0:
            Q_k[4, 4] *= vel_boost      # vx 过程噪声放大
            Q_k[5, 5] *= vel_boost      # vy 过程噪声放大

        # 预测状态方程
        # X_k = F_k * X_k-1
        x_pred = np.dot(self.F_k, x)

        # 预测协方差矩阵
        # P_k = F_k * P_k-1 * F_k^T + Q_k
        P_current = np.dot(self.F_k, np.dot(P_result, self.F_k.T)) + Q_k

        # ── [debug] predict 诊断输出 ──
        if self._debug:
            speed_before = float(np.sqrt(x[4]**2 + x[5]**2))
            speed_after = float(np.sqrt(x_pred[4]**2 + x_pred[5]**2))
            if speed_before > 5.0:
                print(f"[BBoxKF predict] "
                      f"pos=({x_pred[0]:.1f},{x_pred[1]:.1f}) "
                      f"vel=({x_pred[4]:.1f},{x_pred[5]:.1f}) "
                      f"speed={speed_before:.1f}->{speed_after:.1f} "
                      f"damping={self.vel_damping_xy} "
                      f"vel_boost={vel_boost:.2f}")

        return x_pred, P_current

    def update(self, x: np.ndarray, P_current: np.ndarray, z: np.ndarray, dt: float = 0.033):
        """
        观测更新步。结合 YOLO 实际测量值纠正预测状态。
        引入 AKF 自适应机制与速度直注入，解决隐状态速度学习慢的结构性问题。
        参数:
            x: 预测状态向量 (X_k)
            P_current: 预测协方差矩阵 (P_k)
            z: 当前时刻 YOLO 传感器读数 [cx, cy, w, h]
            dt: 真实时间步长 (秒)，用于速度直注入
        返回:
            x_new: 更新后的状态向量
            P_result: 更新后的协方差矩阵
        """
        

        # 依据当前观测值计算尺度因子
        sf = self._get_adaptive_sf(z[2], z[3])

        # R_k: 基线传感器测量噪声协方差矩阵
        std = [
            self._r_weight_pos * z[2] * sf,
            self._r_weight_pos * z[3] * sf,
            self._r_weight_scale * z[2] * sf,
            self._r_weight_scale * z[3] * sf
        ]
        R_k = np.diag(np.square(std))

        # ── AKF: 基于归一化像素残差的自适应噪声调节 ──
        # Step 1: 计算残差
        innovation = z - np.dot(self.H_k, x)

         # 调试用：预测中心与观测中心的欧氏距离
        dist = float(np.linalg.norm(x[:2] - z[:2]))


        # Step 2: 归一化像素残差（除以框对角线，消除尺度影响）
        innov_pos = innovation[:2]
        diag = np.sqrt(z[2]**2 + z[3]**2) + 1e-5
        norm_innov = float(np.sqrt(innov_pos[0]**2 + innov_pos[1]**2)) / diag

        # Step 3: 自适应因子，归一化残差超出阈值时启动
        alpha = max(1.0, norm_innov / self._akf_innov_thr)
        alpha = min(alpha, self._akf_alpha_max)

        # Step 4: 膨胀先验协方差 + 收缩位置观测噪声
        P_adapted = P_current.copy()
        R_adapted = R_k.copy()
        if alpha > 1.0:
            P_adapted[:2, :2] *= alpha          # 位置协方差膨胀
            P_adapted[4:6, 4:6] *= alpha        # 速度协方差同步膨胀
            r_scale = 1.0 / (1.0 + (alpha - 1.0) * self._akf_r_gain)
            R_adapted[0, 0] *= r_scale          # cx 观测噪声收缩
            R_adapted[1, 1] *= r_scale          # cy 观测噪声收缩

        # ── 标准 KF 更新 (使用自适应后的矩阵) ──
        # S_k: 创新协方差矩阵
        # S_k = H_k * P_adapted * H_k^T + R_adapted
        S_k = np.dot(self.H_k, np.dot(P_adapted, self.H_k.T)) + R_adapted

        # 卡尔曼增益: K_k
        K_k = np.dot(np.dot(P_adapted, self.H_k.T), np.linalg.inv(S_k))

        # 最优预测状态向量值
        # X^_k = X_k + K_k * innovation
        x_new = x + np.dot(K_k, innovation)

        P_result = np.dot(self.I - np.dot(K_k, self.H_k), P_adapted)

        # ── 速度直注入（带死区 + 钳位） ──
        # 仅在归一化残差超过死区时注入，过滤静止/低速时的 YOLO 检测噪声
        if dt > 1e-6 and self._vel_inject_gain > 0 and norm_innov > self._vel_inject_deadzone:
            vel_inject = innovation[:2] / dt * self._vel_inject_gain
            # 钳位：限制单次注入幅值，防止消失前异常残差导致飞框
            inject_speed = float(np.sqrt(vel_inject[0]**2 + vel_inject[1]**2))
            if inject_speed > self._vel_inject_max:
                vel_inject *= self._vel_inject_max / inject_speed
            x_new[4] += vel_inject[0]
            x_new[5] += vel_inject[1]

        # ── [debug] update 诊断输出 ──
        if self._debug:
            k_cx, k_cy = float(K_k[0, 0]), float(K_k[1, 1])
            print(f"[BBoxKF update] "
                  f"dist={dist:.1f} "          # ← 新增：正常<10, 遮挡恢复>30                                                           
                  f"pred=({x[0]:.1f},{x[1]:.1f}) "
                  f"obs=({z[0]:.1f},{z[1]:.1f}) "
                  f"out=({x_new[0]:.1f},{x_new[1]:.1f}) | "
                  f"innov=({innovation[0]:.1f},{innovation[1]:.1f}) "
                  f"norm={norm_innov:.3f} alpha={alpha:.2f} "
                  f"K=({k_cx:.3f},{k_cy:.3f}) | "
                  f"vel=({x_new[4]:.1f},{x_new[5]:.1f})")
                    



        # 跳变检测：YOLO 观测点和卡尔曼预测点中心距离超过 150 像素时硬重置
        dist = np.linalg.norm(x[:2] - z[:2])
        if dist > 150:
            # 大残差回退：重置位置与尺度，保留平移速度连续性
            x_new[:4] = z

            # 保留并衰减平移速度，避免”速度归零”导致后续几乎不动
            x_new[4:6] = 0.7 * x[4:6]

            # 尺度速度在突变时清零，防止宽高发散
            x_new[6:8] = 0.0

            return x_new, np.eye(self.n)


        return x_new, P_result