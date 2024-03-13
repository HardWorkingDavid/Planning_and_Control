## 运动学模型及其线性化

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/4202f54a26ea479fb8c2729a5c08a889.png#pic_center)

$v$：无人车的速度

$\dot x$：无人车在世界坐标系中X轴方向上的分速度

$\dot y$：无人车在世界坐标系中Y轴方向上的分速度

$\theta$：无人车在世界坐标中的航向角

$\dot \theta$：无人车的角速度

两个主要控制对象：$v$、$w$

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/153fcd2a627f44688a3cf5a90617f7f0.png#pic_center)

运动学模型线性化:
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/548ff20b17a14979b308787fdb06430a.png#pic_center)

测量具有离散的本质，现有的物理系统只能被离散的观测与控制。时间维度上的状态都被离散化为一个个状态序列$x_0,x_1,...,x_t,...$。物理系统本质上是连续的，那些状态更新间隔，因为仅仅能够离散的控制、影响这些系统。

模型的离散化是对物理系统在时间维度的近似。离散化后的差分模型与系统真实的连续模型是有误差的。

有误差，还要将就采用离散的差分形式？
>**不能立马**是指控制指令并不能立马从当前值变成期望的控制输入值，就算用连续模型加上阶梯形式控制输入函数计算状态的响应输出也是不准确的。与其不准，不如用形式相对简洁的差分模型。另外，并不是所有的系统模型都是可导的，这使得绝大多数情况是被迫采用差分模型。

>**差不多就可以** 由于控制是按一定的周期观测系统内部真实的状态，也即系统的状态初值是不断更新。如果我们的更新周期不是太长，在短时间内，差分函数的状态预测误差是在可接受的范围。

推导过程：
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/46b1c630c7ef4373bc6b49b97fef5a9d.png#pic_center)

无人车运动学模型**在低速时可以较好的预测（估计）系统未来的状态，但是随着车速的提高，汽车侧向动力学特性的影响越来越明显**。因此，**在高速运行时，一个更加准确的无人车模型必须加入汽车的动力学特性**。

## LQR

参考博客：

（1）[LQR的理解与运用 第一期——理解篇](https://blog.csdn.net/weixin_51772802/article/details/128767706?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522170813441316800227417406%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=170813441316800227417406&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-128767706-null-null.142%5Ev99%5Econtrol&utm_term=LQR&spm=1018.2226.3001.4187)

（2）[线性二次型调节器(LQR)原理详解](https://blog.csdn.net/qq_36133747/article/details/123413115?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522170813441316800227417406%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=170813441316800227417406&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_click~default-2-123413115-null-null.142%5Ev99%5Econtrol&utm_term=LQR&spm=1018.2226.3001.4187)

（3）[LQR控制基本原理（包括Riccati方程具体推导过程）](https://blog.csdn.net/weixin_43487974/article/details/127195643?utm_medium=distribute.pc_relevant.none-task-blog-2~default~baidujs_utm_term~default-2-127195643-blog-123413115.235%5Ev43%5Epc_blog_bottom_relevance_base9&spm=1001.2101.3001.4242.2&utm_relevant_index=5)

（4）[【基础】自动驾驶控制算法第五讲 连续方程的离散化与离散LQR原理](https://www.bilibili.com/video/BV1P54y1m7CZ/?spm_id_from=333.999.0.0&vd_source=0abba1bacb70946046a9777385d69921)

### 1 LQR算法原理

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/37f13f38591f471f82770d19c9674a84.png#pic_center)
让系统稳定的条件是矩阵$A_{cl}$的特征值实部均为负数。因此我们可以手动选择几个满足上述条件的特征值，然后反解出K，从而得到控制器。

代价函数 $J$

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/565a781074b04008b9b5d23caf5a8126.png#pic_center)
在系统稳定的前提下，通过设计合适的K，让代价函数J最小。

Q大：希望状态变量x更快收敛

R大：希望输入量u收敛更快，以更小的代价实现系统稳定

#### 1.1 连续时间LQR推导

求解连续时间LQR反馈控制器参数K的过程：

（1）设计参数矩阵Q、R

（2）求解Riccati方程$A^TP+PA-PBR^{-1}B^TP+Q=0$得到P

（3）计算$K=R^{-1}B^TP$得到反馈控制量$u=-kx$

#### 1.2 离散时间LQR推导
离散系统：

$x(K+1)=Ax(k)+Bu(k)$

代价函数：
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/96cb703fa5a44247991a2f3434ca538e.png)

设计步骤：

① 确定迭代范围N

② 设置迭代初始值$P_N=Q$

③ $t=N,...,1$，从后向前循环迭代求解离散时间的代数Riccati方程

$$
P_{t-1}=Q+A^TP_tA-A^TP_tB(R+B^TP_{t+1}B)^{-1}B^TP_tA
$$

④ $t=0,...,N$循环计算反馈系数$K_t=(R+B^TP_{t+1}B)^{-1}B^TP_{t+1}A$ 得到控制量$u_t=-K_tx_t$

#### 1.3 LQR主要步骤

（1）确定迭代范围N，预设精度EPS

（2）设置迭代初始值P = Qf，Qf = Q

（3）循环迭代，$t=1,...,N$

$$
P _{new} =Q+A ^TPA−A ^TPB(R+B ^T PB) 
^{−1}B ^TPA
$$

若$||P_{new}-P||<EPS$：跳出循环；否则：$P=P_{new}$

（4）计算反馈系数$K=(R + B^TP_{new}B)^{-1}B^TP_{new}A$

（5）最终的优化控制量$u^*=-Kx$


## PID

### 0 单环PID
目标位置→系统→速度→当前位置

输入目标位置，得到输出的位置是当前位置作为反馈量，而这个输出结果的位置量是我们通过控制中间过程量速度来控制的

`单环控制的是速度量`
### 1 PID双环控制
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/4a079058fa884f9b938b157d35285b02.png#pic_center)

位置环做为外环，速度环作为内环。外环的输出值作为内环的目标值，外环计算一次pid，根据pid公式可以知道，当离位置目标越来越近时，第一个误差(外环误差)就越来越小，以至于输出的内环目标值就越来越小，所以最终达到的效果就是，离目标值越近，速度越小。（理想情况下，要调节好pid参数，不然系统也会崩溃）

>双环控制的时候，外环PID参数调节幅度不要太大，这对于整个曲线的影响很大

代码实现步骤：

① **PID参数结构体**：定义位置、速度闭环的PID参数结构体变量

② **初始化PID参数**：把目标值、期望值、累计偏差清零，配置PID系数
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/7a47ff4b0b864848938b6d8da0095ade.png)

③ **设置目标位置**：只要设置外环的，内环的不用设置，因为内环的目标值，就是外环的输出值。

④ **PID双环控制**
在定时器中断（1ms）里，每50ms计算一次当前编码器的总计数值，并通过这个值进行外环PID
（位置环）计算，得到目标速度，要经过速度限幅(防止速度过快)，然后存放到g_motor_data.motor_pwm变量(临时存放而已)，根据目标速度，再进行速度环PID计算，最终得出要输出的pwm比较值，存放g_motor_data.motor_pwm变量(最终存放)，然后再限制pwm比较值输出
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/4593dc7f028046548dba41ef23d07bb9.png)
### 3 PID双环控制详细解析
如果采用PID双环控制，控制的其实就是加速度。如果直接输入目标位置，输出当前位置，控制直接是用加速度来控制这个位置显然是不稳定的。==所以我们采用双环==，用外环来输出目标速度，用内环来控制加速度从而来控制位置

总体流程：目标位置→系统1→目标速度→系统2→加速度→当前位置和速度→反馈回系统1和2

**外环**：目标位置→系统1→目标速度→反馈给系统1

>外环控制的是速度，也就是通过输入目标位置，来控制速度，从而反馈给系统，当我们把目标速度调的比较稳定后，再把此输出的目标速度当作内环的目标值传进内环中

**内环**：目标速度→系统2→加速度→当前位置→反馈给系统2

内环是把外环的目标速度作为目标值，根据当前的速度控制加速度从而向目标加速度靠近，最后再把输出值作为反馈值

### 4 双环PID控制器的C++代码实现

```cpp
float outer_pid_controller(float setpoint, float input, float kp, float ki, float kd, float dt) {
    static float integral = 0;
    static float prev_error = 0;
    float error = setpoint - input;

    // 计算积分项
    integral += error * dt;

    // 计算微分项
    float derivative = (error - prev_error) / dt;

    // 计算输出
    float output = kp * error + ki * integral + kd * derivative;

    // 更新上一个误差
    prev_error = error;

    return output;
}

float inner_pid_controller(float setpoint, float input, float kp, float ki, float kd, float dt) {
    static float integral = 0;
    static float prev_error = 0;
    float error = setpoint - input;

    // 计算积分项
    integral += error * dt;

    // 计算微分项
    float derivative = (error - prev_error) / dt;

    // 计算输出
    float output = kp * error + ki * integral + kd * derivative;

    // 更新上一个误差
    prev_error = error;

    return output;
}

float double_pid_controller(float setpoint, float input, float outer_kp, float outer_ki, float outer_kd, float inner_kp, float inner_ki, float inner_kd, float dt) {
    // 计算外环控制器输出
    float outer_output = outer_pid_controller(setpoint, input, outer_kp, outer_ki, outer_kd, dt);

    // 计算内环控制器输出
    float inner_output = inner_pid_controller(outer_output, input, inner_kp, inner_ki, inner_kd, dt);

    return inner_output;
}
```
`outer_pid_controller`和`inner_pid_controller`分别是外环和内环PID控制器的实现函数。

`double_pid_controller`函数则是将两个PID控制器串联起来，实现双环PID控制器。其中，`setpoint`是设定值，`input`是输入值，`outer_kp`、`outer_ki`和`outer_kd`是外环控制器的比例、积分和微分系数，`inner_kp`、`inner_ki`和`inner_kd`是内环控制器的比例、积分和微分系数，dt是采样时间。该函数返回内环PID控制器的输出值。

百度Apollo纵向控制原理如下所示，该图可以为纵向控制器的设置提供参考：
![百度Apollo纵向控制框架](https://img-blog.csdnimg.cn/93ad858fcacc4c4caa26c865358ec5f1.png)

>可以看出纵向控制是**基于Frenet坐标系**的，位置跟踪控制器采用P控制器实现车辆位置闭环控制，速度跟踪控制器实现速度闭环控制，根据车辆的俯仰角得出坡道加速度补偿，以及预览点的加速度实现加速度开环控制。基于加速度和定位反馈纵向速度查找油门制动标定表得到油门和刹车的控制量，从而实现车辆的纵向控制。


#### 6 油门刹车标定表
纵向控制逻辑：==油门 --> 功率 --> 转速 / 扭矩 --> 车速 / 车加速度 --> 车加速==

找到油门和v，a对应的关系
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/9c878138514741968da7e2a46186f186.png)

做实验，踩不同的油门，得到不同的v，a曲线

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/773ba8af21454962a6cfb72660a03b18.png)

对于一个throttle，不同的时间 t 得到一系列v、a点，v，a可以合并，得到v，a曲线
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/6637d6c29f2c420f993a27bbdef8f97a.png)

不同的throttle会得到不同的v、a曲线
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/e1b34141d8cb4c7ab5ea9ed4d759f62b.png)

使用不同的throttle做实验，可以得到一个三维曲面

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/0f4ae504f4c14ebc9d1bab51fa9a24ff.png)

通过做实验，得到大量的（v，a，throttle）的三维点，从而拟合出throttle = f(v,a)

### 7 PID总结

任何闭环控制系统的首要任务是要**稳、准、快**的响应命令。PID的主要工作就是如何实现这一任务。

PID控制器的比例单元 ( P) 、积分单元(I)和微分单元(D)分别对应**目前误差、过去累计误差及未来误差**。若是不知道受控系统的特性，一般认为PID控制器是最适用的控制器。

>P:增大比例加快系统的响应，它的作用于**输出值较快，但不能很好稳定在一个理想的数值**。Kp过大，会产生超调，并产生振荡。

>I:在P的基础上消除余差，对稳定后有累积误差的系统进行误差修整，**减小稳态误差**。

>D:可以使系统超调量减小，减小振荡，增加稳定性。

位置式PID：当前系统的实际位置，与你想要达到的预期位置的偏差，进行PID控制

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/f4f0657f1e624c43af63f4779870bd72.png#pic_center)当采样时间足够小时，能够获得最够精确的结果，离散控制过程与连续过程非常接近。

位置式PID在积分项达到饱和时,误差仍然会在积分作用下继续累积，一旦误差开始反向变化，系统需要一定时间从饱和区退出，所以在u(k)达到最大和最小时，要停止积分作用，并且要有积分限幅和输出限幅。

抗积分饱和：**如果上一次的输出控制量超过了饱和值，饱和值为正，则这一次只积分负的偏差，饱和值为负，则这一次只积分正的偏差，从而避免系统长期留在饱和区！**
