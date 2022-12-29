#! https://zhuanlan.zhihu.com/p/595270760
# Quick Note on OSQP

OSQP 是一个 QP 问题的求解器，即主要用于求解如下所示形式的问题：

$$
\begin{alignat*}{3}
    &\min_{x}\quad &&\frac{1}{2}x^TPx + q^Tx + r\\
    &\text{s.t.} && l\leq Gx \leq u \\
\end{alignat*}
$$

举一个例子：

$$
\begin{alignat*}{3}
    &\min_{x}\quad &&\frac{1}{2}x^T\begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix}x\\
    &\text{s.t.} && \begin{bmatrix} 1 \\ 1 \end{bmatrix}x =1 \\
\end{alignat*}
$$

再具体化就是当 $x + y = 1$ 时，求解 $x^2 + y^2$ 的最小值。人肉计算器告诉我们答案是 $x = y = 0.5$

用 python 里的 OSQP 求解器求解这个问题，代码如下：

```python
import osqp
import numpy as np
from scipy import sparse

# Define problem data
P = sparse.csc_matrix([[1, 0], [0, 1]])
A = sparse.csc_matrix([[1, 1]])
l = np.array([1])
u = np.array([1])

# Create an OSQP object
prob = osqp.OSQP()

# Setup workspace and change alpha parameter
prob.setup(P=P, q=None, A=A, l=l, u=u, verbose=False, alpha = 1.0)

# Solve problem
res = prob.solve()
print(res.x)
```

打印出来是：

```
[0.5 0.5]
```