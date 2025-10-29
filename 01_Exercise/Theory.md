## 1.1.1
#### a)
Multiplications with these matrices the vectors are adapted to a new position while maintaining the the geometrical shape of the form.

**Rotation Matrix** : Rotates every vector by a specified angle $\theta$. (Angles between vectors are preserved and thus shape is maintained)
For a 3D rotation on the z Axis (therefore rows and columns = 0 except for the last one):
$$
\begin{bmatrix}
\cos \theta && -\sin \theta && 0\\
-\sin \theta && \cos \theta && 0 \\
0 && 0 && 1
\end{bmatrix}
$$
**Translation Matrix**: Moves every vector to a new (x,y,z) position. 
- $TA = B$ 
For a 2D translation with a Matrix $T=$ 3x3  and $A=$ 1x3
$$
\begin{bmatrix}
1 && 0 && t_{x} \\
0 && 1 && t_{y} \\
0 && 0 && 1

\end{bmatrix}
\begin{bmatrix}
a_{x} \\
a_{y} \\
1
\end{bmatrix}
=
\begin{bmatrix}
a_{x} + t_{x} \\
a_{y} + t_{y} \\
1
\end{bmatrix}
$$
The diagonal on the Translation Matrix is equal to 1. Otherwise it would also scale the Matrix A

**Scaling Matrix**: This Matrix "stretches" or "compresses" points on a matrix
- $SA=B$
For a 3D Scaling: The Matrix $S$ has on its diagonal the the scaling values per axis
$$
\begin{bmatrix}
s_{x} && 0 && 0 \\
0 && s_{y} && 0 \\
0 && 0 && s_{z}
\end{bmatrix}
$$


**Reflection Matrix** This Matrix will reflect a point having a Axis as pivot
- Elements on the diagonal have to be -1 (for a reflection without scaling), which inverses orientation  (except for the element on the desired Axis and the last one)
- For a 2D reflection on the x Axis with a Matrix $R$ (an extra last element is added $1_{33}$)
$$
\begin{bmatrix}
 1 && 0 && 0 \\
0 && -1  && 0 \\
0 && 0 && 1
\end{bmatrix}
$$


If a transform Matrix applies an operation on a Matrix, like translation. Then the inverse of  a transform Matrix will undo this operation. Using the same example of a translation, then the Inverse of the translation Matrix will return the Matrix to its original position


#### b)
Commuting
- $T_{1} \cdot T_{2}$ : A sequence of translations is the addition of translation vectors, which is commutative
- $S_{1}\cdot S_{2}$ : The scaling Matrices can be summarized into one (multiplied) Matrix and multiplication is commutative

Non-Commuting 
- $R\cdot T$  : This operation is not commutative. The final position depends on the order of the operations, e.g.: given a point (1,0), rotate it 90 degrees counter-clockwise around the origin, to get (0,1). Then translate it with (0,5) and get (0,6). Otherwise, translate it first with (0,5), and get (1,5). Then rotate it 90 degrees and get (-5,1)
- $S\cdot T$ : Not commutative, because translating a point (adding a vector) and then scaling it (multiplying) causes a different  result than first multiplying and then adding


Special 
- $R_{1}\cdot R_{2}$ : Can be commutative if the rotation is on the same axis (then the rotated degrees add together). Otherwise, if the rotation occurs on different axis, this operation is not commutative. The result is different if it is rotated on the x-Axis and then on the y-Axis than first on the y-Axis and then x-Axis
- $R\cdot S$ : Can be commutative if the scaling is uniform on all axis. Otherwise not. Rotating a non-uniformly scaled object is different from non-uniformly scaling a rotated object

## 1.1.2
### a)
We can use the following Matrix to swap the y and z values of Matrix P
$$
\begin{bmatrix}
1 && 0 && 0 \\
0 && 0 && 1 \\
0 && 1 && 0 \\
\end{bmatrix}
\cdot P=P'
$$
### b) 
For a Matrix $P =3\times n$ then it has to be multiplied with a vector with $n$ rows
$$
P_{3\times n} \cdot v_{n\times 1} = 
P \cdot
\begin{bmatrix}
1 \\
1 \\
\dots \\
1
\end{bmatrix}
= P'_{3\times 1}
$$
and $P'$ being the sum of all points (P' as a point itself)

