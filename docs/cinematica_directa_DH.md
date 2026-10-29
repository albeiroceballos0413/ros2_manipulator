# Cinemática directa del manipulador planar RRR mediante Denavit-Hartenberg

**Proyecto:** `ros2_manipulator` · paquete `arm_kinematics`
**Semana 1–2 — Robótica**

---

## 1. ¿Qué es el método de Denavit-Hartenberg?

### 1.1 El problema que resuelve

Un manipulador serial es una cadena de eslabones rígidos unidos por articulaciones. La **cinemática directa** responde a esta pregunta:

> Si conozco los valores de las articulaciones $q = (q_1, q_2, \dots, q_n)$, ¿dónde está el efector final y cómo está orientado?

Para responderla se fija un sistema de coordenadas (*frame*) a cada eslabón y se encadenan las transformaciones entre frames consecutivos:

$$
T_0^n = A_1(q_1)\,A_2(q_2)\cdots A_n(q_n)
$$

Una transformación rígida general en el espacio tiene **6 grados de libertad** (3 de traslación y 3 de rotación). Si cada frame se colocara de forma arbitraria, harían falta 6 números por eslabón. Además, cada persona pondría los frames de manera distinta, lo que dificulta comparar resultados.

### 1.2 La idea de Denavit y Hartenberg (1955)

Jacques Denavit y Richard Hartenberg propusieron unas **reglas para colocar los frames**. Con esas reglas, la transformación entre dos frames consecutivos queda descrita con solo **4 parámetros**:

| Parámetro | Nombre | Definición (convención clásica / *standard*) | Tipo |
|---|---|---|---|
| $\theta_i$ | ángulo de articulación | Rotación **alrededor de $z_{i-1}$** que lleva $x_{i-1}$ hasta $x_i$ | Variable si la articulación es rotacional |
| $d_i$ | desplazamiento | Traslación **a lo largo de $z_{i-1}$** | Variable si la articulación es prismática |
| $a_i$ | longitud del eslabón | Distancia **a lo largo de $x_i$** entre los ejes $z_{i-1}$ y $z_i$ (normal común) | Constante (geometría) |
| $\alpha_i$ | torsión del eslabón | Rotación **alrededor de $x_i$** que lleva $z_{i-1}$ hasta $z_i$ | Constante (geometría) |

Bajar de 6 a 4 parámetros es posible porque las reglas imponen dos restricciones:

1. El eje $x_i$ debe ser **perpendicular** a $z_{i-1}$.
2. El eje $x_i$ debe **cortar** a $z_{i-1}$.

Cada restricción elimina un grado de libertad, así que quedan $6 - 2 = 4$ parámetros.

### 1.3 Reglas para colocar los frames (convención clásica)

1. **Eje $z_{i-1}$:** se alinea con el eje de movimiento de la articulación $i$. En una articulación rotacional es el eje de giro; en una prismática, la dirección de deslizamiento.
2. **Eje $x_i$:** va a lo largo de la **normal común** entre $z_{i-1}$ y $z_i$, apuntando de $z_{i-1}$ hacia $z_i$.
   - Si $z_{i-1}$ y $z_i$ son paralelos (es nuestro caso), hay infinitas normales comunes. Se elige la que simplifique el modelo, normalmente la que pasa por el centro de la articulación.
   - Si los ejes se cortan, $x_i$ se toma perpendicular al plano que forman.
3. **Origen $O_i$:** se ubica en la intersección de $x_i$ con $z_i$.
4. **Eje $y_i$:** queda definido por la regla de la mano derecha, $y_i = z_i \times x_i$.
5. **Frame 0 (base):** es fijo. Se suele elegir para que coincida con el frame 1 cuando $q_1 = 0$.
6. **Frame $n$ (efector final):** se pone en la punta de la herramienta.

### 1.4 Advertencia: existen dos convenciones

| | DH clásica (*standard*) | DH modificada (Craig) |
|---|---|---|
| Libro de referencia | Spong, Siciliano | Craig |
| Ubicación del frame $i$ | Al **final** del eslabón $i$ | Al **inicio** del eslabón $i$ |
| Orden de la transformación | $R_z(\theta)\,T_z(d)\,T_x(a)\,R_x(\alpha)$ | $R_x(\alpha_{i-1})\,T_x(a_{i-1})\,R_z(\theta_i)\,T_z(d_i)$ |

Las dos convenciones dan **el mismo resultado final** $T_0^n$, pero las tablas y las matrices intermedias son distintas. Aquí usamos la **clásica**, que es la que está implementada en `kinematics.py`. Cuando leas un artículo o un libro, verifica primero qué convención usa.

---

## 2. Deducción de la matriz homogénea general $A_i$

En la convención clásica, $A_i$ es el producto de cuatro movimientos elementales, aplicados en este orden:

$$
A_i = R_z(\theta_i)\; T_z(d_i)\; T_x(a_i)\; R_x(\alpha_i)
$$

Usaremos la notación abreviada $c_\theta = \cos\theta_i$, $s_\theta = \sin\theta_i$, $c_\alpha = \cos\alpha_i$ y $s_\alpha = \sin\alpha_i$.

### 2.1 Las cuatro matrices elementales

$$
R_z(\theta) =
\begin{bmatrix}
c_\theta & -s_\theta & 0 & 0\\
s_\theta & c_\theta & 0 & 0\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{bmatrix},
\qquad
T_z(d) =
\begin{bmatrix}
1 & 0 & 0 & 0\\
0 & 1 & 0 & 0\\
0 & 0 & 1 & d\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$
T_x(a) =
\begin{bmatrix}
1 & 0 & 0 & a\\
0 & 1 & 0 & 0\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{bmatrix},
\qquad
R_x(\alpha) =
\begin{bmatrix}
1 & 0 & 0 & 0\\
0 & c_\alpha & -s_\alpha & 0\\
0 & s_\alpha & c_\alpha & 0\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

### 2.2 Paso 1: $R_z(\theta)\,T_z(d)$

$T_z(d)$ solo añade $d$ en la componente $z$ de la traslación. Como $R_z$ no altera el eje $z$, el producto es:

$$
R_z(\theta)\,T_z(d) =
\begin{bmatrix}
c_\theta & -s_\theta & 0 & 0\\
s_\theta & c_\theta & 0 & 0\\
0 & 0 & 1 & d\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

### 2.3 Paso 2: multiplicar por $T_x(a)$

La columna de traslación pasa a ser la matriz anterior aplicada al punto $(a, 0, 0, 1)^T$:

$$
\begin{bmatrix}
c_\theta & -s_\theta & 0 & 0\\
s_\theta & c_\theta & 0 & 0\\
0 & 0 & 1 & d\\
0 & 0 & 0 & 1
\end{bmatrix}
\begin{bmatrix} a\\0\\0\\1 \end{bmatrix} =
\begin{bmatrix} a\,c_\theta\\ a\,s_\theta\\ d\\ 1 \end{bmatrix}
$$

Por lo tanto:

$$
R_z(\theta)\,T_z(d)\,T_x(a) =
\begin{bmatrix}
c_\theta & -s_\theta & 0 & a\,c_\theta\\
s_\theta & c_\theta & 0 & a\,s_\theta\\
0 & 0 & 1 & d\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

### 2.4 Paso 3: multiplicar por $R_x(\alpha)$

$R_x(\alpha)$ deja intacta la primera columna y la traslación, y mezcla la segunda y la tercera columna de la parte de rotación:

- Columna 2: $c_\alpha\,(\text{col}_2) + s_\alpha\,(\text{col}_3) = (-s_\theta c_\alpha,\; c_\theta c_\alpha,\; s_\alpha)^T$
- Columna 3: $-s_\alpha\,(\text{col}_2) + c_\alpha\,(\text{col}_3) = (s_\theta s_\alpha,\; -c_\theta s_\alpha,\; c_\alpha)^T$

### 2.5 Resultado: la matriz DH general

$$
\boxed{
A_i =
\begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i\\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i\\
0 & \sin\alpha_i & \cos\alpha_i & d_i\\
0 & 0 & 0 & 1
\end{bmatrix}}
$$

Esta es exactamente la matriz que construye la función `dh_transform(a, alpha, d, theta)`.

**Estructura de la matriz.** Toda matriz homogénea tiene la forma

$$
A = \begin{bmatrix} R_{3\times3} & p_{3\times1}\\ 0_{1\times3} & 1 \end{bmatrix},
$$

donde $R$ es la **orientación** del frame $i$ vista desde el frame $i-1$, y $p$ es la **posición** de su origen.

---

## 3. Aplicación al manipulador planar RRR

### 3.1 Geometría

```
            y0
            ^                         O3 (efector final)
            |                     L3  *
            |                       /
            |           O2 *------*        θ3 = q3
            |         L2  /
            |            /  θ2 = q2
            |      O1   *
            |     L1  /
            |       /  θ1 = q1
            |     /
       O0   *--------------------------> x0
          (base)
```

- Tres articulaciones **rotacionales**. Todos los ejes $z_0, z_1, z_2$ son **paralelos** entre sí y salen del plano del papel.
- Longitudes: $L_1 = 0{,}30\ \text{m}$, $L_2 = 0{,}25\ \text{m}$, $L_3 = 0{,}10\ \text{m}$.
- El movimiento ocurre completamente en el plano $x_0$–$y_0$.

### 3.2 Colocación de los frames

| Frame | Origen | $z_i$ | $x_i$ |
|---|---|---|---|
| 0 | Articulación 1 (base) | Eje de giro de la articulación 1, saliendo del papel | Horizontal hacia la derecha (referencia fija) |
| 1 | Articulación 2 | Eje de giro de la articulación 2, paralelo a $z_0$ | A lo largo del eslabón 1, de $O_0$ hacia $O_1$ |
| 2 | Articulación 3 | Eje de giro de la articulación 3, paralelo a $z_1$ | A lo largo del eslabón 2 |
| 3 | Punta (efector final) | Paralelo a $z_2$ | A lo largo del eslabón 3 |

Como los ejes $z$ son paralelos, la **normal común** entre ellos es la línea del eslabón. Por eso cada $x_i$ apunta a lo largo de su eslabón.

### 3.3 Parámetros DH

- $\alpha_i = 0$: los ejes $z_{i-1}$ y $z_i$ son paralelos, así que no hay torsión.
- $d_i = 0$: los ejes $x_{i-1}$ y $x_i$ están en el mismo plano, así que no hay desplazamiento a lo largo de $z$.
- $a_i = L_i$: la distancia entre ejes consecutivos es la longitud del eslabón.
- $\theta_i = q_i$: es la variable articular.

| $i$ | $a_i$ | $\alpha_i$ | $d_i$ | $\theta_i$ |
|---|---|---|---|---|
| 1 | $L_1 = 0{,}30$ | $0$ | $0$ | $q_1$ |
| 2 | $L_2 = 0{,}25$ | $0$ | $0$ | $q_2$ |
| 3 | $L_3 = 0{,}10$ | $0$ | $0$ | $q_3$ |

### 3.4 Matrices individuales

Al sustituir $\alpha_i = 0$ ($\cos\alpha_i = 1$, $\sin\alpha_i = 0$) y $d_i = 0$ en la matriz general:

$$
A_i =
\begin{bmatrix}
c_i & -s_i & 0 & L_i c_i\\
s_i & c_i & 0 & L_i s_i\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{bmatrix},
\qquad c_i = \cos q_i,\; s_i = \sin q_i
$$

Es una rotación pura alrededor de $z$, seguida de un avance $L_i$ a lo largo del nuevo eje $x$.

### 3.5 Producto $A_1 A_2$

Las entradas se calculan una por una:

$$
A_1 A_2 =
\begin{bmatrix}
c_1 & -s_1 & 0 & L_1 c_1\\
s_1 & c_1 & 0 & L_1 s_1\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
c_2 & -s_2 & 0 & L_2 c_2\\
s_2 & c_2 & 0 & L_2 s_2\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

- $(1,1)$: $c_1 c_2 - s_1 s_2 = \cos(q_1+q_2) \equiv c_{12}$
- $(1,2)$: $-c_1 s_2 - s_1 c_2 = -\sin(q_1+q_2) \equiv -s_{12}$
- $(2,1)$: $s_1 c_2 + c_1 s_2 = s_{12}$
- $(2,2)$: $-s_1 s_2 + c_1 c_2 = c_{12}$
- $(1,4)$: $c_1 (L_2 c_2) - s_1 (L_2 s_2) + L_1 c_1 = L_1 c_1 + L_2 c_{12}$
- $(2,4)$: $s_1 (L_2 c_2) + c_1 (L_2 s_2) + L_1 s_1 = L_1 s_1 + L_2 s_{12}$

En estos pasos se usaron las identidades de suma de ángulos:

$$
\cos(a+b) = \cos a\cos b - \sin a \sin b,\qquad
\sin(a+b) = \sin a\cos b + \cos a\sin b
$$

El resultado es:

$$
T_0^2 = A_1 A_2 =
\begin{bmatrix}
c_{12} & -s_{12} & 0 & L_1 c_1 + L_2 c_{12}\\
s_{12} & c_{12} & 0 & L_1 s_1 + L_2 s_{12}\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

**Interpretación:** en un brazo planar, los ángulos de rotación **se suman**. La orientación del eslabón 2 respecto a la base es $q_1 + q_2$.

### 3.6 Producto $T_0^2 A_3$: la cinemática directa completa

Se repite el mismo procedimiento con $A_3$:

- $(1,1)$: $c_{12} c_3 - s_{12} s_3 = c_{123}$
- $(2,1)$: $s_{12} c_3 + c_{12} s_3 = s_{123}$
- $(1,4)$: $c_{12}(L_3 c_3) - s_{12}(L_3 s_3) + L_1 c_1 + L_2 c_{12} = L_1 c_1 + L_2 c_{12} + L_3 c_{123}$
- $(2,4)$: $s_{12}(L_3 c_3) + c_{12}(L_3 s_3) + L_1 s_1 + L_2 s_{12} = L_1 s_1 + L_2 s_{12} + L_3 s_{123}$

$$
\boxed{
T_0^3 = A_1 A_2 A_3 =
\begin{bmatrix}
c_{123} & -s_{123} & 0 & L_1 c_1 + L_2 c_{12} + L_3 c_{123}\\
s_{123} & c_{123} & 0 & L_1 s_1 + L_2 s_{12} + L_3 s_{123}\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}}
$$

donde $c_{12} = \cos(q_1+q_2)$ y $c_{123} = \cos(q_1+q_2+q_3)$, y lo mismo para los senos.

### 3.7 Pose del efector final

De la cuarta columna de $T_0^3$ se obtiene la **posición**:

$$
\begin{aligned}
x &= L_1\cos q_1 + L_2\cos(q_1+q_2) + L_3\cos(q_1+q_2+q_3)\\
y &= L_1\sin q_1 + L_2\sin(q_1+q_2) + L_3\sin(q_1+q_2+q_3)
\end{aligned}
$$

La **orientación** en el plano se obtiene del bloque de rotación con `atan2`. Se usa `atan2` en lugar de `acos` porque distingue los cuatro cuadrantes:

$$
\varphi = \operatorname{atan2}(r_{21},\, r_{11}) = \operatorname{atan2}(s_{123},\, c_{123}) = q_1 + q_2 + q_3
$$

**Interpretación geométrica:** la posición de la punta es la **suma vectorial** de los tres eslabones. El eslabón $k$ apunta en la dirección acumulada $q_1 + \dots + q_k$. Para un brazo planar podría escribirse directamente con trigonometría. DH lleva al mismo resultado con un procedimiento sistemático, que funciona igual para brazos espaciales de 6 grados de libertad, donde la intuición geométrica ya no alcanza.

---

## 4. Verificación numérica

### 4.1 Casos fáciles de comprobar a mano

| $q_1$ | $q_2$ | $q_3$ | $x$ [m] | $y$ [m] | $\varphi$ | Razonamiento |
|---|---|---|---|---|---|---|
| 0° | 0° | 0° | 0,65 | 0,00 | 0° | Brazo extendido sobre $x_0$: $0{,}30+0{,}25+0{,}10$ |
| 90° | 0° | 0° | 0,00 | 0,65 | 90° | Brazo extendido sobre $y_0$ |
| 0° | 90° | 0° | 0,30 | 0,35 | 90° | Eslabón 1 sobre $x_0$; eslabones 2 y 3 hacia arriba: $0{,}25+0{,}10$ |

### 4.2 Caso general: $q = (30^\circ,\ 45^\circ,\ -30^\circ)$

Ángulos acumulados: $q_1 = 30^\circ$, $q_1+q_2 = 75^\circ$, $q_1+q_2+q_3 = 45^\circ$.

$$
\begin{aligned}
x &= 0{,}30\cos 30^\circ + 0{,}25\cos 75^\circ + 0{,}10\cos 45^\circ\\
  &= 0{,}30(0{,}8660) + 0{,}25(0{,}2588) + 0{,}10(0{,}7071)\\
  &= 0{,}2598 + 0{,}0647 + 0{,}0707 = \mathbf{0{,}3952\ m}\\[4pt]
y &= 0{,}30\sin 30^\circ + 0{,}25\sin 75^\circ + 0{,}10\sin 45^\circ\\
  &= 0{,}30(0{,}5000) + 0{,}25(0{,}9659) + 0{,}10(0{,}7071)\\
  &= 0{,}1500 + 0{,}2415 + 0{,}0707 = \mathbf{0{,}4622\ m}\\[4pt]
\varphi &= 30^\circ + 45^\circ - 30^\circ = \mathbf{45^\circ}
\end{aligned}
$$

Matriz completa calculada con `forward_kinematics` (coincide con el cálculo a mano):

$$
T_0^3 =
\begin{bmatrix}
0{,}7071 & -0{,}7071 & 0 & 0{,}3952\\
0{,}7071 & 0{,}7071 & 0 & 0{,}4622\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

---

## 5. Correspondencia con el código

| Matemática | `arm_kinematics/kinematics.py` |
|---|---|
| Matriz general $A_i(a_i, \alpha_i, d_i, \theta_i)$ | `dh_transform(a, alpha, d, theta)` |
| $T_0^3 = A_1 A_2 A_3$ | `forward_kinematics(q, lengths)`: producto acumulado `T = T @ A_i` |
| $(x,\ y,\ \varphi)$ | `end_effector_pose(q, lengths)`: `T[0,3]`, `T[1,3]`, `atan2(T[1,0], T[0,0])` |
| Verificación de la sección 4 | `test/test_kinematics.py` |

---

## 6. Observaciones para las próximas semanas

- **Espacio de trabajo.** La punta solo alcanza puntos con $\sqrt{x^2+y^2} \le L_1+L_2+L_3 = 0{,}65\ \text{m}$. El radio interior depende de las longitudes y de los límites articulares. Se estudia en la semana 5–6.
- **Redundancia.** En el plano, la posición $(x, y)$ tiene 2 grados de libertad y el brazo tiene 3. Si solo se pide la posición, hay infinitas soluciones de cinemática inversa. Si se pide también $\varphi$, hay como máximo dos: "codo arriba" y "codo abajo".
- **Singularidades.** Si la tarea es controlar $(x, y, \varphi)$, el determinante del jacobiano es $\det J = L_1 L_2 \sin q_2$. El brazo es singular cuando $q_2 = 0$ o $q_2 = \pi$, es decir, cuando los eslabones 1 y 2 quedan alineados (extendidos o plegados). En esas posturas pierde capacidad de moverse en alguna dirección. El ángulo $q_3$ no aparece en el determinante, porque la orientación $\varphi$ siempre se puede ajustar con la última articulación. Se estudia en la semana 5–6.
- **URDF frente a DH.** El formato URDF que usa ROS no usa parámetros DH. Describe cada articulación con un `origin` (posición y orientación) y un `axis`. La tabla DH se traduce de forma directa: en este robot, cada articulación va a $(L_i, 0, 0)$ de la anterior y gira alrededor de $z$.

---

## Referencias

- Spong, M. W., Hutchinson, S., & Vidyasagar, M. *Robot Modeling and Control*, 2.ª ed., Wiley, 2020. Cap. 3 (convención DH clásica).
- Craig, J. J. *Introduction to Robotics: Mechanics and Control*, 4.ª ed., Pearson, 2017. Cap. 3 (DH modificada).
- Denavit, J., & Hartenberg, R. S. "A kinematic notation for lower-pair mechanisms based on matrices." *ASME Journal of Applied Mechanics*, 22, 215–221, 1955.
