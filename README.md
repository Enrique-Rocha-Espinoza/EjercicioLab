# Python

Python es un lenguaje de programación de alto nivel, interpretado y de propósito general, es conocido por su sintaxis simple y legible, lo que facilita el aprendizaje y la escritura de código, Python es muy popular en áreas como el desarrollo web, la ciencia de datos, la inteligencia artificial y la automatización de tareas, su gran comunidad y amplia biblioteca de módulos y herramientas lo hacen versátil y poderoso para una variedad de aplicaciones, en esta introducción veremos elementos básicos para poder entender como funciona este lenguaje al igual que su sintaxis

## Tipos de variables

En Python, las variables pueden contener diferentes tipos de datos. Algunos de los tipos de variables más comunes incluyen:

- **Enteros (int):** Son números enteros, ya sea positivos, negativos o cero, sin decimales, ejemplo: 42, -7, 0.
```python
numero_entero = 10
```
- **Float (flotantes):** Son números reales con decimales, positivos o negativos, ejemplo: 3.14, -0.001, 2.0.
 ```python
numero_flotante = 3.14
```
- **Str (cadenas de texto):** Son valores lógicos que pueden ser True (verdadero) o False (falso), ejemplo: True, False.
```python
texto = "Hola, mundo"
```
- **bool (booleanos):** Son valores lógicos que pueden ser True (verdadero) o False (falso), ejemplo: True, False.
```python
valor_verdadero = True
valor_falso = False
```
- **complex (números complejos):** Son números que tienen una parte real y una parte imaginaria, representados como a + bj, donde a es la parte real y b es la parte imaginaria, ejemplo: 3 + 4j, -2 - 5j.
```python
numero_complejo = 3 + 4j
```

## Operadores Aritméticos

- `+` : Suma
- `-` : Resta
- `*` : Multiplicación
- `/` : División
- `//` : División entera
- `%` : Módulo (resto de la división)
- `**` : Potencia

Ejemplos:

```python
suma = 5 + 3       # 8
resta = 10 - 2     # 8
multiplicacion = 4 * 2  # 8
division = 16 / 2  # 8.0
division_entera = 17 // 2  # 8
modulo = 18 % 10   # 8
potencia = 2 ** 3  # 8
```
## Listas

Una lista en Python es una estructura de datos que permite almacenar una colección de elementos,  los elementos de una lista pueden ser de cualquier tipo, como números, cadenas de texto, o incluso otras listas, las listas son ordenadas, lo que significa que los elementos mantienen el orden en el que se añaden, y son mutables, lo que significa que se pueden modificar después de su creación.

La sintaxis para crear una lista en Python es usar corchetes `[]` y separar los elementos con comas. Por ejemplo:

```python
mi_lista = [1, 2, 3, 4, 5]
```

En este caso, `mi_lista` es una lista de cinco números enteros.

También puedes crear una lista vacía simplemente usando corchetes sin ningún elemento dentro:

```python
lista_vacia = []
```

Las listas en Python admiten una variedad de operaciones, como agregar elementos, eliminar elementos, acceder a elementos específicos, etc. Por ejemplo, para agregar un elemento al final de la lista, puedes usar el método `append()`:

```python
mi_lista.append(6)
```

Ahora, `mi_lista` sería `[1, 2, 3, 4, 5, 6]`.

- `lista[i]`: Accede al elemento de la lista que se encuentra en la posición `i`.
- `lista.pop(i)`: Elimina y devuelve el elemento en la posición `i` de la lista. Si no se especifica `i`, remueve el último elemento.
- `lista.append(elemento)`: Añade `elemento` al final de la lista.
- `lista.insert(i, elemento)`: Inserta `elemento` en la posición `i`, desplazando los demás elementos a la derecha.
- `lista.extend(lista2)`: Añade los elementos de `lista2` al final de `lista`.
- `lista.remove(elemento)`: Elimina la primera ocurrencia de `elemento` en la lista.
- `round(number, digits)`: Redondea `number` al número de `digits` decimales especificados. Si `digits` se omite, redondea al entero más cercano.

## for

En Python, un `for` es una estructura de control que se utiliza para iterar sobre una secuencia (como una lista, una tupla, un diccionario, un conjunto o una cadena) y ejecutar un bloque de código para cada elemento de la secuencia.

La sintaxis básica de un bucle `for` en Python es la siguiente:

```python
for elemento in secuencia:
    # Bloque de código a ejecutar para cada elemento
```

Aquí, `elemento` es una variable que toma el valor de cada elemento de la `secuencia` en cada iteración del bucle. El `# Bloque de código a ejecutar para cada elemento` es el código que se ejecutará para cada elemento de la secuencia.

Por ejemplo, si quieres imprimir cada número en una lista de números, puedes usar un bucle `for` de la siguiente manera:

```python
numeros = [1, 2, 3, 4, 5]
for numero in numeros:
    print(numero)
```

Este bucle `for` recorrerá la lista `numeros` e imprimirá cada `numero` en la lista.

También puedes usar la función `range()` para generar una secuencia de números y luego iterar sobre ellos con un bucle `for`. Por ejemplo, para imprimir los números del 0 al 4, puedes hacer lo siguiente:

```python
for i in range(5):
    print(i)
```

En este caso, `range(5)` genera una secuencia de números del 0 al 4, y el bucle `for` itera sobre esta secuencia, asignando cada número a la variable `i` e imprimiéndolo.


## If

En Python, un `if` es una estructura de control que se utiliza para tomar decisiones en el código. Permite ejecutar un bloque de código si se cumple una condición específica y, opcionalmente, ejecutar otro bloque de código si la condición no se cumple.

La sintaxis básica de una instrucción `if` en Python es la siguiente:

```python
if condicion:
    # Bloque de código a ejecutar si la condición es verdadera
```

Aquí, `condicion` es una expresión que se evalúa como verdadera (`True`) o falsa (`False`). Si la `condicion` es verdadera, se ejecutará el `# Bloque de código a ejecutar si la condición es verdadera`.

También puedes incluir una cláusula `else` para ejecutar un bloque de código si la condición no se cumple:

```python
if condicion:
    # Bloque de código a ejecutar si la condición es verdadera
else:
    # Bloque de código a ejecutar si la condición es falsa
```

Además, puedes encadenar múltiples condiciones usando la cláusula `elif` (que es una abreviatura de "else if"):

```python
if condicion1:
    # Bloque de código a ejecutar si la condicion1 es verdadera
elif condicion2:
    # Bloque de código a ejecutar si la condicion2 es verdadera
else:
    # Bloque de código a ejecutar si ninguna de las condiciones anteriores es verdadera
```

Por ejemplo, si quieres verificar si un número es positivo, negativo o cero, puedes usar una estructura `if`-`elif`-`else` de la siguiente manera:

```python
numero = 5

if numero > 0:
    print("El número es positivo.")
elif numero < 0:
    print("El número es negativo.")
else:
    print("El número es cero.")
```

En este caso, si el `numero` es mayor que 0, se imprimirá "El número es positivo". Si el `numero` es menor que 0, se imprimirá "El número es negativo". Si el `numero` es igual a 0, se imprimirá "El número es cero".
