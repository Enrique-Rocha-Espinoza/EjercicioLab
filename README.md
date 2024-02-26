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


## While

En Python, un `while` es una estructura de control que permite ejecutar repetidamente un bloque de código mientras se cumpla una condición específica. La ejecución del bucle continuará hasta que la condición se evalúe como falsa (`False`).

La sintaxis básica de un bucle `while` en Python es la siguiente:

```python
while condicion:
    # Bloque de código a ejecutar mientras la condición sea verdadera
```

Aquí, `condicion` es una expresión que se evalúa en cada iteración del bucle. Si la `condicion` es verdadera (`True`), el `# Bloque de código a ejecutar mientras la condición sea verdadera` se ejecutará. Luego, la condición se vuelve a evaluar, y si sigue siendo verdadera, el bloque de código se ejecutará nuevamente. Este proceso se repite hasta que la condición se evalúe como falsa.

Es importante tener cuidado al usar bucles `while`, ya que si la condición siempre se evalúa como verdadera, el bucle se ejecutará infinitamente, lo que puede hacer que el programa se bloquee o se comporte de manera inesperada.

Aquí hay un ejemplo simple de un bucle `while` que imprime los números del 1 al 5:

```python
contador = 1

while contador <= 5:
    print(contador)
    contador += 1
```

En este ejemplo, la variable `contador` se inicializa en 1. Mientras `contador` sea menor o igual a 5, el bucle `while` imprimirá el valor de `contador` y luego incrementará `contador` en 1. Cuando `contador` sea mayor que 5, la condición `contador <= 5` se evaluará como falsa, y el bucle terminará.


## problema 1
Escribir un programa que lea un entero positivo “n” introducido por el usuario y después muestreen pantalla la suma de todos los enteros desde 1 hasta n . La suma de los primeros enterospositivos puede ser calculada de la siguiente forma: 

suma = (n(n + 1))/2

El código para resolver este problema es el siguiente:
```python
n= int(input("inserte el numero"))


a= (n(n+1))/2
print(f"\n la suma es: {a}")
```
Este consiste en que en la primera línea le pedimos al usuario ingresar un número para que se haga la suma, este al ser recibido por un input, se debe de transformar a un entero (int) y este valor se le asigna a n, después, colocamos la fórmula dada en el problema y el resultado de esta se asocia a una variable a, por último, solo se imprime esta variable a para mostrar el resultado.


## problema 2

Escribir un programa que pregunte al usuario por el número de horas trabajadas y el costo por hora, Después debe mostrar por pantalla la paga que le corresponde.

El código para resolver este problema es el siguiente:

```python
a, b= map(int,input("inserte las horas y el costo: ").split())

print(f"pago : {a*b}")
```
En la primera línea le pedimos al usuario que ingrese las horas trabajadas y el costo por hora, después, mapeamos estos dos valores enteros a las variables a y b, por último, para mostrar la paga que le corresponde, imprimimos la multiplicación de a * b para obtener el valor deseado.

## problema 3

Crea una lista de nombre + sueldo por hora + horas trabajadas de al menos seis operadores.Imprime el nombre y el sueldo a pagar de cada operador.

El código para resolver este problema es el siguiente:


```python
def captura():
    a=str(input("inserte el nombre del trabajador: "))
    c=int(input(f"cuanto gana por hora {a}: "))
    b=int(input(f"cuantas horas trabajo {a}: "))

    

    return a,b,c


if __name__ == "__main__":
    trabajadores=int(input("Cuantos trabajadores son: "))
    i= 0
    lista= []
    while(i<trabajadores):
        
        a,b,c= captura()
        c = c*b

        lista.append(a)
        lista.append(b)
        lista.append(c)
        i+=1


    print("\nNombre del trabajador\tSueldo")
    print("-" * 80)
    for i in range(0, len(lista), 3):
        print(f"{lista[i]:20}\t{lista[i + 2]:<10}")
```

En la función `capturar()` se capturan los datos que se van a utilizar para poder trabajar con ellos, en primer lugar, se pide el nombre del trabajador, después cuánto gana este por hora y, por último, cuántas horas trabajó. Estos valores se asignan a las variables a, b y c.

En la función main, primero le preguntamos al usuario cuántos trabajadores va a ingresar, después, creamos nuestra lista vacía, la cual vamos a ir rellenando con los valores `a  b  c`, en el while, se colocó como condición que se repitiera las veces que fueran necesarias hasta que alcance el número de trabajadores ingresados, dentro de este, se llama a la función capturar() y después de obtener los valores, modificaremos el valor de c para que este sea ahora `c * b`, para que obtengamos cuánto se le debe pagar a cada trabajador.

Estos valores de a, b, c se irán metiendo a la lista, para que después tengamos una lista con todos los datos recopilados por el while, por último, imprimimos los valores de esa lista, se usó un for para iterar cada 3 espacios, ya que después de 3 elementos de la lista empieza un nuevo trabajador, solo queremos imprimir el nombre y el valor de c que es la paga que se le debe dar al trabajador, por lo que se imprime la posición `i` de la lista y la posición `i + 2`, la cual contiene c.

