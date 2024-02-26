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
# se le piden al usuario que inserte el numero para hacer la sumatoria
n= int(input("inserte el numero"))

# se declara la ecuacion
a= (n(n+1))/2
# se imprime el resultado
print(f"\n la suma es: {a}")
```
Este consiste en que en la primera línea le pedimos al usuario ingresar un número para que se haga la suma, este al ser recibido por un input, se debe de transformar a un entero (int) y este valor se le asigna a n, después, colocamos la fórmula dada en el problema y el resultado de esta se asocia a una variable a, por último, solo se imprime esta variable a para mostrar el resultado.


## problema 2

Escribir un programa que pregunte al usuario por el número de horas trabajadas y el costo por hora, Después debe mostrar por pantalla la paga que le corresponde.

El código para resolver este problema es el siguiente:

```python
# se piden al usuario los datos de las horas trabajadas y cuanto se le paga por hora
a, b= map(int,input("inserte las horas y el costo: ").split())
# se imprime el pago multiplicando las horas por el pago por hora
print(f"pago : {a*b}")
```
En la primera línea le pedimos al usuario que ingrese las horas trabajadas y el costo por hora, después, mapeamos estos dos valores enteros a las variables a y b, por último, para mostrar la paga que le corresponde, imprimimos la multiplicación de a * b para obtener el valor deseado.

## problema 3

Crea una lista de nombre + sueldo por hora + horas trabajadas de al menos seis operadores.Imprime el nombre y el sueldo a pagar de cada operador.

El código para resolver este problema es el siguiente:


```python
# se piden los datos del empleado, nombre, ganacia por hora, horas trabajadas
def captura():
    a=str(input("inserte el nombre del trabajador: "))
    c=int(input(f"cuanto gana por hora {a}: "))
    b=int(input(f"cuantas horas trabajo {a}: "))

    

    return a,b,c


if __name__ == "__main__":
    # se le pide al usuario que ingrese cuantos trabajadores son
    trabajadores=int(input("Cuantos trabajadores son: "))
    i= 0
    lista= []
    # se crea un ciclo while para llamar la funcion capturar las veces que sean nesesarias
    while(i<trabajadores):
        
        a,b,c= captura()
        # se hace el calculo del pago
        c = c*b
        # se colocan los valores dentro de la lista
        lista.append(a)
        lista.append(b)
        lista.append(c)
        i+=1

    
    print("\nNombre del trabajador\tSueldo")
    print("-" * 80)
    # se imprime la lista cada 3 elemntos
    for i in range(0, len(lista), 3):
        print(f"{lista[i]:20}\t{lista[i + 2]:<10}")
```

En la función `capturar()` se capturan los datos que se van a utilizar para poder trabajar con ellos, en primer lugar, se pide el nombre del trabajador, después cuánto gana este por hora y, por último, cuántas horas trabajó. Estos valores se asignan a las variables a, b y c.

En la función main, primero le preguntamos al usuario cuántos trabajadores va a ingresar, después, creamos nuestra lista vacía, la cual vamos a ir rellenando con los valores `a  b  c`, en el while, se colocó como condición que se repitiera las veces que fueran necesarias hasta que alcance el número de trabajadores ingresados, dentro de este, se llama a la función capturar() y después de obtener los valores, modificaremos el valor de c para que este sea ahora `c * b`, para que obtengamos cuánto se le debe pagar a cada trabajador.

Estos valores de a, b, c se irán metiendo a la lista, para que después tengamos una lista con todos los datos recopilados por el while, por último, imprimimos los valores de esa lista, se usó un for para iterar cada 3 espacios, ya que después de 3 elementos de la lista empieza un nuevo trabajador, solo queremos imprimir el nombre y el valor de c que es la paga que se le debe dar al trabajador, por lo que se imprime la posición `i` de la lista y la posición `i + 2`, la cual contiene c.

## Problema 4

Crea una lista llamada numeros que contenga al menos 10 números, Calcula el promedio de los números pares y el producto de los números impares, Imprime los resultados.

El código para resolver este problema es el siguiente:

```python
# Lista de números enteros
Lista = [2, 4, 5, 8, 4, 3, 1, 19, 23, 29]

# Crear una lista de números pares y calcular su promedio
numeros_pares = [num for num in Lista if num % 2 == 0]  # Lista de números pares
promedio_pares = sum(numeros_pares) / len(numeros_pares) if numeros_pares else 0  # Promedio de números pares

# Crear una lista de números impares y calcular su producto
numeros_impares = [num for num in Lista if num % 2 != 0]  # Lista de números impares
producto_impares = 1  # Inicializar el producto de números impares
for num in numeros_impares:  # Calcular el producto de los números impares
    producto_impares *= num

# Imprimir los resultados
print(f"Promedio de los números pares: {promedio_pares}")
print(f"Producto de los números impares: {producto_impares}")

```


- **Inicialización de la Lista:** Se crea una lista de números enteros llamada `Lista`.

- **Cálculo del Promedio de Números Pares:**
  - Se utiliza una comprensión de lista para filtrar los números pares de `Lista` y almacenarlos en `numeros_pares`, esta es que dividimos el nuemro entre 2 y si su residuo es igual a 0 significa que es un numero par.
  - Se calcula el promedio de los números pares sumándolos y dividiendo por la cantidad de números pares. Si la lista de números pares está vacía, el promedio se establece en 0 para evitar la división por cero.

- **Cálculo del Producto de Números Impares:**
  - Se utiliza otra comprensión de lista para filtrar los números impares de `Lista` y almacenarlos en `numeros_impares`, esto se logró dividiendo entre 2 y viendo que el residuo fuera diferente de 0.
  - Se inicializa una variable `producto_impares` en 1 y luego se multiplica por cada número impar en `numeros_impares` para obtener el producto total de los números impares.

- **Impresión de Resultados:** Se imprimen en consola el promedio de los números pares y el producto de los números impares.

## Problema 5

Crea un programa que solicite al usuario adivinar un número secreto. El programa debe generarun número aleatorio entre 1 y 10, y el usuario debe intentar adivinarlo. El programa debeproporcionar pistas si el número ingresado por el usuario es demasiado alto o bajo. El bucle whiledebe continuar hasta que el usuario adivine correctamente. Al final se debe imprimir en cuantosintentos el usuario logró adivinar el número.

El código para resolver este problema es el siguiente:

```python
import random

# Generar un número aleatorio entre 1 y 10 y almacenarlo en la variable 'numero_secreto'
numero_secreto = random.randint(1, 10)

# Inicializar la variable 'intentos' para llevar la cuenta de los intentos del usuario
intentos = 0

# Iniciar un bucle infinito para permitir al usuario adivinar el número secreto
while True:
    # Solicitar al usuario que ingrese un número entre 1 y 10 y convertirlo a entero
    numero_usuario = int(input("Adivina el número secreto entre 1 y 10: "))

    # Incrementar el contador de intentos en cada iteración del bucle
    intentos += 1

    # Verificar si el número ingresado por el usuario es igual al número secreto
    if numero_usuario == numero_secreto:
        # Si el usuario adivina el número, imprimir un mensaje de felicitación y salir del bucle
        print(f"¡Felicidades! Adivinaste el número secreto en {intentos} intentos.")
        break
    # Verificar si el número ingresado por el usuario es menor que el número secreto
    elif numero_usuario < numero_secreto:
        # Si el número es demasiado bajo, informar al usuario e invitarlo a intentar de nuevo
        print("El número es demasiado bajo. Intenta de nuevo.")
    # Si el número ingresado no es menor ni igual al número secreto, debe ser mayor
    else:
        # Informar al usuario que el número es demasiado alto e invitarlo a intentar de nuevo
        print("El número es demasiado alto. Intenta de nuevo.")

```
1. **Generar un número aleatorio entre 1 y 10:** El código utiliza la función `randint` del módulo `random` para generar un número entero aleatorio entre 1 y 10. Este número se almacena en la variable `numero_secreto`.

2. **Inicializar la variable de intentos:** La variable `intentos` se inicializa en 0 para llevar la cuenta de cuántos intentos ha hecho el usuario para adivinar el número.

3. **Bucle infinito:** El código entra en un bucle `while` infinito (`while True:`) para permitir al usuario seguir adivinando hasta que acierte el número.

4. **Solicitar al usuario que adivine el número:** Dentro del bucle, el código pide al usuario que ingrese un número entre 1 y 10. El número ingresado se convierte a entero y se almacena en la variable `numero_usuario`.

5. **Incrementar la variable de intentos:** Cada vez que el usuario hace un intento, la variable `intentos` se incrementa en 1.

6. **Verificar la adivinanza del usuario:**
   - Si el `numero_usuario` es igual al `numero_secreto`, el usuario ha adivinado correctamente. El código imprime un mensaje de felicitación junto con el número de intentos que tomó, y luego sale del bucle con `break`.
   - Si el `numero_usuario` es menor que el `numero_secreto`, el código informa al usuario que el número es demasiado bajo y le pide que intente de nuevo.
   - Si el `numero_usuario` es mayor que el `numero_secreto`, el código informa al usuario que el número es demasiado alto y le pide que intente de nuevo.


## Problema 6
Robot exploradorEl programa debe generar una matriz de al menos 5x5.El robot inicia su camino en la posición (0,0) de la matriz y debe salir en la posición (4,4) o lamáxima posición si se cambia el tamaño de matriz.El numero y la posición de los obstáculos es aleatoria.El robot solo puede avanzar, girar a la izquierda o a la derecha para buscar un camino libre, en eleventual caso que el robot no pueda salir debe imprimir en pantalla “Imposible llegar al destino”En caso de que el robot llegue a su destino final deberá imprimir el mapa, con los espacios libres yobstáculos de la siguiente forma X obstáculo o libreo 

o o X o o

o o o o o

o o o o X

o o o o o

o X X X o

Deberá imprimir también la ruta que siguió.

Mostrar un segundo mapa con el “camino” seguido por el robot mediante flechas

