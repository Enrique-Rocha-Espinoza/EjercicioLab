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

La siguiente es una descripción de operaciones básicas con listas en Python:

- `lista[i]`: Accede al elemento de la lista que se encuentra en la posición `i`.
- `lista.pop(i)`: Elimina y devuelve el elemento en la posición `i` de la lista. Si no se especifica `i`, remueve el último elemento.
- `lista.append(elemento)`: Añade `elemento` al final de la lista.
- `lista.insert(i, elemento)`: Inserta `elemento` en la posición `i`, desplazando los demás elementos a la derecha.
- `lista.extend(lista2)`: Añade los elementos de `lista2` al final de `lista`.
- `lista.remove(elemento)`: Elimina la primera ocurrencia de `elemento` en la lista.
- `round(number, digits)`: Redondea `number` al número de `digits` decimales especificados. Si `digits` se omite, redondea al entero más cercano.
