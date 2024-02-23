Lista = [2, 4, 5, 8, 4, 3, 1, 19, 23, 29]

numeros_pares = [num for num in Lista if num % 2 == 0]
promedio_pares = sum(numeros_pares) / len(numeros_pares) if numeros_pares else 0

numeros_impares = [num for num in Lista if num % 2 != 0]
producto_impares = 1
for num in numeros_impares:
    producto_impares *= num

# Imprimiendo los resultados
print(f"Promedio de los números pares: {promedio_pares}")
print(f"Producto de los números impares: {producto_impares}")