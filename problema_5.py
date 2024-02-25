import random

# Generar un número aleatorio entre 1 y 10
numero_secreto = random.randint(1, 10)
intentos = 0

while True:
    # Solicitar al usuario que adivine el número
    numero_usuario = int(input("Adivina el número secreto entre 1 y 10: "))
    intentos += 1

    # Verificar si el usuario adivinó correctamente
    if numero_usuario == numero_secreto:
        print(f"¡Felicidades! Adivinaste el número secreto en {intentos} intentos.")
        break
    elif numero_usuario < numero_secreto:
        print("El número es demasiado bajo. Intenta de nuevo.")
    else:
        print("El número es demasiado alto. Intenta de nuevo.")
