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

