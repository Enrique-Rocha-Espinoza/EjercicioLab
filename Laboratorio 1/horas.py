# se piden al usuario los datos de las horas trabajadas y cuanto se le paga por hora
a, b= map(int,input("inserte las horas y el costo: ").split())
# se imprime el pago multiplicando las horas por el pago por hora
print(f"pago : {a*b}")
