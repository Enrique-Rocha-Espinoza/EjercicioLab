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
   