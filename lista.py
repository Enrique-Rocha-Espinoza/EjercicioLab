
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
   