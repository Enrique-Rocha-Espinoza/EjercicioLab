
def captura():
    a=str(input("inserte el nombre del trabajador: "))
    b=int(input(f"cuanto gana por hora {a}"))
    c=int(input(f"cuantas horas trabajo {a}"))

    return a,b,c


if __name__ == "__main__":
    trabajadores=int(input("ucnatos trabajadores son:"))
    i= 0
    lista= []
    while(i<trabajadores):
        
        a,b,c= captura()
        print(f"{a} {b} {c}")
        lista.append(a)
        lista.append(b)
        lista.append(c)
        i+=1
        

        hkjkjii
    
    for x in range(0, len(lista), 2):
        print(x)
