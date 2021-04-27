from matplotlib import pyplot as plt

def plot_rota(rota):
    x = []
    y = []
    for cordenada in rota:
        x.append(cordenada[0])
        y.append(cordenada[1])
    
    # plot obstaculo
    l1 = [-2.5, -1.5, 0, 1.5, 2.5]
    l2 = [4.5, 5.5]
    a1 = [ 4.5 for i in l1]
    a2 = [ 5.5 for i in l1]
    a3 = [ -2.5 for i in l2]
    a4 = [ 2.5 for i in l2]
    plt.plot(a1, l1, 'b')
    plt.plot(a2, l1, 'b')
    plt.plot(l2, a3, 'b')
    plt.plot(l2, a4, 'b')

    # plot rota
    plt.plot(x, y)
    plt.savefig('atrator_repulsor/src/rota')