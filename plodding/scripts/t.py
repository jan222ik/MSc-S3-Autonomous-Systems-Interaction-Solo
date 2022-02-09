#! /usr/bin/env python

import random, numpy, math, copy, matplotlib.pyplot as plt

def cost(c0, c1):
    return math.sqrt(sum([(c0[d] - c1[d]) ** 2 for d in [0, 1]]))

def tsp():
    cities = [[0,0], [1,7], [1,0], [1,3], [2,0], [2,3], [5,5]]
    N = len(cities)
    tour = random.sample(range(N),N)
    for temperature in numpy.logspace(0,5,num=100000)[::-1]:
        [i,j] = sorted(random.sample(range(N),2))
        newTour =  tour[:i] + tour[j:j+1] +  tour[i+1:j] + tour[i:i+1] + tour[j+1:]
        oldDistances = sum([
            cost(cities[tour[(k + 1) % N]], cities[tour[k % N]]) for k in[j, j - 1, i, i - 1]]
        )
        newDistances = sum([
            cost(cities[newTour[(k + 1) % N]], cities[newTour[k % N]]) for k in[j, j - 1, i, i - 1]]
        )
        if math.exp((oldDistances - newDistances) / temperature) > random.random():
             tour = copy.copy(newTour)
    plt.plot([cities[tour[i % N]][0] for i in range(N + 1)], [cities[tour[i % N]][1] for i in range(N + 1)], 'xb-')
    print(map(lambda x: "{} {}".format(x, cities[x]), tour))
    plt.show()

tsp()
