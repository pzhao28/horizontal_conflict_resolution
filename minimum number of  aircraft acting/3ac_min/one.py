import matplotlib.pyplot as plt

f = plt.figure()
arrow = plt.arrow(0, 0, 0.5, 0.6, 'dummy',label='My label')
plt.legend([arrow,], ['My label',])