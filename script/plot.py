import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pandas as pd

file = pd.read_csv('./data/data_2.csv')
def update(frame):
    plt.cla()  # Clear the current plot
    plt.plot(file['prior_x'][:frame], file['prior_y'][:frame], label='Prior')
    plt.plot(file['observe_x'][:frame], file['observe_y'][:frame], label='observe')
    # plt.plot(file['post_x'][:frame], file['post_y'][:frame], label='Post')
    plt.legend()
    
# animation = FuncAnimation(plt.gcf(), update, frames=len(file), interval=100)
# plt.show()

# plt.plot(file['prior_x'], file['prior_y'])
# plt.plot(file['post_x'], file['post_y'])
plt.plot(file['prior_x'], file['prior_y'], label='Prior')
plt.plot(file['observe_x'], file['observe_y'], label='observe')
plt.plot(file['post_x'], file['post_y'], label='Post')
plt.legend()
plt.show()
