import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pandas as pd

# file = pd.read_csv('./data/data_2.csv')
# file_debug = pd.read_csv('./data/data_2_debug.csv')
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

# plt.plot(file['prior_x'], file['prior_y'], label='Prior')
# plt.plot(file['observe_x'], file['observe_y'], label='observe')
# plt.plot(file['post_x'], file['post_y'], label='Post')
# plt.legend()
# plt.show()

# plt.plot(file_debug['path0_x'], file_debug['path0_y'], label='path0')
# plt.plot(file_debug['path1_x'], file_debug['path1_y'], label='path1')
# plt.legend()
# plt.show()


#test3
file_3 = pd.read_csv('./data/data_3.csv')

plt.plot(file_3['other0_x'], file_3['other0_y'], label='other0')
plt.plot(file_3['other1_x'], file_3['other1_y'], label='other1')
plt.plot(file_3['other2_x'], file_3['other2_y'], label='other2')
plt.plot(file_3['other3_x'], file_3['other3_y'], label='other3')
plt.plot(file_3['self_x'], file_3['self_y'], label='self')
plt.legend()
plt.show()