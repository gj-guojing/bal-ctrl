import matplotlib.pyplot as plt

## reading txt document
file_path = "D:\\Code\\TriplePendulum\\bal_ctrl\\out\\build\\x64-Debug\\demo\\demo_triplependulum\\AngularMomentandCoMstates.txt"

data = []

with open(file_path, "r") as file:
    for line in file:
        data.append([float(x) for x in line.strip().split()])



columns = list(zip(*data))

max_values = [max(column) for column in columns]
min_values = [min(column) for column in columns]

max_indexes = [column.index(max_value) for column, max_value in zip(columns, max_values)]
min_indexes = [column.index(min_value) for column, min_value in zip(columns, min_values)]


labels = ['AngularMoment', 'CoM_position', 'CoM_velocity', 'CoM_acceleration']

for column, label in zip(columns, labels):
    plt.plot(column, label = label)

# for max_value, max_index, min_value, min_index, label in zip(max_values, max_indexes, min_values, min_indexes, labels):

#     plt.annotate(f'Max:{max_value}', xy=(max_index, max_value), xytext=(max_index, max_value))
#     plt.annotate(f'Min:{min_value}', xy=(min_index, min_value), xytext=(min_index, min_value))

plt.legend()

# 创建一个表格
table_data = [['Label', 'Max', 'Min']]
for label, max_value, min_value in zip(labels, max_values, min_values):
    table_data.append([label, max_value, min_value])

bbox = [0, -0.5, 1, 0.4]

plt.table(cellText=table_data, loc='bottom', bbox=bbox)
plt.subplots_adjust(bottom=0.31)


plt.show()


