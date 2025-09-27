import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_smoothing_spline

fn = 'clean_battery_calibration.tsv'

data = []
for i, line in enumerate(open(fn)):
	try:
		numbers = list(map(float, line.split()))
		if len(numbers) == 3:
			data.append(numbers)
	except:
		# print('fail', i)
		pass

data = np.array(data)

plt.plot(data[:,0], data[:,2]); plt.grid(True); plt.title('time vs voltage'); plt.show()
# plt.plot(data[:,0], data[:,1]); plt.grid(True); plt.title('time vs adc count'); plt.show() #time vs voltage



# plt.plot(data[:,1], data[:,0] / data[-1,0] * 100); plt.grid(True); plt.show()

x = data[:,1]
y = 100 - data[:,0] / data[-1,0] * 100

xi = np.argsort(-x)
# print(xi)
x = x[xi]
y = y[xi]

# plt.plot(x,y); plt.show()

min_count = 1082
max_count = 1560
scale = 2

# percentage_maps = [(min_count, 1), ]
percentage_maps = []

for count in range(min_count+1, max_count, scale):
	d = np.abs(count - x)
	w = np.exp(-d*d / 10)
	smoothed_value = int(np.sum(w * y) / np.sum(w))
	percentage_maps.append((count, smoothed_value))

# percentage_maps.append((max_count, 100))
percentage_maps = np.array(percentage_maps)

if 1:
	plt.plot(data[:,1], 100 - data[:,0] / data[-1,0] * 100)
	plt.scatter(percentage_maps[:,0], percentage_maps[:,1], color='r')
	plt.plot(percentage_maps[:,0], percentage_maps[:,1], color='r')
	plt.grid(True)
	plt.show()

code_template = """

int get_battery_percentage(uint32_t vbatt) {{
	uint32_t scale = {scale};
	uint32_t min_count = 1082 / scale;
	uint32_t max_count = 1560 / scale;

	uint32_t lookup_table[{lookup_count}] = {{ {lookup_table} }};

	uint32_t lookup_value = vbatt / scale;
	if(lookup_value <= min_count) {{
		return 1;
	}} else if (lookup_value >= max_count) {{
		return 100;
	}} else {{
		return lookup_table[lookup_value - min_count];
	}}

}}
""".format(scale = scale, lookup_count = len(percentage_maps), 
	lookup_table = ",".join(str(pm[1]) for pm in percentage_maps))

print(code_template)