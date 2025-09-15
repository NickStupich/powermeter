import pandas as pd
from bs4 import BeautifulSoup	
import matplotlib.pyplot as plt
import time
import dateutil
import numpy as np


content = open("ride2/Afternoon_Ride.gpx", 'r').read()

# content = open("C:/Users/nicks/Downloads/Afternoon_Ride (1).gpx", 'r').read(); start_time = (30*60); end_time = (91*60) #2025 seymour hill climb easy-ish
 
soup = BeautifulSoup(content, "xml")
tracking = soup.find("trkseg")
sport = soup.find("type").text
print('sport: ', sport)
points = tracking.find_all("trkpt")

times = []
powers = []
hrs = []

for i, point in enumerate(points):
	elevation = float(point.ele.text)
	power = float(point.extensions.power.text)

	hr_pt = point.find("gpxtpx:hr")
	if hr_pt:
		hr = float(hr_pt.text)
	else:
		hr = -1
	
	if point.time:
		t = time.mktime(dateutil.parser.isoparse(point.time.text).timetuple())
	else:
		t = times[-1] + 1 if len(times) else 0
	
	hrs.append(hr)
	times.append(t)
	powers.append(power)


hrs = np.array(hrs)
times = np.array(times)
powers = np.array(powers)
print(times.shape, powers.shape)

times -= times[0]


df_raw = pd.read_csv('ride2/raw.txt', index_col=False)
df_smooth = pd.read_csv('ride2/smooth.txt', index_col=False)

post_smoothed_times = []
post_smoothed_powers = []
post_smoothed_powers_windowed = []


N = 83*2
window = np.hamming(N)
window /= np.mean(window)
window_sum1 = window / N
# print(','.join(map(str, window_sum1)))
for i in range(40, len(df_raw['Power'])-N, N):
	t = np.array(df_raw['Time'])[i+N]
	p = np.mean(np.array(df_raw['Power'])[i:i+N])

	p_window = np.mean(np.array(df_raw['Power'])[i:i+N] * window)
	
	post_smoothed_times.append(t)
	post_smoothed_powers.append(p)
	post_smoothed_powers_windowed.append(p_window)

post_smoothed_times = np.array(post_smoothed_times)
post_smoothed_powers = np.array(post_smoothed_powers)
post_smoothed_powers_windowed = np.array(post_smoothed_powers_windowed)

if 0:
	plt.subplot(2,2,1)
	plt.hist(np.diff(-df_smooth['Power']), bins=100); plt.grid(True)
	plt.title(np.mean(np.abs(np.diff(df_smooth['Power']))))

	plt.subplot(2,2,2)
	plt.hist(np.diff(post_smoothed_powers), bins=100); plt.grid(True)
	plt.title(np.mean(np.abs(np.diff(post_smoothed_powers))))

	plt.subplot(2,2,3)
	plt.hist(np.diff(post_smoothed_powers_windowed), bins=100); plt.grid(True)
	plt.title('windowed' + str(np.mean(np.abs((np.diff(post_smoothed_powers_windowed))))))

	plt.show()


# t1 = 840
# t2 = 1080


t1 = 1290
t2 = 1500

n1_raw = np.where(df_raw['Time']/1000 > t1)[0][0]
n2_raw = np.where(df_raw['Time']/1000 > t2)[0][0]

interval_power_raw = np.mean(df_raw['Power'][n1_raw:n2_raw])
print('interval_power_raw: ', interval_power_raw)


n1_smooth = np.where(df_smooth['Time']/1000 > t1)[0][0]
n2_smooth = np.where(df_smooth['Time']/1000 > t2)[0][0]

interval_power_smooth = np.mean(df_smooth['Power'][n1_smooth:n2_smooth])
print('interval_power_smooth: ', interval_power_smooth)


if 1:
	plt.subplot(3,1,1)
	plt.plot(df_raw['Time']/1000, df_raw['Power'], label='raw')
	plt.grid(True)

	plt.plot(df_smooth['Time']/1000, df_smooth['Power'], label='smoothed')
	plt.plot(post_smoothed_times/1000, post_smoothed_powers, label='Post-smoothed')
	plt.plot(post_smoothed_times/1000, post_smoothed_powers_windowed, label='Windowed')


	plt.plot(times+89, powers, label='garmin')

	plt.legend()

	plt.subplot(3,1,2)
	plt.plot(df_raw['Time']/1000, df_raw['Force'], label='Force (N)')
	plt.grid(True)
	plt.legend()

	plt.subplot(3,1,3)
	plt.plot(df_raw['Time']/1000, df_raw['Gyro'], label='Gyro (rad/s)')
	plt.grid(True)
	plt.legend()
	plt.show()