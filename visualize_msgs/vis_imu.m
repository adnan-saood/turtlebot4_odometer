imu = getIMU(bagFile);

%%

t_ax = linspace(imu.ax.TimeInfo.Start, imu.ax.TimeInfo.End, imu.ax.TimeInfo.Length);

ax_ = resample(imu.ax, t_ax);
plot(ax_)