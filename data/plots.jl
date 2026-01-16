using CSV, DataFrames, GLMakie

start = 233400;
stop =  241480; #chute opened
#stop =  290000; #touchdown


#max 16 bit signed 32767
# 32767 lsb == 1000 deg/s

lsb2degs = 1000/32767;


lsb2g = 1/2050;
g2ms = 9.81;

sampling_rate = 1000;
data = CSV.read("rocket logger/data/raketa3.csv", DataFrame);

array_data = Matrix(data);

time  = array_data[start:stop,1] / 1000;
ax  = array_data[start:stop,3] * lsb2g * g2ms;
ay  = array_data[start:stop,4] * lsb2g * g2ms;
az  = array_data[start:stop,5] * lsb2g * g2ms;

gx  = array_data[start:stop,6] * lsb2degs;
gy  = array_data[start:stop,7] * lsb2degs;
gz  = array_data[start:stop,8] * lsb2degs;


vx = zeros(size(ax));
vy = zeros(size(ax));
vz = zeros(size(ax));

for i = 2:length(ax)
    vx[i] = vx[i-1] + ax[i-1]*1/sampling_rate 
    vy[i] = vy[i-1] + ay[i-1]*1/sampling_rate
    vz[i] = vz[i-1] + az[i-1]*1/sampling_rate - 1/sampling_rate*g2ms
end

dx = zeros(size(ax));
dy = zeros(size(ax));
dz = zeros(size(ax));

for i = 2:length(ax)
    dx[i] = dx[i-1] + vx[i-1]*1/sampling_rate 
    dy[i] = dy[i-1] + vy[i-1]*1/sampling_rate
    dz[i] = dz[i-1] + vz[i-1]*1/sampling_rate
end


roll = zeros(size(ax));
pitch = zeros(size(ax));
yaw = zeros(size(ax));

for i = 2:length(ax)
    roll[i] = roll[i-1]   + gx[i-1]*1/sampling_rate 
    pitch[i] = pitch[i-1] + gy[i-1]*1/sampling_rate 
    yaw[i] = yaw[i-1]     + gz[i-1]*1/sampling_rate 
end



fig = Figure(size = (1200, 1200))

axis = Axis(fig[1, 1], xlabel = "Time", ylabel = "Acceleration (m/s^2)")
scatter!(axis, time, ax, label = "Acceleration x")
scatter!(axis, time, ay, label = "Acceleration y")
scatter!(axis, time, az, label = "Acceleration z")
axislegend(axis)

axis_gyros = Axis(fig[2, 1], xlabel = "Time", ylabel = "???deg/s")
scatter!(axis_gyros, time, gx, label = "angular rate x")
scatter!(axis_gyros, time, gy, label = "angular rate y")
scatter!(axis_gyros, time, gz, label = "angular rate z")
axislegend(axis_gyros)

axis_vel = Axis(fig[3, 1], xlabel = "Time", ylabel = "Velocity (m/s)")
scatter!(axis_vel, time, vx, label = "Velocity x")
scatter!(axis_vel, time, vy, label = "Velocity y")
scatter!(axis_vel, time, vz, label = "Velocity z")
axislegend(axis_vel)

axis_dist = Axis(fig[4, 1], xlabel = "Time", ylabel = "Distance (m)")
scatter!(axis_dist, time, dx, label = "distance x")
scatter!(axis_dist, time, dy, label = "distance y")
scatter!(axis_dist, time, dz, label = "distance z")
axislegend(axis_dist)


axis_euler = Axis(fig[5, 1], xlabel = "Time", ylabel = "euler angles (deg)")
scatter!(axis_euler, time, roll,   label = "roll [deg]")
scatter!(axis_euler, time, pitch, label = "pitch [deg]")
#scatter!(axis_euler, time, yaw,     label = "yaw [deg]")
axislegend(axis_euler)

save("rocket logger/data/rocket_data_plot.jpg", fig)

fig 
