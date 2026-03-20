using CSV, DataFrames, GLMakie

start = 233400;
stop =  241480; #chute opened 


start = 40000
stop =  50480


#max 16 bit signed 32767
# 32767 lsb == 1000 deg/s

lsb2degs = 1000/32767;


lsb2g = 1/2048;
g2ms = 9.81;

sampling_rate = 1000;
data = CSV.read("rocket logger/data/motor_D9-7_start_1.csv", DataFrame);

array_data = Matrix(data);

time  = array_data[start:stop,1] / 1000;
ax  = array_data[start:stop,3] * lsb2g * g2ms;
ay  = array_data[start:stop,4] * lsb2g * g2ms;
az  = array_data[start:stop,5] * lsb2g * g2ms .-g2ms;

gx  = array_data[start:stop,6] * lsb2degs;
gy  = array_data[start:stop,7] * lsb2degs;
gz  = array_data[start:stop,8] * lsb2degs;


vx = zeros(size(ax));
vy = zeros(size(ax));
vz = zeros(size(ax));

for i = 2:length(ax)
    vx[i] = vx[i-1] + ax[i-1]*1/sampling_rate 
    vy[i] = vy[i-1] + ay[i-1]*1/sampling_rate
    vz[i] = vz[i-1] + az[i-1]*1/sampling_rate # - 1/sampling_rate*g2ms
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

axis = Axis(fig[1, 1], xlabel = "Time", ylabel = "Acceleration in world frame (m/s^2)")
lines!(axis, time, ax, label = "Acceleration x", linewidth = 2, inspector_label = (self, i, pos) -> "$(round(pos[1], digits=2))s, $(round(pos[2], digits=2)) m/s²")
lines!(axis, time, ay, label = "Acceleration y", linewidth = 2, inspector_label = (self, i, pos) -> "$(round(pos[1], digits=2))s, $(round(pos[2], digits=2)) m/s²")
lines!(axis, time, az, label = "Acceleration z", linewidth = 2, inspector_label = (self, i, pos) -> "$(round(pos[1], digits=2))s, $(round(pos[2], digits=2)) m/s²")
axislegend(axis)


axis_vel = Axis(fig[2, 1], xlabel = "Time", ylabel = "Velocity (m/s)")
lines!(axis_vel, time, vx, label = "Velocity x", linewidth = 2, inspector_label = (self, i, pos) -> "$(round(pos[1], digits=2))s, $(round(pos[2], digits=2)) m/s")
lines!(axis_vel, time, vy, label = "Velocity y", linewidth = 2, inspector_label = (self, i, pos) -> "$(round(pos[1], digits=2))s, $(round(pos[2], digits=2)) m/s")
lines!(axis_vel, time, vz, label = "Velocity z", linewidth = 2, inspector_label = (self, i, pos) -> "$(round(pos[1], digits=2))s, $(round(pos[2], digits=2)) m/s")
axislegend(axis_vel)

axis_dist = Axis(fig[3, 1], xlabel = "Time", ylabel = "Distance (m)")
lines!(axis_dist, time, dx, label = "distance x", linewidth = 2, inspector_label = (self, i, pos) -> "$(round(pos[1], digits=2))s, $(round(pos[2], digits=2)) m")
lines!(axis_dist, time, dy, label = "distance y", linewidth = 2, inspector_label = (self, i, pos) -> "$(round(pos[1], digits=2))s, $(round(pos[2], digits=2)) m")
lines!(axis_dist, time, dz, label = "distance z", linewidth = 2, inspector_label = (self, i, pos) -> "$(round(pos[1], digits=2))s, $(round(pos[2], digits=2)) m")
axislegend(axis_dist)

# Link x-axes so zooming one plot zooms all plots to the same time
linkxaxes!(axis, axis_vel, axis_dist)

# Single DataInspector for the entire figure
DataInspector(fig)


#axis_gyros = Axis(fig[4, 1], xlabel = "Time", ylabel = "???deg/s")
#lines!(axis_gyros, time, gx, label = "angular rate x")
#lines!(axis_gyros, time, gy, label = "angular rate y")
#lines!(axis_gyros, time, gz, label = "angular rate z")
#axislegend(axis_gyros)
#
#
#
#axis_euler = Axis(fig[5, 1], xlabel = "Time", ylabel = "euler angles (deg)")
#lines!(axis_euler, time, roll,   label = "roll [deg]")
#lines!(axis_euler, time, pitch, label = "pitch [deg]")
##lines!(axis_euler, time, yaw,     label = "yaw [deg]")
#axislegend(axis_euler)

save("rocket logger/data/rocket_data_plot2.jpg", fig)

fig 
