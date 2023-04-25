clear

% Constants
karman_line = 100000;
GM = 3.986004418e14;
Re = 6371000;

% Altitude things 
altitude_above = [0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 15000, 20000, 25000, 30000, 40000, 50000, 60000, 70000, 80000]; 
density = [1.225, 1.112, 1.007, 0.9093, .8194, .7364, .6601, .5900, .5258, .4671, .4135, .1948, .08891, .04008, .01841, .003996, .001027, .0003097, .00008283, .00001846]; 
rho_altitude = polyfit(altitude_above, density, 5);


% Rocket parameters
initial_mass = 20; % Initial mass of the rocket, including payload (kg)
propellant_mass = 60; % Mass of the rocket propellant (kg)
thrust = 3550; % Rocket engine thrust (N)
Cd = .75; % Rocket drag coefficient
diameter = 8; % Inches
A = (3.14*(diameter/2*0.0254)^2); % Rocket's cross-sectional area (m^2)
m_dot = 1.3; % Mass flow rate (kg/s)

burn_time = propellant_mass/m_dot; % Engine burn time (s)

% Time step for simulation
dt = 0.1;

% Initialize variables and arrays for plotting
t = 0;
v = 0;
altitude = 0;
max_altitude = 0;

time = [];
velocity = [];
altitude_data = [];
drag_force_data = [];
mass_data = [];
acceleration_data = [];
thrust_data = [];
gravity_data = [];
rho_data = []; 

% Simulation
while v >= 0
    mass = rocket_mass(t, initial_mass, m_dot, burn_time, propellant_mass);
    g = gravity(altitude, GM, Re);
    current_thrust = find_thrust(t, burn_time, thrust); 
    rho = polyval(rho_altitude, altitude); 
    if (altitude > 80000)
        rho = 0; 
    end
    drag_force = rocket_drag(v, Cd, A, rho);
    acceleration = (current_thrust - mass * g - drag_force) / mass;
    v = v + acceleration * dt;
    altitude = altitude + v * dt;
    
    max_altitude = max(max_altitude, altitude);
    
    time = [time, t];
    velocity = [velocity, v];
    altitude_data = [altitude_data, altitude];
    drag_force_data = [drag_force_data, drag_force];
    mass_data = [mass_data, mass];
    acceleration_data = [acceleration_data, acceleration];
    thrust_data = [thrust_data, current_thrust];
    gravity_data = [gravity_data, g]; 
    rho_data = [rho_data, rho];
    
    t = t + dt;
end

gs = acceleration_data / 9.81; 

% Check if the rocket reached the Karman line
if max_altitude >= karman_line
    disp('Rocket reached the Karman line');
else
    disp('Rocket failed to reach the Karman line');
end

% Plot velocity, drag force, and altitude over time
figure;
subplot(4, 1, 1);
plot(time, velocity);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity vs. Time');

subplot(4, 1, 2);
plot(time, drag_force_data);
xlabel('Time (s)');
ylabel('Drag Force (N)');
title('Drag Force vs. Time');

subplot(4, 1, 3);
plot(time, altitude_data);
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Altitude vs. Time');

subplot(4, 1, 4);
plot(time, gs);
xlabel('Time (s)');
ylabel('Gs');
title('Gs vs. Time');

% Rocket mass function
function mass = rocket_mass(t, initial_mass, m_dot, burn_time, prop_mass)
    if t <= burn_time
        mass = initial_mass + prop_mass - m_dot * t;
    else
        mass = initial_mass;
    end
end

function thrust = find_thrust(t, burn_time, rthrust)
    if t <= burn_time
        thrust = rthrust;
    else 
        thrust = 0; 
    end
end

function g = gravity(altitude, GM, Re)
    g = GM / (Re + altitude)^2;
end

function drag_force = rocket_drag(v, Cd, A, rho)
    drag_force = 0.5 * rho * abs(v)^2 * Cd * A * sign(v);
end

function rho = air_density_calc(current_altitude, altitude, density)
    rho = polyfit(altitude, density, current_altitude); 
end




