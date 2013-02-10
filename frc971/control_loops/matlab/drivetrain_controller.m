close all;
load 'drivetrain_spin_low'
load 'drivetrain_strait_low'
m = 68;
rb = 0.617998644 / 2.0;
J = 7;
stall_current = 133.0;
R = 12.0 / stall_current / 4 / 0.43;
Km = (12.0 - R * 2.7) / (4650.0 / 60.0 * 2.0 * pi);
Kt = 0.008;
r = 0.04445; % 3.5 inches diameter
G_low = 60.0 / 15.0 * 50.0 / 15.0;
G_high = 45.0 / 30.0 * 50.0 / 15.0;
dt = 0.01;

G = G_low;

msp = (1.0 / m + rb ^ 2.0 / J);
msn = (1.0 / m - rb ^ 2.0 / J);
tc = -Km * Kt * G ^ 2.0 / (R * r ^ 2.0);
mp = G * Kt / (R * r);

A = [0 1 0 0; 0 msp*tc 0 msn*tc; 0 0 0 1; 0 msn*tc 0 msp*tc];
B = [0 0; msp * mp msn * mp; 0 0; msn * mp msp * mp];
C = [1 0 0 0; 0 0 1 0];
D = [0 0; 0 0];

dm = c2d(ss(A, B, C, D), dt);

hp = .8;
lp = .85;
K = place(dm.a, dm.b, [hp, hp, lp, lp]);

hlp = 0.07;
llp = 0.09;
L = place(dm.a', dm.c', [hlp, hlp, llp, llp])';

% Plot what we computed

fd = fopen('/home/aschuh/frc971/2012/trunk/src/atom_code/control_loops/Drivetrain.mat', 'w');
n = 1;
sm = [];
writeMatHeader(fd, size(dm.a, 1), size(dm.b, 2));
writeMat(fd, dm.a, 'A');
writeMat(fd, dm.b, 'B');
writeMat(fd, dm.c, 'C');
writeMat(fd, dm.d, 'D');
writeMat(fd, L, 'L');
writeMat(fd, K, 'K');
writeMat(fd, [12; 12], 'U_max');
writeMat(fd, [-12; -12], 'U_min');
writeMatFooter(fd);
fclose(fd);

full_model = dss([dm.a (-dm.b * K); eye(4) (dm.a - dm.b * K - L * dm.c)], [0, 0; 0, 0; 0, 0; 0, 0; L], [C, [0, 0, 0, 0; 0, 0, 0, 0]], 0, eye(8), 0.01);

n = 1;
sm_strait = [];
t = drivetrain_strait_low(1, 1) + dt * (n - 1);
x = [drivetrain_strait_low(1, 2); 0; drivetrain_strait_low(1, 3); 0];
while t < drivetrain_strait_low(end, 1)
    sm_strait(n, 1) = t;
    sm_strait(n, 2) = (x(1,1) + x(3,1)) / 2.0;
    t = t + dt;
    x = dm.a * x + dm.b * [drivetrain_strait_low(n, 4); drivetrain_strait_low(n, 5)];
    n = n + 1;
end

figure;
plot(drivetrain_strait_low(:, 1), (drivetrain_strait_low(:, 2) + drivetrain_strait_low(:, 3)) / 2.0, sm_strait(:, 1), sm_strait(:, 2));
legend('actual', 'sim');

n = 1;
sm_spin = [];
t = drivetrain_spin_low(1, 1) + dt * (n - 1);
x = [drivetrain_spin_low(1, 2); 0; drivetrain_spin_low(1, 3); 0];
while t < drivetrain_spin_low(end, 1)
    sm_spin(n, 1) = t;
    sm_spin(n, 2) = (x(1,1) - x(3,1)) / 2.0;
    t = t + dt;
    x = dm.a * x + dm.b * [drivetrain_spin_low(n, 4); drivetrain_spin_low(n, 5)];
    n = n + 1;
end

figure;
plot(drivetrain_spin_low(:, 1), (drivetrain_spin_low(:, 2) - drivetrain_spin_low(:, 3)) / 2.0, sm_spin(:, 1), sm_spin(:, 2));
legend('actual', 'sim');

%figure;
%nyquist(full_model);


%%
t = 0;
x = [0; 0; 0; 0;];
while t < logging(end, 1)
    sm(n, 1) = t;
    sm(n, 2) = x(1,1);
    sm(n, 3) = x(3,1);
    t = t + dt;
    x = dm.a * x + dm.b * [12.0; 12.0];
    n = n + 1;
end

figure;
plot(logging(:, 1), logging(:, 2), sm(:, 1), sm(:, 2));
legend('actual', 'sim');

%% Simulation of a small turn angle with a large distance to travel
tf = 2;
x = [0; 0; 0.1; 0;];
r = [10; 0; 10; 0];

smt = zeros(tf / dt, 8);
t = 0;
xhat = x;
n = 1;
% 1 means scale
% 2 means just limit to 12 volts
% 3 means preserve the difference in power
captype = 1;
while n <= size(smt, 1)
    smt(n, 1) = t;
    smt(n, 2) = x(1,1);
    smt(n, 3) = x(3,1);
    t = t + dt;
    
    u = K * (r - xhat);
    smt(n, 4) = u(1,1);
    smt(n, 5) = u(2,1);
    
    if captype == 1
        if sum(abs(u) > 12.0)
            % We have a problem!
            % Check to see if it's a big steering power problem,
            % or a big drive error.
            turnPower = (u(1, 1) - u(2, 1));
            drivePower = (u(1, 1) + u(2, 1));
            scaleFactor = 12.0 / max(abs(u));
            smt(n, 8) = 1.0 / scaleFactor;
            % Only start scaling the turn power up if we are far out of
            % range.
            if abs(turnPower) < 0.5 * abs(drivePower)
                % Turn power is swamped.
                deltaTurn = turnPower / 2.0 / scaleFactor * 0.5;
                u(1, 1) = u(1, 1) + deltaTurn;
                u(2, 1) = u(2, 1) - deltaTurn;
                scaleFactor = 12.0 / max(abs(u));
            else
                if 0.5 * abs(turnPower) > abs(drivePower)
                    % Drive power is swamped.
                    deltaDrive = drivePower / 2.0 / scaleFactor * 0.5;
                    u(1, 1) = u(1, 1) + deltaDrive;
                    u(2, 1) = u(2, 1) + deltaDrive;
                    scaleFactor = 12.0 / max(abs(u));
                end
            end
            u = u * scaleFactor;
        end
    else
        if captype == 2
            if u(1, 1) > 12.0
                u(1, 1) = 12.0;
            end
            if u(1, 1) < -12.0
                u(1, 1) = -12.0;
            end
            if u(2, 1) > 12.0
                u(2, 1) = 12.0;
            end
            if u(2, 1) < -12.0
                u(2, 1) = -12.0;
            end
        else
            if captype == 3
                if u(1, 1) > 12.0
                    u(2, 1) = u(2, 1) - (u(1, 1) - 12.0);
                else
                    if u(1, 1) < -12.0
                        u(2, 1) = u(2, 1) - (u(1, 1) + 12.0);
                    end
                end
                if u(2, 1) > 12.0
                    u(1, 1) = u(1, 1) - (u(2, 1) - 12.0);
                else
                    if u(2, 1) < -12.0
                        u(1, 1) = u(1, 1) - (u(2, 1) + 12.0);
                    end
                end
                if u(1, 1) > 12.0
                    u(1, 1) = 12.0;
                end
                if u(1, 1) < -12.0
                    u(1, 1) = -12.0;
                end
                if u(2, 1) > 12.0
                    u(2, 1) = 12.0;
                end
                if u(2, 1) < -12.0
                    u(2, 1) = -12.0;
                end
            end
        end
        
    end
    smt(n, 6) = u(1,1);
    smt(n, 7) = u(2,1);
    xhat = dm.a * xhat + dm.b * u + L * (dm.c * x - dm.c * xhat);
    x = dm.a * x + dm.b * u;
    
    n = n + 1;
end

figure;
subplot(6, 1, 1);
plot(smt(:, 1), smt(:, 2) + smt(:, 3));
legend('dist');
subplot(6, 1, 2);
plot(smt(:, 1), smt(:, 2) - smt(:, 3));
legend('angle');
subplot(3, 1, 2);
plot(smt(:, 1), smt(:, 4), smt(:, 1), smt(:, 5));
legend('lu', 'ru');
subplot(3, 1, 3);
plot(smt(:, 1), smt(:, 6), smt(:, 1), smt(:, 7));
legend('lu_{real}', 'ru_{real}');

%figure;
%plot(smt(:, 1), smt(:, 8))
%legend('Scale Factor');

