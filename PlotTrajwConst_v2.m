%% Check if the boundary conditions of alpha and beta can be met
% Plot of BCs and measurements of alpha and beta
figure(1);clf;hold on;
load('output.mat');
load('input.mat');

% Plot alpha and beta measurments
% refernce
plot(rad2deg(pos1(:,1)),rad2deg(pos1(:,2)), 'g--' );
plot(rad2deg(pos2(:,1)),rad2deg(pos2(:,2)), 'g--' );
plot(rad2deg(pos3(:,1)),rad2deg(pos3(:,2)), 'g--' );

%plot(out.y(:,1), out.y(:,2), 'b','LineWidth',1);
plot(rad2deg(y(2,:)), rad2deg(y(3,:)), 'b','LineWidth',1);

% alpha=-15-7.5 h-line
plot([-37.5 -7.5], [-7.5 -7.5], 'r');

% alpha=0 h-line
plot([-7.5 7.5], [-27 -27], 'r');

% alpha=0 v-lines
plot([-7.5 -7.5], [-27 -7.5], 'r');
plot([7.5 7.5], [-27 -7.5], 'r');

% alpha=0-90 traveling h-line
plot([7.5 82.5], [-7.5 -7.5], 'r');

% alpha=90 h-line
plot([82.5 97.5], [-22 -22], 'r');

% alpha=90 v-lines
plot([82.5 82.5],[-22 -7.5], 'r');
plot([97.5 97.5],[-22 -7.5], 'r');

% alpha=90-450 h-line
plot([97.5 442.5], [-7.5 -7.5], 'r');

% alpha=450 h-line
plot([442.5 457.5], [-22 -22], 'r');

% alpha=450 v-lines
plot([442.5 442.5],[-22 -7.5], 'r');
plot([457.5 457.5],[-22 -7.5], 'r');

% alpha=450-457.5 h-line
plot([457.5 480], [-7.5 -7.5], 'r');

% refernce
plot(rad2deg(pos1(:,1)),rad2deg(pos1(:,2)), 'g--' );
plot(rad2deg(pos2(:,1)),rad2deg(pos2(:,2)), 'g--' );
plot(rad2deg(pos3(:,1)),rad2deg(pos3(:,2)), 'g--' );
% make up
grid on;
box on;
xlabel('$\alpha$ [grad]', 'Interpreter','latex');
ylabel('$\beta$ [grad]','Interpreter','latex');
xlim([-37.5,480]);
ylim([-30,5]);
legend('Soll-Verlauf', 'Ist-Verlauf', 'Beschränkung')
%% 
figure(2);  clf; hold on;
plot(y(1,:), rad2deg(y(2,:)));
plot(y(1,:), rad2deg(y(3,:)));
% make up
grid on;
box on;
xlabel('$Zeit$ [s]', 'Interpreter','latex');
legend('$\alpha$ [Grad]','$\beta$ [Grad]','Interpreter','latex');
xlim([0,180]);

%%
figure(3);  clf; hold on;
plot(u(1,:), u(2,:));
plot(u(1,:), u(3,:));
% make up
grid on;
box on;
xlabel('$Zeit$ [s]', 'Interpreter','latex');
legend('Spannung Frontmotor [V]','Spannung Backmotor [V]','Interpreter','latex');
xlim([0,180]);
ylim([-2, 4.5]);