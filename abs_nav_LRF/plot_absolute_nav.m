figure(1)
subplot(3,1,1)
hold on; grid on;
plot(tspan,error(:,1)', '-r'); plot(tspan,-3*sta(:,1)', '--b'); plot(tspan,3*sta(:,1)', '--b'); 
xlabel('t / s'); ylabel('X / m');legend('¹À¼ÆÎó²î', '3 sigma');set(gca,'Fontname','Monospaced')
subplot(3,1,2)
hold on; grid on;
plot(tspan,error(:,2)', '-r'); plot(tspan,-3*sta(:,2)', '--b'); plot(tspan,3*sta(:,2)', '--b'); 
xlabel('t / s'); ylabel('X / m');legend('¹À¼ÆÎó²î', '3 sigma');set(gca,'Fontname','Monospaced')
subplot(3,1,3)
hold on; grid on;
plot(tspan,error(:,3)', '-r'); plot(tspan,-3*sta(:,3)', '--b'); plot(tspan,3*sta(:,3)', '--b'); 
xlabel('t / s'); ylabel('X / m');legend('¹À¼ÆÎó²î', '3 sigma');set(gca,'Fontname','Monospaced')

figure(2)
subplot(3,1,1)
hold on; grid on;
plot(tspan,error(:,4)', '-r'); plot(tspan,-3*sta(:,4)', '--b'); plot(tspan,3*sta(:,4)', '--b'); 
xlabel('t / s'); ylabel('X / m');legend('¹À¼ÆÎó²î', '3 sigma');set(gca,'Fontname','Monospaced')
subplot(3,1,2)
hold on; grid on;
plot(tspan,error(:,5)', '-r'); plot(tspan,-3*sta(:,5)', '--b'); plot(tspan,3*sta(:,5)', '--b'); 
xlabel('t / s'); ylabel('X / m');legend('¹À¼ÆÎó²î', '3 sigma');set(gca,'Fontname','Monospaced')
subplot(3,1,3)
hold on; grid on;
plot(tspan,error(:,6)', '-r'); plot(tspan,-3*sta(:,6)', '--b'); plot(tspan,3*sta(:,6)', '--b'); 
xlabel('t / s'); ylabel('X / m');legend('¹À¼ÆÎó²î', '3 sigma');set(gca,'Fontname','Monospaced')