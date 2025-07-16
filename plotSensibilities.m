function plotSensibilities(pstruct,param,name)
close all

percentages =[90, 95, 100, 105, 110];
colors = lines(5);

figure;

%velocity plot
subplot(2,1,1);
hold on;

for i =1:5
    plot(pstruct.(param).time{i}, squeeze(pstruct.(param).velocity{i}),'Color',colors(i,:), 'DisplayName',[num2str(percentages(i)) '%']);
end

title("Bot Geschwindigkeit");
xlabel('Zeit [s]')
ylabel('Geschwindikeit [m/s]')
legend;
grid on;

%position plot
subplot(2,1,2);
hold on;
for i =1:5
    Data = pstruct.(param).position{i};
    plot(Data(:,1), Data(:,2),'Color',colors(i,:), 'DisplayName',[num2str(percentages(i)) '%']);
end

title('Bot-Position');
xlabel('x-Position [m]')
ylabel('y-Position [m]')
legend;
grid on;

sgtitle(['Sensibitaet des Modells auf Abweichungen von ',name], 'Interpreter', 'latex');

end

