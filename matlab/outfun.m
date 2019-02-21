function stop = outfun(x, optimValues, state)
stop = false;
hold on;
plot3(x(1),x(2), x(3), '*');
xlabel('X');
ylabel('Y');
zlabel('Z');
view(3)
drawnow
end