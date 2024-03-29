H = [0.0001, 0, 0, 0;
     0.0, 0.00000001, 0, 0;
     0.0, 0, 0.0000001, 0; 
     0.0, 0, 0, 20];
f = [0; 0; 0; 0];
Aeq = [-22.5403, -1, 0, 0; 
        42.395, 0, -1, 0; 
        6.16137, 0, 0, -1];
beq = [ -6.0542; 
        10.0672; 
        -1.4205];
lb = [-10; -1; -1; -1];
ub = [10; 1; 1; 1];

[wsout, fval, exitflag, output, lambda] = quadprog(H, f, [], [], Aeq, beq, lb, ub);

disp('Optimal solution:');
disp(wsout);
disp('Optimal objective function value:');
disp(fval);

